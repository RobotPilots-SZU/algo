#pragma once

#include <algorithm>
#include <cassert>
#include <cmath>
#include <functional>
#include <optional>
#include <utility>

#include <etl/vector.h>

#include "robotpilots/algorithm/matrix.hpp"

namespace robotpilots::algorithm
{
namespace ekf_detail
{

template <std::size_t N>
inline void RequireShape(const Matrix<float, N>& matrix, int rows, int columns)
{
    assert(matrix.Rows() == rows && matrix.Cols() == columns);
    static_cast<void>(matrix);
    static_cast<void>(rows);
    static_cast<void>(columns);
}

// 以下两个函数在 src/ekf_filter.cpp 中实现（非模板），避免大矩阵运算重复内联到每个 TU。
void ApplyVarianceFloorRaw(float* covariance, int size, const etl::vector<float, 16>& minimum);
bool InvertCholesky(const float* covariance, int size, float* inverse);

template <std::size_t N>
inline void ApplyVarianceFloor(Matrix<float, N>& covariance, const etl::vector<float, 16>& minimum)
{
    if (!covariance.Empty())
    {
        ApplyVarianceFloorRaw(covariance.Data(), covariance.Rows(), minimum);
    }
}

// 使用 Cholesky 分解求逆，临时数据只有 16x16 的固定缓冲。
template <std::size_t N>
inline std::optional<Matrix<float, N>> InvertInnovation(const Matrix<float, N>& covariance)
{
    const auto size = covariance.Rows();
    auto inverse = Eye<float, N>(size);
    if (inverse.Empty() || !InvertCholesky(covariance.Data(), size, inverse.Data()))
    {
        return std::nullopt;
    }
    return inverse;
}

} // namespace ekf_detail

/// @brief 扩展卡尔曼滤波器，模型回调只读取当前估计并返回计算结果。
/// @tparam N 各矩阵的元素总数上限(行 x 列)，需覆盖 n x n、m x n、m x m 等中间结果。
template <std::size_t N>
class Ekf final
{
public:
    using StateFunction = std::function<Matrix<float, N>(
        const Matrix<float, N>& state, const Matrix<float, N>& control, float deltaTime)>;
    using MeasurementFunction = std::function<Matrix<float, N>(const Matrix<float, N>& state)>;
    using StateJacobianFunction = StateFunction;
    using MeasurementJacobianFunction = MeasurementFunction;
    using CorrectionFunction = std::function<Matrix<float, N>(
        const Matrix<float, N>& state, const Matrix<float, N>& correction)>;

    /// @brief n 为状态维度，m 为观测维度，均在 [1, 16] 内。
    /// 协方差应对称半正定，参与校正的创新协方差必须正定。
    struct Config
    {
        Matrix<float, N> InitialState;           ///< n x 1。
        Matrix<float, N> InitialCovariance;      ///< n x n。
        Matrix<float, N> ProcessNoise;           ///< n x n，每次 Predict 累加的离散过程噪声。
        Matrix<float, N> MeasurementNoise;       ///< m x m。
        StateFunction StateTransition;       ///< 必填，返回 n x 1 预测状态。
        MeasurementFunction MeasurementModel; ///< 必填，返回 m x 1 预测观测。
        StateJacobianFunction StateJacobian;  ///< 必填，在预测前状态处求值，返回 n x n。
        MeasurementJacobianFunction MeasurementJacobian; ///< 必填，返回 m x n。
        CorrectionFunction ApplyCorrection;   ///< 可选，默认 state + correction；可用于归一化状态。
        std::optional<float> InnovationThreshold; ///< 可选，创新马氏距离平方的非负上限。
        etl::vector<float, 16> MinimumVariance; ///< 可选，n 个非负状态方差下限。
    };

    explicit Ekf(Config config)
        : _config(std::move(config)),
          _state(_config.InitialState),
          _covariance(_config.InitialCovariance)
    {
        const auto stateSize = _state.Rows();
        const auto measurementSize = _config.MeasurementNoise.Rows();
        assert(stateSize > 0 && stateSize <= 16);
        assert(measurementSize > 0 && measurementSize <= 16);
        ekf_detail::RequireShape(_state, stateSize, 1);
        ekf_detail::RequireShape(_covariance, stateSize, stateSize);
        ekf_detail::RequireShape(_config.ProcessNoise, stateSize, stateSize);
        ekf_detail::RequireShape(_config.MeasurementNoise, measurementSize, measurementSize);
        assert(_config.StateTransition && _config.MeasurementModel &&
               _config.StateJacobian && _config.MeasurementJacobian);
        assert(!_config.InnovationThreshold ||
               (std::isfinite(*_config.InnovationThreshold) && *_config.InnovationThreshold >= 0.0f));
        assert(_config.MinimumVariance.empty() ||
               _config.MinimumVariance.size() == static_cast<std::size_t>(stateSize));
        for ([[maybe_unused]] const auto minimum : _config.MinimumVariance)
        {
            assert(std::isfinite(minimum) && minimum >= 0.0f);
        }
        ekf_detail::ApplyVarianceFloor(_covariance, _config.MinimumVariance);
    }

    /// @brief 立即推进状态与协方差；deltaTime 必须有限且非负，control 的形状由模型约定。
    void Predict(float deltaTime, const Matrix<float, N>& control = {})
    {
        assert(std::isfinite(deltaTime) && deltaTime >= 0.0f);
        const auto transition = _config.StateJacobian(_state, control, deltaTime);
        auto predictedState = _config.StateTransition(_state, control, deltaTime);
        ekf_detail::RequireShape(transition, _state.Rows(), _state.Rows());
        ekf_detail::RequireShape(predictedState, _state.Rows(), 1);
        auto predictedCovariance =
            transition * _covariance * transition.Transposed() + _config.ProcessNoise;
        ekf_detail::ApplyVarianceFloor(predictedCovariance, _config.MinimumVariance);
        _state = std::move(predictedState);
        _covariance = std::move(predictedCovariance);
    }

    /// @brief 校正当前估计，不隐式预测；非有限观测分量自动忽略。
    /// 阈值针对本次有效观测的创新距离，不自动随有效观测数调整。
    /// @return 成功时返回校正后的 n x 1 状态副本；无有效观测、创新计算失败或超过阈值时
    /// 返回 std::nullopt，保持当前估计。
    [[nodiscard]] std::optional<Matrix<float, N>> Update(const Matrix<float, N>& measurement)
    {
        _innovationSquared.reset();
        ekf_detail::RequireShape(measurement, _config.MeasurementNoise.Rows(), 1);
        etl::vector<int, 16> validIndices;
        for (int index = 0; index < measurement.Rows(); ++index)
        {
            if (std::isfinite(measurement[index][0]))
            {
                validIndices.push_back(index);
            }
        }
        if (validIndices.empty())
        {
            return std::nullopt;
        }

        const auto expected = _config.MeasurementModel(_state);
        const auto jacobian = _config.MeasurementJacobian(_state);
        ekf_detail::RequireShape(expected, measurement.Rows(), 1);
        ekf_detail::RequireShape(jacobian, measurement.Rows(), _state.Rows());

        const auto count = static_cast<int>(validIndices.size());
        auto observation = Zeros<float, N>(count, _state.Rows());
        auto noise = Zeros<float, N>(count, count);
        auto innovation = Zeros<float, N>(count, 1);
        for (int row = 0; row < count; ++row)
        {
            const auto source = validIndices[row];
            innovation[row][0] = measurement[source][0] - expected[source][0];
            for (int column = 0; column < _state.Rows(); ++column)
            {
                observation[row][column] = jacobian[source][column];
            }
            for (int column = 0; column < count; ++column)
            {
                noise[row][column] = _config.MeasurementNoise[source][validIndices[column]];
            }
        }

        const auto crossCovariance = _covariance * observation.Transposed();
        const auto inverse = ekf_detail::InvertInnovation(observation * crossCovariance + noise);
        if (!inverse)
        {
            return std::nullopt;
        }

        const auto distance = innovation.Transposed() * *inverse * innovation;
        if (!std::isfinite(distance[0][0]) || distance[0][0] < 0.0f)
        {
            return std::nullopt;
        }
        _innovationSquared = distance[0][0];
        if (_config.InnovationThreshold && *_innovationSquared > *_config.InnovationThreshold)
        {
            return std::nullopt;
        }

        const auto gain = crossCovariance * *inverse;
        const auto correction = gain * innovation;
        auto correctedState = _config.ApplyCorrection
            ? _config.ApplyCorrection(_state, correction) : _state + correction;
        ekf_detail::RequireShape(correctedState, _state.Rows(), 1);
        const auto residualProjection = Eye<float, N>(_state.Rows()) - gain * observation;
        // Joseph 形式，独立于是否启用创新检验。
        auto correctedCovariance = residualProjection * _covariance * residualProjection.Transposed() +
            gain * noise * gain.Transposed();
        correctedCovariance += correctedCovariance.Transposed();
        correctedCovariance *= 0.5f;
        ekf_detail::ApplyVarianceFloor(correctedCovariance, _config.MinimumVariance);
        _state = std::move(correctedState);
        _covariance = std::move(correctedCovariance);
        return _state;
    }

    [[nodiscard]] const Matrix<float, N>& State() const noexcept
    {
        return _state;
    }

    [[nodiscard]] const Matrix<float, N>& Covariance() const noexcept
    {
        return _covariance;
    }

    /// @brief 最近一次 Update 的创新距离平方；尚未计算或本次无法计算时为空。
    [[nodiscard]] std::optional<float> InnovationSquared() const noexcept
    {
        return _innovationSquared;
    }

private:
    const Config _config;
    Matrix<float, N> _state;
    Matrix<float, N> _covariance;
    std::optional<float> _innovationSquared;
};

} // namespace robotpilots::algorithm
