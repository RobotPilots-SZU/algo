#pragma once

#include <algorithm>
#include <cassert>
#include <cmath>
#include <optional>
#include <utility>

#include <etl/vector.h>

#include "robotpilots/algorithm/matrix.hpp"

namespace robotpilots::algorithm
{
namespace kf_detail
{

template <std::size_t N>
inline void RequireShape(const Matrix<float, N>& matrix, int rows, int columns)
{
    assert(matrix.Rows() == rows && matrix.Cols() == columns);
    static_cast<void>(matrix);
    static_cast<void>(rows);
    static_cast<void>(columns);
}

// 以下两个函数在 src/kf_filter.cpp 中实现（非模板），避免大矩阵运算重复内联到每个 TU。
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

// Cholesky 分解求逆，避免 Matrix::Inverted 的增广矩阵超过 N 的容量；
// 创新协方差必须正定，失败时不产生用于更新的逆矩阵。
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

} // namespace kf_detail

/// @brief 线性卡尔曼滤波器。构造后即可预测或校正，无需注册或初始化。
/// @tparam N 各矩阵的元素总数上限(行 x 列)，需覆盖 n x n、m x n、m x m 等中间结果。
template <std::size_t N>
class Kf final
{
public:
    /// @brief n 为状态维度，m 为观测维度，均在 [1, 16] 内。
    /// 协方差应对称半正定，参与校正的创新协方差必须正定。
    struct Config
    {
        Matrix<float, N> InitialState;           ///< n x 1。
        Matrix<float, N> InitialCovariance;      ///< n x n。
        Matrix<float, N> StateTransition;        ///< n x n，每次 Predict 使用的离散状态转移。
        Matrix<float, N> ControlTransition;      ///< n x k；无控制输入时留空。
        Matrix<float, N> ProcessNoise;           ///< n x n，每次 Predict 累加的离散过程噪声。
        Matrix<float, N> MeasurementModel;       ///< m x n。
        Matrix<float, N> MeasurementNoise;       ///< m x m，允许相关观测噪声。
        etl::vector<float, 16> MinimumVariance; ///< 可选，n 个非负状态方差下限。
    };

    explicit Kf(Config config)
        : _config(std::move(config)),
          _state(_config.InitialState),
          _covariance(_config.InitialCovariance)
    {
        const auto stateSize = _state.Rows();
        const auto measurementSize = _config.MeasurementModel.Rows();
        assert(stateSize > 0 && stateSize <= 16);
        assert(measurementSize > 0 && measurementSize <= 16);
        kf_detail::RequireShape(_state, stateSize, 1);
        kf_detail::RequireShape(_covariance, stateSize, stateSize);
        kf_detail::RequireShape(_config.StateTransition, stateSize, stateSize);
        kf_detail::RequireShape(_config.ProcessNoise, stateSize, stateSize);
        kf_detail::RequireShape(_config.MeasurementModel, measurementSize, stateSize);
        kf_detail::RequireShape(_config.MeasurementNoise, measurementSize, measurementSize);
        if (!_config.ControlTransition.Empty())
        {
            assert(_config.ControlTransition.Cols() > 0 && _config.ControlTransition.Cols() <= 16);
            kf_detail::RequireShape(_config.ControlTransition, stateSize,
                                    _config.ControlTransition.Cols());
        }
        assert(_config.MinimumVariance.empty() ||
               _config.MinimumVariance.size() == static_cast<std::size_t>(stateSize));
        for ([[maybe_unused]] const auto minimum : _config.MinimumVariance)
        {
            assert(std::isfinite(minimum) && minimum >= 0.0f);
        }
        kf_detail::ApplyVarianceFloor(_covariance, _config.MinimumVariance);
    }

    /// @brief 立即推进状态与协方差；配置了控制矩阵时必须传入 k x 1 控制量。
    void Predict(const Matrix<float, N>& control = {})
    {
        auto predictedState = _config.StateTransition * _state;
        if (!_config.ControlTransition.Empty())
        {
            kf_detail::RequireShape(control, _config.ControlTransition.Cols(), 1);
            predictedState += _config.ControlTransition * control;
        }
        else
        {
            assert(control.Empty());
        }

        auto predictedCovariance = _config.StateTransition * _covariance *
            _config.StateTransition.Transposed() + _config.ProcessNoise;
        kf_detail::ApplyVarianceFloor(predictedCovariance, _config.MinimumVariance);
        _state = std::move(predictedState);
        _covariance = std::move(predictedCovariance);
    }

    /// @brief 用 m x 1 观测校正当前估计，不隐式预测；非有限观测分量自动忽略。
    /// @return 成功时返回校正后的 n x 1 状态副本；无有效观测或创新协方差求逆失败时
    /// 返回 std::nullopt，保持当前估计。
    [[nodiscard]] std::optional<Matrix<float, N>> Update(const Matrix<float, N>& measurement)
    {
        kf_detail::RequireShape(measurement, _config.MeasurementModel.Rows(), 1);
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

        const auto count = static_cast<int>(validIndices.size());
        auto observation = Zeros<float, N>(count, _state.Rows());
        auto noise = Zeros<float, N>(count, count);
        auto values = Zeros<float, N>(count, 1);
        for (int row = 0; row < count; ++row)
        {
            const auto source = validIndices[row];
            values[row][0] = measurement[source][0];
            for (int column = 0; column < _state.Rows(); ++column)
            {
                observation[row][column] = _config.MeasurementModel[source][column];
            }
            for (int column = 0; column < count; ++column)
            {
                noise[row][column] = _config.MeasurementNoise[source][validIndices[column]];
            }
        }

        const auto crossCovariance = _covariance * observation.Transposed();
        const auto inverse = kf_detail::InvertInnovation(observation * crossCovariance + noise);
        if (!inverse)
        {
            return std::nullopt;
        }

        const auto gain = crossCovariance * *inverse;
        auto correctedState = _state + gain * (values - observation * _state);
        const auto residualProjection = Eye<float, N>(_state.Rows()) - gain * observation;
        // Joseph 形式，避免简单相减破坏协方差的半正定性。
        auto correctedCovariance = residualProjection * _covariance * residualProjection.Transposed() +
            gain * noise * gain.Transposed();
        correctedCovariance += correctedCovariance.Transposed();
        correctedCovariance *= 0.5f;
        kf_detail::ApplyVarianceFloor(correctedCovariance, _config.MinimumVariance);
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

private:
    const Config _config;
    Matrix<float, N> _state;
    Matrix<float, N> _covariance;
};

} // namespace robotpilots::algorithm
