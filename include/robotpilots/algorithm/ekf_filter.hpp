#pragma once

#include <functional>
#include <optional>

#include <etl/vector.h>

#include "robotpilots/algorithm/matrix.hpp"

namespace robotpilots::algorithm
{

/// @brief 扩展卡尔曼滤波器，模型回调只读取当前估计并返回计算结果。
class Ekf final
{
public:
    using StateFunction = std::function<Matrix<float>(
        const Matrix<float>& state, const Matrix<float>& control, float deltaTime)>;
    using MeasurementFunction = std::function<Matrix<float>(const Matrix<float>& state)>;
    using StateJacobianFunction = StateFunction;
    using MeasurementJacobianFunction = MeasurementFunction;
    using CorrectionFunction = std::function<Matrix<float>(
        const Matrix<float>& state, const Matrix<float>& correction)>;

    /// @brief n 为状态维度，m 为观测维度，均在 [1, 16] 内。
    /// 协方差应对称半正定，参与校正的创新协方差必须正定。
    struct Config
    {
        Matrix<float> InitialState;           ///< n x 1。
        Matrix<float> InitialCovariance;      ///< n x n。
        Matrix<float> ProcessNoise;           ///< n x n，每次 Predict 累加的离散过程噪声。
        Matrix<float> MeasurementNoise;       ///< m x m。
        StateFunction StateTransition;       ///< 必填，返回 n x 1 预测状态。
        MeasurementFunction MeasurementModel; ///< 必填，返回 m x 1 预测观测。
        StateJacobianFunction StateJacobian;  ///< 必填，在预测前状态处求值，返回 n x n。
        MeasurementJacobianFunction MeasurementJacobian; ///< 必填，返回 m x n。
        CorrectionFunction ApplyCorrection;   ///< 可选，默认 state + correction；可用于归一化状态。
        std::optional<float> InnovationThreshold; ///< 可选，创新马氏距离平方的非负上限。
        etl::vector<float, 16> MinimumVariance; ///< 可选，n 个非负状态方差下限。
    };

    explicit Ekf(Config config);

    /// @brief 立即推进状态与协方差；deltaTime 必须有限且非负，control 的形状由模型约定。
    void Predict(float deltaTime, const Matrix<float>& control = {});

    /// @brief 校正当前估计，不隐式预测；非有限观测分量自动忽略。
    /// 阈值针对本次有效观测的创新距离，不自动随有效观测数调整。
    /// @return 成功时返回校正后的 n x 1 状态副本；无有效观测、创新计算失败或超过阈值时
    /// 返回 std::nullopt，保持当前估计。
    [[nodiscard]] std::optional<Matrix<float>> Update(const Matrix<float>& measurement);

    [[nodiscard]] const Matrix<float>& State() const noexcept;
    [[nodiscard]] const Matrix<float>& Covariance() const noexcept;

    /// @brief 最近一次 Update 的创新距离平方；尚未计算或本次无法计算时为空。
    [[nodiscard]] std::optional<float> InnovationSquared() const noexcept;

private:
    const Config _config;
    Matrix<float> _state;
    Matrix<float> _covariance;
    std::optional<float> _innovationSquared;
};

} // namespace robotpilots::algorithm
