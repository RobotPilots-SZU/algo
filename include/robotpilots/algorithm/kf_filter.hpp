#pragma once

#include <optional>

#include <etl/vector.h>

#include "robotpilots/algorithm/matrix.hpp"

namespace robotpilots::algorithm
{

/// @brief 线性卡尔曼滤波器。构造后即可预测或校正，无需注册或初始化。
class Kf final
{
public:
    /// @brief n 为状态维度，m 为观测维度，均在 [1, 16] 内。
    /// 协方差应对称半正定，参与校正的创新协方差必须正定。
    struct Config
    {
        Matrix<float> InitialState;           ///< n x 1。
        Matrix<float> InitialCovariance;      ///< n x n。
        Matrix<float> StateTransition;        ///< n x n，每次 Predict 使用的离散状态转移。
        Matrix<float> ControlTransition;      ///< n x k；无控制输入时留空。
        Matrix<float> ProcessNoise;           ///< n x n，每次 Predict 累加的离散过程噪声。
        Matrix<float> MeasurementModel;       ///< m x n。
        Matrix<float> MeasurementNoise;       ///< m x m，允许相关观测噪声。
        etl::vector<float, 16> MinimumVariance; ///< 可选，n 个非负状态方差下限。
    };

    explicit Kf(Config config);

    /// @brief 立即推进状态与协方差；配置了控制矩阵时必须传入 k x 1 控制量。
    void Predict(const Matrix<float>& control = {});

    /// @brief 用 m x 1 观测校正当前估计，不隐式预测；非有限观测分量自动忽略。
    /// @return 成功时返回校正后的 n x 1 状态副本；无有效观测或创新协方差求逆失败时
    /// 返回 std::nullopt，保持当前估计。
    [[nodiscard]] std::optional<Matrix<float>> Update(const Matrix<float>& measurement);

    [[nodiscard]] const Matrix<float>& State() const noexcept;
    [[nodiscard]] const Matrix<float>& Covariance() const noexcept;

private:
    const Config config_;
    Matrix<float> state_;
    Matrix<float> covariance_;
};

} // namespace robotpilots::algorithm
