#include "robotpilots/algorithm/kf_filter.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <optional>
#include <utility>

namespace robotpilots::algorithm
{
namespace
{

void RequireShape(const Matrix<float>& matrix, int rows, int columns)
{
    assert(matrix.Rows() == rows && matrix.Cols() == columns);
    static_cast<void>(matrix);
    static_cast<void>(rows);
    static_cast<void>(columns);
}

void ApplyVarianceFloor(Matrix<float>& covariance, const etl::vector<float, 16>& minimum)
{
    for (std::size_t index = 0; index < minimum.size(); ++index)
    {
        covariance[index][index] = std::max(covariance[index][index], minimum[index]);
    }
}

// Cholesky 分解避免 Matrix::Inverted 的增广矩阵超过定长存储容量。
// 创新协方差必须正定；失败时不产生用于更新的逆矩阵。
std::optional<Matrix<float>> InvertInnovation(const Matrix<float>& covariance)
{
    const auto size = covariance.Rows();
    auto lower = Zeros<float>(size, size);
    for (int row = 0; row < size; ++row)
    {
        for (int column = 0; column <= row; ++column)
        {
            float value = covariance[row][column];
            for (int index = 0; index < column; ++index)
            {
                value -= lower[row][index] * lower[column][index];
            }
            if (!std::isfinite(value) || (row == column && value <= 0.0f))
            {
                return std::nullopt;
            }
            lower[row][column] = row == column
                ? std::sqrt(value) : value / lower[column][column];
        }
    }

    auto inverse = Eye<float>(size);
    for (int column = 0; column < size; ++column)
    {
        for (int row = 0; row < size; ++row)
        {
            for (int index = 0; index < row; ++index)
            {
                inverse[row][column] -= lower[row][index] * inverse[index][column];
            }
            inverse[row][column] /= lower[row][row];
        }
        for (int row = size - 1; row >= 0; --row)
        {
            for (int index = row + 1; index < size; ++index)
            {
                inverse[row][column] -= lower[index][row] * inverse[index][column];
            }
            inverse[row][column] /= lower[row][row];
            if (!std::isfinite(inverse[row][column]))
            {
                return std::nullopt;
            }
        }
    }
    return inverse;
}

} // namespace

Kf::Kf(Config config)
    : config_(std::move(config)),
      state_(config_.InitialState),
      covariance_(config_.InitialCovariance)
{
    const auto stateSize = state_.Rows();
    const auto measurementSize = config_.MeasurementModel.Rows();
    assert(stateSize > 0 && stateSize <= 16);
    assert(measurementSize > 0 && measurementSize <= 16);
    RequireShape(state_, stateSize, 1);
    RequireShape(covariance_, stateSize, stateSize);
    RequireShape(config_.StateTransition, stateSize, stateSize);
    RequireShape(config_.ProcessNoise, stateSize, stateSize);
    RequireShape(config_.MeasurementModel, measurementSize, stateSize);
    RequireShape(config_.MeasurementNoise, measurementSize, measurementSize);
    if (!config_.ControlTransition.Empty())
    {
        assert(config_.ControlTransition.Cols() > 0 && config_.ControlTransition.Cols() <= 16);
        RequireShape(config_.ControlTransition, stateSize, config_.ControlTransition.Cols());
    }
    assert(config_.MinimumVariance.empty() ||
           config_.MinimumVariance.size() == static_cast<std::size_t>(stateSize));
    for ([[maybe_unused]] const auto minimum : config_.MinimumVariance)
    {
        assert(std::isfinite(minimum) && minimum >= 0.0f);
    }
    ApplyVarianceFloor(covariance_, config_.MinimumVariance);
}

void Kf::Predict(const Matrix<float>& control)
{
    auto predictedState = config_.StateTransition * state_;
    if (!config_.ControlTransition.Empty())
    {
        RequireShape(control, config_.ControlTransition.Cols(), 1);
        predictedState += config_.ControlTransition * control;
    }
    else
    {
        assert(control.Empty());
    }

    auto predictedCovariance = config_.StateTransition * covariance_ *
        config_.StateTransition.Transposed() + config_.ProcessNoise;
    ApplyVarianceFloor(predictedCovariance, config_.MinimumVariance);
    state_ = std::move(predictedState);
    covariance_ = std::move(predictedCovariance);
}

std::optional<Matrix<float>> Kf::Update(const Matrix<float>& measurement)
{
    RequireShape(measurement, config_.MeasurementModel.Rows(), 1);
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
    auto observation = Zeros<float>(count, state_.Rows());
    auto noise = Zeros<float>(count, count);
    auto values = Zeros<float>(count, 1);
    for (int row = 0; row < count; ++row)
    {
        const auto source = validIndices[row];
        values[row][0] = measurement[source][0];
        for (int column = 0; column < state_.Rows(); ++column)
        {
            observation[row][column] = config_.MeasurementModel[source][column];
        }
        for (int column = 0; column < count; ++column)
        {
            noise[row][column] = config_.MeasurementNoise[source][validIndices[column]];
        }
    }

    const auto crossCovariance = covariance_ * observation.Transposed();
    const auto inverse = InvertInnovation(observation * crossCovariance + noise);
    if (!inverse)
    {
        return std::nullopt;
    }

    const auto gain = crossCovariance * *inverse;
    auto correctedState = state_ + gain * (values - observation * state_);
    const auto residualProjection = Eye<float>(state_.Rows()) - gain * observation;
    // Joseph 形式，避免简单相减破坏协方差的半正定性。
    auto correctedCovariance = residualProjection * covariance_ * residualProjection.Transposed() +
        gain * noise * gain.Transposed();
    correctedCovariance += correctedCovariance.Transposed();
    correctedCovariance *= 0.5f;
    ApplyVarianceFloor(correctedCovariance, config_.MinimumVariance);
    state_ = std::move(correctedState);
    covariance_ = std::move(correctedCovariance);
    return state_;
}

const Matrix<float>& Kf::State() const noexcept
{
    return state_;
}

const Matrix<float>& Kf::Covariance() const noexcept
{
    return covariance_;
}

} // namespace robotpilots::algorithm
