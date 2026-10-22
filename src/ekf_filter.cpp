#include "robotpilots/algorithm/ekf_filter.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
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

// 使用 Cholesky 分解，所有临时矩阵均不超过原创新协方差的大小。
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

Ekf::Ekf(Config config)
    : config_(std::move(config)),
      state_(config_.InitialState),
      covariance_(config_.InitialCovariance)
{
    const auto stateSize = state_.Rows();
    const auto measurementSize = config_.MeasurementNoise.Rows();
    assert(stateSize > 0 && stateSize <= 16);
    assert(measurementSize > 0 && measurementSize <= 16);
    RequireShape(state_, stateSize, 1);
    RequireShape(covariance_, stateSize, stateSize);
    RequireShape(config_.ProcessNoise, stateSize, stateSize);
    RequireShape(config_.MeasurementNoise, measurementSize, measurementSize);
    assert(config_.StateTransition && config_.MeasurementModel &&
           config_.StateJacobian && config_.MeasurementJacobian);
    assert(!config_.InnovationThreshold ||
           (std::isfinite(*config_.InnovationThreshold) && *config_.InnovationThreshold >= 0.0f));
    assert(config_.MinimumVariance.empty() ||
           config_.MinimumVariance.size() == static_cast<std::size_t>(stateSize));
    for ([[maybe_unused]] const auto minimum : config_.MinimumVariance)
    {
        assert(std::isfinite(minimum) && minimum >= 0.0f);
    }
    ApplyVarianceFloor(covariance_, config_.MinimumVariance);
}

void Ekf::Predict(float deltaTime, const Matrix<float>& control)
{
    assert(std::isfinite(deltaTime) && deltaTime >= 0.0f);
    const auto transition = config_.StateJacobian(state_, control, deltaTime);
    auto predictedState = config_.StateTransition(state_, control, deltaTime);
    RequireShape(transition, state_.Rows(), state_.Rows());
    RequireShape(predictedState, state_.Rows(), 1);
    auto predictedCovariance = transition * covariance_ * transition.Transposed() + config_.ProcessNoise;
    ApplyVarianceFloor(predictedCovariance, config_.MinimumVariance);
    state_ = std::move(predictedState);
    covariance_ = std::move(predictedCovariance);
}

std::optional<Matrix<float>> Ekf::Update(const Matrix<float>& measurement)
{
    innovationSquared_.reset();
    RequireShape(measurement, config_.MeasurementNoise.Rows(), 1);
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

    const auto expected = config_.MeasurementModel(state_);
    const auto jacobian = config_.MeasurementJacobian(state_);
    RequireShape(expected, measurement.Rows(), 1);
    RequireShape(jacobian, measurement.Rows(), state_.Rows());

    const auto count = static_cast<int>(validIndices.size());
    auto observation = Zeros<float>(count, state_.Rows());
    auto noise = Zeros<float>(count, count);
    auto innovation = Zeros<float>(count, 1);
    for (int row = 0; row < count; ++row)
    {
        const auto source = validIndices[row];
        innovation[row][0] = measurement[source][0] - expected[source][0];
        for (int column = 0; column < state_.Rows(); ++column)
        {
            observation[row][column] = jacobian[source][column];
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

    const auto distance = innovation.Transposed() * *inverse * innovation;
    if (!std::isfinite(distance[0][0]) || distance[0][0] < 0.0f)
    {
        return std::nullopt;
    }
    innovationSquared_ = distance[0][0];
    if (config_.InnovationThreshold && *innovationSquared_ > *config_.InnovationThreshold)
    {
        return std::nullopt;
    }

    const auto gain = crossCovariance * *inverse;
    const auto correction = gain * innovation;
    auto correctedState = config_.ApplyCorrection
        ? config_.ApplyCorrection(state_, correction) : state_ + correction;
    RequireShape(correctedState, state_.Rows(), 1);
    const auto residualProjection = Eye<float>(state_.Rows()) - gain * observation;
    // Joseph 形式，独立于是否启用创新检验。
    auto correctedCovariance = residualProjection * covariance_ * residualProjection.Transposed() +
        gain * noise * gain.Transposed();
    correctedCovariance += correctedCovariance.Transposed();
    correctedCovariance *= 0.5f;
    ApplyVarianceFloor(correctedCovariance, config_.MinimumVariance);
    state_ = std::move(correctedState);
    covariance_ = std::move(correctedCovariance);
    return state_;
}

const Matrix<float>& Ekf::State() const noexcept
{
    return state_;
}

const Matrix<float>& Ekf::Covariance() const noexcept
{
    return covariance_;
}

std::optional<float> Ekf::InnovationSquared() const noexcept
{
    return innovationSquared_;
}

} // namespace robotpilots::algorithm
