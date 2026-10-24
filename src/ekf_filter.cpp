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
    : _config(std::move(config)),
      _state(_config.InitialState),
      _covariance(_config.InitialCovariance)
{
    const auto stateSize = _state.Rows();
    const auto measurementSize = _config.MeasurementNoise.Rows();
    assert(stateSize > 0 && stateSize <= 16);
    assert(measurementSize > 0 && measurementSize <= 16);
    RequireShape(_state, stateSize, 1);
    RequireShape(_covariance, stateSize, stateSize);
    RequireShape(_config.ProcessNoise, stateSize, stateSize);
    RequireShape(_config.MeasurementNoise, measurementSize, measurementSize);
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
    ApplyVarianceFloor(_covariance, _config.MinimumVariance);
}

void Ekf::Predict(float deltaTime, const Matrix<float>& control)
{
    assert(std::isfinite(deltaTime) && deltaTime >= 0.0f);
    const auto transition = _config.StateJacobian(_state, control, deltaTime);
    auto predictedState = _config.StateTransition(_state, control, deltaTime);
    RequireShape(transition, _state.Rows(), _state.Rows());
    RequireShape(predictedState, _state.Rows(), 1);
    auto predictedCovariance = transition * _covariance * transition.Transposed() + _config.ProcessNoise;
    ApplyVarianceFloor(predictedCovariance, _config.MinimumVariance);
    _state = std::move(predictedState);
    _covariance = std::move(predictedCovariance);
}

std::optional<Matrix<float>> Ekf::Update(const Matrix<float>& measurement)
{
    _innovationSquared.reset();
    RequireShape(measurement, _config.MeasurementNoise.Rows(), 1);
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
    RequireShape(expected, measurement.Rows(), 1);
    RequireShape(jacobian, measurement.Rows(), _state.Rows());

    const auto count = static_cast<int>(validIndices.size());
    auto observation = Zeros<float>(count, _state.Rows());
    auto noise = Zeros<float>(count, count);
    auto innovation = Zeros<float>(count, 1);
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
    _innovationSquared = distance[0][0];
    if (_config.InnovationThreshold && *_innovationSquared > *_config.InnovationThreshold)
    {
        return std::nullopt;
    }

    const auto gain = crossCovariance * *inverse;
    const auto correction = gain * innovation;
    auto correctedState = _config.ApplyCorrection
        ? _config.ApplyCorrection(_state, correction) : _state + correction;
    RequireShape(correctedState, _state.Rows(), 1);
    const auto residualProjection = Eye<float>(_state.Rows()) - gain * observation;
    // Joseph 形式，独立于是否启用创新检验。
    auto correctedCovariance = residualProjection * _covariance * residualProjection.Transposed() +
        gain * noise * gain.Transposed();
    correctedCovariance += correctedCovariance.Transposed();
    correctedCovariance *= 0.5f;
    ApplyVarianceFloor(correctedCovariance, _config.MinimumVariance);
    _state = std::move(correctedState);
    _covariance = std::move(correctedCovariance);
    return _state;
}

const Matrix<float>& Ekf::State() const noexcept
{
    return _state;
}

const Matrix<float>& Ekf::Covariance() const noexcept
{
    return _covariance;
}

std::optional<float> Ekf::InnovationSquared() const noexcept
{
    return _innovationSquared;
}

} // namespace robotpilots::algorithm
