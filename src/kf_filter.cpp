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
    : _config(std::move(config)),
      _state(_config.InitialState),
      _covariance(_config.InitialCovariance)
{
    const auto stateSize = _state.Rows();
    const auto measurementSize = _config.MeasurementModel.Rows();
    assert(stateSize > 0 && stateSize <= 16);
    assert(measurementSize > 0 && measurementSize <= 16);
    RequireShape(_state, stateSize, 1);
    RequireShape(_covariance, stateSize, stateSize);
    RequireShape(_config.StateTransition, stateSize, stateSize);
    RequireShape(_config.ProcessNoise, stateSize, stateSize);
    RequireShape(_config.MeasurementModel, measurementSize, stateSize);
    RequireShape(_config.MeasurementNoise, measurementSize, measurementSize);
    if (!_config.ControlTransition.Empty())
    {
        assert(_config.ControlTransition.Cols() > 0 && _config.ControlTransition.Cols() <= 16);
        RequireShape(_config.ControlTransition, stateSize, _config.ControlTransition.Cols());
    }
    assert(_config.MinimumVariance.empty() ||
           _config.MinimumVariance.size() == static_cast<std::size_t>(stateSize));
    for ([[maybe_unused]] const auto minimum : _config.MinimumVariance)
    {
        assert(std::isfinite(minimum) && minimum >= 0.0f);
    }
    ApplyVarianceFloor(_covariance, _config.MinimumVariance);
}

void Kf::Predict(const Matrix<float>& control)
{
    auto predictedState = _config.StateTransition * _state;
    if (!_config.ControlTransition.Empty())
    {
        RequireShape(control, _config.ControlTransition.Cols(), 1);
        predictedState += _config.ControlTransition * control;
    }
    else
    {
        assert(control.Empty());
    }

    auto predictedCovariance = _config.StateTransition * _covariance *
        _config.StateTransition.Transposed() + _config.ProcessNoise;
    ApplyVarianceFloor(predictedCovariance, _config.MinimumVariance);
    _state = std::move(predictedState);
    _covariance = std::move(predictedCovariance);
}

std::optional<Matrix<float>> Kf::Update(const Matrix<float>& measurement)
{
    RequireShape(measurement, _config.MeasurementModel.Rows(), 1);
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
    auto observation = Zeros<float>(count, _state.Rows());
    auto noise = Zeros<float>(count, count);
    auto values = Zeros<float>(count, 1);
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
    const auto inverse = InvertInnovation(observation * crossCovariance + noise);
    if (!inverse)
    {
        return std::nullopt;
    }

    const auto gain = crossCovariance * *inverse;
    auto correctedState = _state + gain * (values - observation * _state);
    const auto residualProjection = Eye<float>(_state.Rows()) - gain * observation;
    // Joseph 形式，避免简单相减破坏协方差的半正定性。
    auto correctedCovariance = residualProjection * _covariance * residualProjection.Transposed() +
        gain * noise * gain.Transposed();
    correctedCovariance += correctedCovariance.Transposed();
    correctedCovariance *= 0.5f;
    ApplyVarianceFloor(correctedCovariance, _config.MinimumVariance);
    _state = std::move(correctedState);
    _covariance = std::move(correctedCovariance);
    return _state;
}

const Matrix<float>& Kf::State() const noexcept
{
    return _state;
}

const Matrix<float>& Kf::Covariance() const noexcept
{
    return _covariance;
}

} // namespace robotpilots::algorithm
