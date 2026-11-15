#include "robotpilots/algorithm/kf_filter.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace robotpilots::algorithm::kf_detail
{

namespace
{
// 构造函数已 assert 状态/观测维数不超过 16，因此 Cholesky 的临时下三角矩阵用固定上限。
constexpr int kMaxInnovationSize = 16;
} // namespace

void ApplyVarianceFloorRaw(float* covariance, int size, const etl::vector<float, 16>& minimum)
{
	for (std::size_t index = 0; index < minimum.size() && static_cast<int>(index) < size; ++index)
	{
		float& value = covariance[index * size + index];
		value = std::max(value, minimum[index]);
	}
}

// Cholesky 分解求逆，避免 Matrix::Inverted 的增广矩阵超过 N 的容量；
// 创新协方差必须正定，失败时不产生用于更新的逆矩阵。
bool InvertCholesky(const float* covariance, int size, float* inverse)
{
	if (size <= 0 || size > kMaxInnovationSize)
	{
		return false;
	}

	std::array<float, kMaxInnovationSize * kMaxInnovationSize> lower{};

	for (int row = 0; row < size; ++row)
	{
		for (int column = 0; column <= row; ++column)
		{
			float value = covariance[row * size + column];
			for (int index = 0; index < column; ++index)
			{
				value -= lower[row * size + index] * lower[column * size + index];
			}
			if (!std::isfinite(value) || (row == column && value <= 0.0f))
			{
				return false;
			}
			lower[row * size + column] =
				row == column ? std::sqrt(value) : value / lower[column * size + column];
		}
	}

	for (int column = 0; column < size; ++column)
	{
		for (int row = 0; row < size; ++row)
		{
			for (int index = 0; index < row; ++index)
			{
				inverse[row * size + column] -=
					lower[row * size + index] * inverse[index * size + column];
			}
			inverse[row * size + column] /= lower[row * size + row];
		}
		for (int row = size - 1; row >= 0; --row)
		{
			for (int index = row + 1; index < size; ++index)
			{
				inverse[row * size + column] -=
					lower[index * size + row] * inverse[index * size + column];
			}
			inverse[row * size + column] /= lower[row * size + row];
			if (!std::isfinite(inverse[row * size + column]))
			{
				return false;
			}
		}
	}
	return true;
}

} // namespace robotpilots::algorithm::kf_detail
