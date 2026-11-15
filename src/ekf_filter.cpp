#include "robotpilots/algorithm/ekf_filter.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace robotpilots::algorithm::ekf_detail
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

// 使用 Cholesky 分解求逆，所有临时数据不超过原创新协方差的大小。
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

} // namespace robotpilots::algorithm::ekf_detail
