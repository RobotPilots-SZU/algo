#if defined(ALGO_ENABLE_ARM_MATH)

#include <new>

#include "arm_math.h"

#include "robotpilots/algorithm/matrix.hpp"

namespace robotpilots::algorithm
{

using ArmMatrix = arm_matrix_instance_f32;

static_assert(sizeof(ArmMatrix) <= detail::ArmMatrixStorageSize,
              "CMSIS matrix descriptor exceeds fixed storage");
static_assert(alignof(ArmMatrix) <= alignof(std::max_align_t),
              "CMSIS matrix descriptor alignment exceeds fixed storage");

static ArmMatrix& AsArm(void* buffer)
{
	return *static_cast<ArmMatrix*>(buffer);
}

static const ArmMatrix& AsArm(const void* buffer)
{
	return *static_cast<const ArmMatrix*>(buffer);
}

void Matrix<float>::SyncArm()
{
	new (ArmBuffer()) ArmMatrix();
	arm_mat_init_f32(&AsArm(ArmBuffer()), static_cast<uint16_t>(rows_),
	                 static_cast<uint16_t>(cols_), data_.data());
}

Matrix<float>::Matrix() : rows_(0), cols_(0), data_{}
{
	SyncArm();
}

Matrix<float>::Matrix(int rows, int cols) : rows_(rows), cols_(cols), data_{}
{
	data_.resize(static_cast<std::size_t>(rows) * static_cast<std::size_t>(cols));
	SyncArm();
}

Matrix<float>::Matrix(const Matrix& other)
	: rows_(other.rows_), cols_(other.cols_), data_{}
{
	data_.resize(other.Size());
	etl::copy(other.data_.data(), other.data_.data() + other.Size(), data_.data());
	SyncArm();
}

Matrix<float>::Matrix(Matrix&& other) noexcept
	: rows_(other.rows_), cols_(other.cols_), data_(etl::move(other.data_))
{
	SyncArm();
	other.rows_ = 0;
	other.cols_ = 0;
	other.SyncArm();
}

Matrix<float>& Matrix<float>::operator=(const Matrix& other)
{
	if (this == &other)
	{
		return *this;
	}

	rows_ = other.rows_;
	cols_ = other.cols_;
	data_.resize(other.Size());
	etl::copy(other.data_.data(), other.data_.data() + other.Size(), data_.data());
	SyncArm();
	return *this;
}

Matrix<float>& Matrix<float>::operator=(Matrix&& other) noexcept
{
	if (this == &other)
	{
		return *this;
	}

	rows_ = other.rows_;
	cols_ = other.cols_;
	data_ = etl::move(other.data_);
	SyncArm();
	other.rows_ = 0;
	other.cols_ = 0;
	other.SyncArm();
	return *this;
}

Matrix<float>& Matrix<float>::operator+=(const Matrix& other)
{
	if (rows_ != other.rows_ || cols_ != other.cols_)
	{
		return *this;
	}

	if (arm_mat_add_f32(&AsArm(ArmBuffer()), &AsArm(other.ArmBuffer()),
	                    &AsArm(ArmBuffer())) != ARM_MATH_SUCCESS)
	{
		detail::AddInPlace(data_.data(), other.data_.data(), Size());
	}
	return *this;
}

Matrix<float>& Matrix<float>::operator-=(const Matrix& other)
{
	if (rows_ != other.rows_ || cols_ != other.cols_)
	{
		return *this;
	}

	if (arm_mat_sub_f32(&AsArm(ArmBuffer()), &AsArm(other.ArmBuffer()),
	                    &AsArm(ArmBuffer())) != ARM_MATH_SUCCESS)
	{
		detail::SubInPlace(data_.data(), other.data_.data(), Size());
	}
	return *this;
}

Matrix<float>& Matrix<float>::operator*=(float value)
{
	if (arm_mat_scale_f32(&AsArm(ArmBuffer()), value, &AsArm(ArmBuffer())) != ARM_MATH_SUCCESS)
	{
		detail::ScaleInPlace(data_.data(), value, Size());
	}
	return *this;
}

Matrix<float>& Matrix<float>::operator/=(float value)
{
	constexpr float epsilon = 1e-6f;
	if (std::abs(value) < epsilon)
	{
		return *this;
	}

	return *this *= static_cast<float>(1) / value;
}

Matrix<float> Matrix<float>::operator+(const Matrix& other) const
{
	if (rows_ != other.rows_ || cols_ != other.cols_)
	{
		return Matrix();
	}

	Matrix result(*this);
	result += other;
	return result;
}

Matrix<float> Matrix<float>::operator-(const Matrix& other) const
{
	if (rows_ != other.rows_ || cols_ != other.cols_)
	{
		return Matrix();
	}

	Matrix result(*this);
	result -= other;
	return result;
}

Matrix<float> Matrix<float>::operator*(const Matrix& other) const
{
	if (cols_ != other.rows_)
	{
		return Matrix();
	}

	Matrix result(rows_, other.cols_);
	if (arm_mat_mult_f32(&AsArm(ArmBuffer()), &AsArm(other.ArmBuffer()),
	                    &AsArm(result.ArmBuffer())) != ARM_MATH_SUCCESS)
	{
		detail::MultiplyInto(result.data_.data(), data_.data(), other.data_.data(),
		                     rows_, cols_, other.cols_);
	}
	return result;
}

Matrix<float> Matrix<float>::operator/(const Matrix& other) const
{
	Matrix inverse = other.Inverted();
	return inverse.Empty() ? Matrix() : *this * inverse;
}

Matrix<float> Matrix<float>::Block(int startRow, int startCol, int blockRows, int blockCols) const
{
	if (startRow < 0 || startCol < 0 || blockRows < 0 || blockCols < 0 ||
		startRow + blockRows > rows_ || startCol + blockCols > cols_)
	{
		return Matrix();
	}

	Matrix result(blockRows, blockCols);
	for (int row = 0; row < blockRows; ++row)
	{
		for (int col = 0; col < blockCols; ++col)
		{
			result[row][col] = (*this)[startRow + row][startCol + col];
		}
	}
	return result;
}

Matrix<float> Matrix<float>::Row(int row) const
{
	return Block(row, 0, 1, cols_);
}

Matrix<float> Matrix<float>::Col(int col) const
{
	return Block(0, col, rows_, 1);
}

Matrix<float> Matrix<float>::Transposed() const
{
	Matrix result(cols_, rows_);
	if (arm_mat_trans_f32(&AsArm(ArmBuffer()), &AsArm(result.ArmBuffer())) != ARM_MATH_SUCCESS)
	{
		for (int row = 0; row < rows_; ++row)
		{
			for (int col = 0; col < cols_; ++col)
			{
				result[col][row] = (*this)[row][col];
			}
		}
	}
	return result;
}

Matrix<float> Matrix<float>::Inverted() const
{
	if (rows_ != cols_)
	{
		return Matrix();
	}

	Matrix result(rows_, cols_);
	ArmMatrix destination;
	arm_mat_init_f32(&destination, static_cast<uint16_t>(rows_), static_cast<uint16_t>(cols_),
	                 result.data_.data());

	const arm_status status = arm_mat_inverse_f32(&AsArm(ArmBuffer()), &destination);
	if (status != ARM_MATH_SUCCESS)
	{
		return Zeros<float>(rows_, cols_);
	}

	result.SyncArm();
	return result;
}

} // namespace robotpilots::algorithm

#endif // ALGO_ENABLE_ARM_MATH
