/**
 * @file matrix.hpp
 * @brief 定长矩阵库
 *
 * 主模板是纯头文件的通用实现, 不依赖任何硬件加速库.
 * 定义 ALGO_ENABLE_ARM_MATH 时, Matrix<float> 使用 CMSIS-DSP 的显式特化,
 */

#pragma once

#include <cmath>
#include <cstddef>
#include <type_traits>
#include <utility>

#include <etl/algorithm.h>
#include <etl/vector.h>

namespace robotpilots::algorithm
{

template <typename T>
class Matrix;

template <typename T>
Matrix<T> Zeros(int rows, int cols);

namespace detail
{

template <typename T>
void AddInPlace(T* result, const T* other, std::size_t size)
{
	for (std::size_t i = 0; i < size; ++i)
	{
		result[i] += other[i];
	}
}

template <typename T>
void SubInPlace(T* result, const T* other, std::size_t size)
{
	for (std::size_t i = 0; i < size; ++i)
	{
		result[i] -= other[i];
	}
}

template <typename T>
void ScaleInPlace(T* data, const T& value, std::size_t size)
{
	for (std::size_t i = 0; i < size; ++i)
	{
		data[i] *= value;
	}
}

/// @brief 朴素矩阵乘法, result 为 rows x cols, left 为 rows x inner, right 为 inner x cols
template <typename T>
void MultiplyInto(T* result, const T* left, const T* right, int rows, int inner, int cols)
{
	for (int row = 0; row < rows; ++row)
	{
		for (int col = 0; col < cols; ++col)
		{
			T sum = T(0);
			for (int index = 0; index < inner; ++index)
			{
				sum += left[row * inner + index] * right[index * cols + col];
			}
			result[row * cols + col] = sum;
		}
	}
}

} // namespace detail

/**
 * @brief 通用矩阵(纯头文件实现)
 * @tparam 元素类型T
 */
template <typename T>
class Matrix
{
public:
	Matrix() : rows_(0), cols_(0), data_{}
	{
	}

	Matrix(int rows, int cols) : rows_(rows), cols_(cols), data_{}
	{
		data_.resize(Size());
	}

	Matrix(const Matrix& other) : rows_(other.rows_), cols_(other.cols_), data_{}
	{
		data_.resize(other.Size());
		etl::copy(other.data_.data(), other.data_.data() + other.Size(), data_.data());
	}

	Matrix(Matrix&& other) noexcept
		: rows_(other.rows_), cols_(other.cols_), data_(etl::move(other.data_))
	{
		other.rows_ = 0;
		other.cols_ = 0;
	}

	Matrix& operator=(const Matrix& other)
	{
		if (this == &other)
		{
			return *this;
		}

		rows_ = other.rows_;
		cols_ = other.cols_;
		data_.resize(other.Size());
		etl::copy(other.data_.data(), other.data_.data() + other.Size(), data_.data());
		return *this;
	}

	Matrix& operator=(Matrix&& other) noexcept
	{
		if (this == &other)
		{
			return *this;
		}

		rows_ = other.rows_;
		cols_ = other.cols_;
		data_ = etl::move(other.data_);
		other.rows_ = 0;
		other.cols_ = 0;
		return *this;
	}

	~Matrix() = default;

	int Rows() const
	{
		return rows_;
	}

	int Cols() const
	{
		return cols_;
	}

	std::size_t Size() const
	{
		return static_cast<std::size_t>(rows_) * static_cast<std::size_t>(cols_);
	}

	bool Empty() const
	{
		return rows_ == 0 || cols_ == 0;
	}

	T* Data()
	{
		return data_.data();
	}

	const T* Data() const
	{
		return data_.data();
	}

	T* operator[](int row)
	{
		return data_.data() + row * cols_;
	}

	const T* operator[](int row) const
	{
		return data_.data() + row * cols_;
	}

	Matrix& operator+=(const Matrix& other)
	{
		if (!SameShape(other))
		{
			return *this;
		}

		detail::AddInPlace(data_.data(), other.data_.data(), Size());
		return *this;
	}

	Matrix& operator-=(const Matrix& other)
	{
		if (!SameShape(other))
		{
			return *this;
		}

		detail::SubInPlace(data_.data(), other.data_.data(), Size());
		return *this;
	}

	template <typename U>
	Matrix& operator*=(const U& value)
	{
		detail::ScaleInPlace(data_.data(), static_cast<T>(value), Size());
		return *this;
	}

	template <typename U>
	Matrix& operator/=(const U& value)
	{
		const U epsilon = std::is_floating_point_v<U> ? U(1e-6) : U(0);
		if (std::abs(value) < epsilon)
		{
			return *this;
		}

		return *this *= static_cast<T>(1) / value;
	}

	Matrix operator+(const Matrix& other) const
	{
		if (!SameShape(other))
		{
			return Matrix();
		}

		Matrix result(*this);
		result += other;
		return result;
	}

	Matrix operator-(const Matrix& other) const
	{
		if (!SameShape(other))
		{
			return Matrix();
		}

		Matrix result(*this);
		result -= other;
		return result;
	}

	Matrix operator*(const Matrix& other) const
	{
		if (cols_ != other.rows_)
		{
			return Matrix();
		}

		Matrix result(rows_, other.cols_);
		detail::MultiplyInto(result.data_.data(), data_.data(), other.data_.data(),
		                     rows_, cols_, other.cols_);
		return result;
	}

	Matrix operator/(const Matrix& other) const
	{
		Matrix inverse = other.Inverted();
		return inverse.Empty() ? Matrix() : *this * inverse;
	}

	Matrix Block(int startRow, int startCol, int blockRows, int blockCols) const
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

	Matrix Row(int row) const
	{
		return Block(row, 0, 1, cols_);
	}

	Matrix Col(int col) const
	{
		return Block(0, col, rows_, 1);
	}

	Matrix Transposed() const
	{
		Matrix result(cols_, rows_);
		for (int row = 0; row < rows_; ++row)
		{
			for (int col = 0; col < cols_; ++col)
			{
				result[col][row] = (*this)[row][col];
			}
		}
		return result;
	}

	/// @brief 高斯-若尔当消元求逆, 非方阵或奇异返回空矩阵/零矩阵
	Matrix Inverted() const
	{
		if (rows_ != cols_)
		{
			return Matrix();
		}

		if constexpr (!std::is_floating_point_v<T>)
		{
			return Matrix();
		}

		Matrix result(rows_, cols_);
		Matrix augmented = Zeros<T>(rows_, cols_ * 2);
		for (int row = 0; row < rows_; ++row)
		{
			for (int col = 0; col < cols_; ++col)
			{
				augmented[row][col] = (*this)[row][col];
			}
			augmented[row][cols_ + row] = T(1);
		}

		for (int pivot = 0; pivot < rows_; ++pivot)
		{
			int maxRow = pivot;
			T maxValue = std::fabs(augmented[pivot][pivot]);
			for (int row = pivot + 1; row < rows_; ++row)
			{
				if (std::fabs(augmented[row][pivot]) > maxValue)
				{
					maxValue = std::fabs(augmented[row][pivot]);
					maxRow = row;
				}
			}

			if (maxValue < T(1e-9))
			{
				return Zeros<T>(rows_, cols_);
			}

			if (maxRow != pivot)
			{
				for (int col = 0; col < cols_ * 2; ++col)
				{
					std::swap(augmented[pivot][col], augmented[maxRow][col]);
				}
			}

			const T divisor = augmented[pivot][pivot];
			for (int col = pivot; col < cols_ * 2; ++col)
			{
				augmented[pivot][col] /= divisor;
			}

			for (int row = 0; row < rows_; ++row)
			{
				if (row == pivot)
				{
					continue;
				}

				const T factor = augmented[row][pivot];
				for (int col = pivot; col < cols_ * 2; ++col)
				{
					augmented[row][col] -= factor * augmented[pivot][col];
				}
			}
		}

		for (int row = 0; row < rows_; ++row)
		{
			for (int col = 0; col < cols_; ++col)
			{
				result[row][col] = augmented[row][cols_ + col];
			}
		}
		return result;
	}

private:
	bool SameShape(const Matrix& other) const
	{
		return rows_ == other.rows_ && cols_ == other.cols_;
	}

	int rows_;
	int cols_;
	etl::vector<T, 256> data_;
};

#if defined(ALGO_ENABLE_ARM_MATH)

namespace detail
{

/// @brief CMSIS 矩阵描述符的预留存储大小
constexpr std::size_t ArmMatrixStorageSize = 32;

} // namespace detail

/**
 * @brief CMSIS-DSP 加速的 float 矩阵特化
 *
 * 用一块对齐的字节数组承载 arm_matrix_instance_f32, 头文件不引入 arm_math.h,
 * 所有成员函数在 src/kf_filter.cpp 中定义.
 */
template <>
class Matrix<float>
{
public:
	Matrix();
	Matrix(int rows, int cols);
	Matrix(const Matrix& other);
	Matrix(Matrix&& other) noexcept;
	Matrix& operator=(const Matrix& other);
	Matrix& operator=(Matrix&& other) noexcept;
	~Matrix() = default;

	int Rows() const
	{
		return rows_;
	}

	int Cols() const
	{
		return cols_;
	}

	std::size_t Size() const
	{
		return static_cast<std::size_t>(rows_) * static_cast<std::size_t>(cols_);
	}

	bool Empty() const
	{
		return rows_ == 0 || cols_ == 0;
	}

	float* Data()
	{
		return data_.data();
	}

	const float* Data() const
	{
		return data_.data();
	}

	float* operator[](int row)
	{
		return data_.data() + row * cols_;
	}

	const float* operator[](int row) const
	{
		return data_.data() + row * cols_;
	}

	Matrix& operator+=(const Matrix& other);
	Matrix& operator-=(const Matrix& other);
	Matrix& operator*=(float value);
	Matrix& operator/=(float value);

	Matrix operator+(const Matrix& other) const;
	Matrix operator-(const Matrix& other) const;
	Matrix operator*(const Matrix& other) const;
	Matrix operator/(const Matrix& other) const;

	Matrix Block(int startRow, int startCol, int blockRows, int blockCols) const;
	Matrix Row(int row) const;
	Matrix Col(int col) const;

	Matrix Transposed() const;
	Matrix Inverted() const;

private:
	/// @brief 让 CMSIS 描述符指向当前数据缓冲区
	void SyncArm();

	void* ArmBuffer()
	{
		return armBuffer_.data();
	}

	const void* ArmBuffer() const
	{
		return armBuffer_.data();
	}

	int rows_;
	int cols_;
	etl::vector<float, 256> data_;
	alignas(std::max_align_t) std::array<std::uint8_t, detail::ArmMatrixStorageSize> armBuffer_{};
};

#endif // ALGO_ENABLE_ARM_MATH

template <typename T>
Matrix<T> Zeros(int rows, int cols)
{
	Matrix<T> result(rows, cols);
	etl::fill(result.Data(), result.Data() + result.Size(), T(0));
	return result;
}

template <typename T>
Matrix<T> Ones(int rows, int cols)
{
	Matrix<T> result(rows, cols);
	etl::fill(result.Data(), result.Data() + result.Size(), T(1));
	return result;
}

template <typename T>
Matrix<T> Eye(int dimension)
{
	Matrix<T> result = Zeros<T>(dimension, dimension);
	for (int i = 0; i < dimension; ++i)
	{
		result[i][i] = T(1);
	}
	return result;
}

template <typename T>
Matrix<T> Trans(const Matrix<T>& matrix)
{
	return matrix.Transposed();
}

template <typename T>
float Trace(const Matrix<T>& matrix)
{
	if (matrix.Rows() != matrix.Cols())
	{
		return 0.0f;
	}

	float result = 0.0f;
	for (int i = 0; i < matrix.Rows(); ++i)
	{
		result += matrix[i][i];
	}
	return result;
}

template <typename T>
float Norm(const Matrix<T>& matrix)
{
	if (!std::is_floating_point_v<T> || (matrix.Rows() != 1 && matrix.Cols() != 1))
	{
		return 0.0f;
	}

	T sum = T(0);
	for (std::size_t i = 0; i < matrix.Size(); ++i)
	{
		sum += matrix.Data()[i] * matrix.Data()[i];
	}
	return std::sqrt(sum);
}

template <typename T>
Matrix<T> Inv(const Matrix<T>& matrix)
{
	return matrix.Inverted();
}

template <typename T>
Matrix<T> Hat(const Matrix<T>& matrix)
{
	if (matrix.Rows() != 3 || matrix.Cols() != 1)
	{
		return Matrix<T>();
	}

	Matrix<T> result(3, 3);
	result[0][1] = -matrix[2][0];
	result[0][2] = matrix[1][0];
	result[1][0] = matrix[2][0];
	result[1][2] = -matrix[0][0];
	result[2][0] = -matrix[1][0];
	result[2][1] = matrix[0][0];
	return result;
}

template <typename T>
Matrix<T> Vee(const Matrix<T>& matrix)
{
	if (matrix.Rows() != 3 || matrix.Cols() != 3)
	{
		return Matrix<T>();
	}

	Matrix<T> result(3, 1);
	result[0][0] = matrix[2][1];
	result[1][0] = matrix[0][2];
	result[2][0] = matrix[1][0];
	return result;
}

template <typename T>
Matrix<T> Cross(const Matrix<T>& left, const Matrix<T>& right)
{
	return Hat(left) * right;
}

} // namespace robotpilots::algorithm
