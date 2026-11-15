/**
 * @file matrix.hpp
 * @brief 定长矩阵库
 *
 * 主模板是纯头文件的通用实现, 不依赖任何硬件加速库.
 * 定义 ALGO_ENABLE_ARM_MATH 时, Matrix<float, N> 使用 CMSIS-DSP 的显式特化.
 *
 * 模板参数 N 为「元素总数上限」(行 x 列), 必须由使用方显式给出:
 */

#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <utility>

#include <etl/algorithm.h>
#include <etl/vector.h>

#if defined(ALGO_ENABLE_ARM_MATH)
#include "arm_math.h"
#endif

namespace robotpilots::algorithm
{

/// @brief 元素类型 T, 元素总数上限 N(行 x 列)
template <typename T, std::size_t N>
class Matrix;

template <typename T, std::size_t N>
Matrix<T, N> Zeros(int rows, int cols);

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
 * @tparam N 元素总数上限(行 x 列)
 */
template <typename T, std::size_t N>
class Matrix
{
public:
	Matrix() : _rows(0), _cols(0), _data{}
	{
	}

	Matrix(int rows, int cols) : _rows(rows), _cols(cols), _data{}
	{
		_data.resize(Size());
	}

	Matrix(const Matrix& other) : _rows(other._rows), _cols(other._cols), _data{}
	{
		_data.resize(other.Size());
		etl::copy(other._data.data(), other._data.data() + other.Size(), _data.data());
	}

	Matrix(Matrix&& other) noexcept
		: _rows(other._rows), _cols(other._cols), _data(etl::move(other._data))
	{
		other._rows = 0;
		other._cols = 0;
	}

	Matrix& operator=(const Matrix& other)
	{
		if (this == &other)
		{
			return *this;
		}

		_rows = other._rows;
		_cols = other._cols;
		_data.resize(other.Size());
		etl::copy(other._data.data(), other._data.data() + other.Size(), _data.data());
		return *this;
	}

	Matrix& operator=(Matrix&& other) noexcept
	{
		if (this == &other)
		{
			return *this;
		}

		_rows = other._rows;
		_cols = other._cols;
		_data = etl::move(other._data);
		other._rows = 0;
		other._cols = 0;
		return *this;
	}

	~Matrix() = default;

	int Rows() const
	{
		return _rows;
	}

	int Cols() const
	{
		return _cols;
	}

	std::size_t Size() const
	{
		return static_cast<std::size_t>(_rows) * static_cast<std::size_t>(_cols);
	}

	bool Empty() const
	{
		return _rows == 0 || _cols == 0;
	}

	T* Data()
	{
		return _data.data();
	}

	const T* Data() const
	{
		return _data.data();
	}

	T* operator[](int row)
	{
		return _data.data() + row * _cols;
	}

	const T* operator[](int row) const
	{
		return _data.data() + row * _cols;
	}

	Matrix& operator+=(const Matrix& other)
	{
		if (!SameShape(other))
		{
			return *this;
		}

		detail::AddInPlace(_data.data(), other._data.data(), Size());
		return *this;
	}

	Matrix& operator-=(const Matrix& other)
	{
		if (!SameShape(other))
		{
			return *this;
		}

		detail::SubInPlace(_data.data(), other._data.data(), Size());
		return *this;
	}

	template <typename U>
	Matrix& operator*=(const U& value)
	{
		detail::ScaleInPlace(_data.data(), static_cast<T>(value), Size());
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
		if (_cols != other._rows)
		{
			return Matrix();
		}

		Matrix result(_rows, other._cols);
		detail::MultiplyInto(result._data.data(), _data.data(), other._data.data(),
		                     _rows, _cols, other._cols);
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
			startRow + blockRows > _rows || startCol + blockCols > _cols)
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
		return Block(row, 0, 1, _cols);
	}

	Matrix Col(int col) const
	{
		return Block(0, col, _rows, 1);
	}

	Matrix Transposed() const
	{
		Matrix result(_cols, _rows);
		for (int row = 0; row < _rows; ++row)
		{
			for (int col = 0; col < _cols; ++col)
			{
				result[col][row] = (*this)[row][col];
			}
		}
		return result;
	}

	/// @brief 高斯-若尔当消元求逆, 非方阵或奇异返回空矩阵/零矩阵
	Matrix Inverted() const
	{
		if (_rows != _cols)
		{
			return Matrix();
		}

		if constexpr (!std::is_floating_point_v<T>)
		{
			return Matrix();
		}

		Matrix result(_rows, _cols);
		Matrix augmented = Zeros<T>(_rows, _cols * 2);
		for (int row = 0; row < _rows; ++row)
		{
			for (int col = 0; col < _cols; ++col)
			{
				augmented[row][col] = (*this)[row][col];
			}
			augmented[row][_cols + row] = T(1);
		}

		for (int pivot = 0; pivot < _rows; ++pivot)
		{
			int maxRow = pivot;
			T maxValue = std::fabs(augmented[pivot][pivot]);
			for (int row = pivot + 1; row < _rows; ++row)
			{
				if (std::fabs(augmented[row][pivot]) > maxValue)
				{
					maxValue = std::fabs(augmented[row][pivot]);
					maxRow = row;
				}
			}

			if (maxValue < T(1e-9))
			{
				return Zeros<T>(_rows, _cols);
			}

			if (maxRow != pivot)
			{
				for (int col = 0; col < _cols * 2; ++col)
				{
					std::swap(augmented[pivot][col], augmented[maxRow][col]);
				}
			}

			const T divisor = augmented[pivot][pivot];
			for (int col = pivot; col < _cols * 2; ++col)
			{
				augmented[pivot][col] /= divisor;
			}

			for (int row = 0; row < _rows; ++row)
			{
				if (row == pivot)
				{
					continue;
				}

				const T factor = augmented[row][pivot];
				for (int col = pivot; col < _cols * 2; ++col)
				{
					augmented[row][col] -= factor * augmented[pivot][col];
				}
			}
		}

		for (int row = 0; row < _rows; ++row)
		{
			for (int col = 0; col < _cols; ++col)
			{
				result[row][col] = augmented[row][_cols + col];
			}
		}
		return result;
	}

private:
	bool SameShape(const Matrix& other) const
	{
		return _rows == other._rows && _cols == other._cols;
	}

	int _rows;
	int _cols;
	etl::vector<T, N> _data;
};

#if defined(ALGO_ENABLE_ARM_MATH)

namespace detail
{

/// @brief CMSIS 矩阵描述符的预留存储大小
constexpr std::size_t ArmMatrixStorageSize = 32;

/// @brief 把预留的对齐存储当作 CMSIS 矩阵描述符使用
inline arm_matrix_instance_f32& AsArm(void* storage)
{
	return *static_cast<arm_matrix_instance_f32*>(storage);
}

inline const arm_matrix_instance_f32& AsArm(const void* storage)
{
	return *static_cast<const arm_matrix_instance_f32*>(storage);
}

} // namespace detail

/**
 * @brief CMSIS-DSP 加速的 float 矩阵特化
 *
 * N 是模板参数, 成员函数无法再放到 .cpp 中按任意 N 实例化, 因此内联在本头文件,
 * 并在 ALGO_ENABLE_ARM_MATH 时引入 arm_math.h; 类内仍用对齐字节数组承载描述符.
 */
template <std::size_t N>
class Matrix<float, N>
{
public:
	Matrix() : rows_(0), cols_(0), data_{}
	{
		SyncArm();
	}

	Matrix(int rows, int cols) : rows_(rows), cols_(cols), data_{}
	{
		data_.resize(Size());
		SyncArm();
	}

	Matrix(const Matrix& other) : rows_(other.rows_), cols_(other.cols_), data_{}
	{
		data_.resize(other.Size());
		etl::copy(other.data_.data(), other.data_.data() + other.Size(), data_.data());
		SyncArm();
	}

	Matrix(Matrix&& other) noexcept
		: rows_(other.rows_), cols_(other.cols_), data_(etl::move(other.data_))
	{
		SyncArm();
		other.rows_ = 0;
		other.cols_ = 0;
		other.SyncArm();
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
		SyncArm();
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
		SyncArm();
		other.rows_ = 0;
		other.cols_ = 0;
		other.SyncArm();
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

	Matrix& operator+=(const Matrix& other)
	{
		if (rows_ != other.rows_ || cols_ != other.cols_)
		{
			return *this;
		}

		if (arm_mat_add_f32(&detail::AsArm(ArmBuffer()), &detail::AsArm(other.ArmBuffer()),
		                    &detail::AsArm(ArmBuffer())) != ARM_MATH_SUCCESS)
		{
			detail::AddInPlace(data_.data(), other.data_.data(), Size());
		}
		return *this;
	}

	Matrix& operator-=(const Matrix& other)
	{
		if (rows_ != other.rows_ || cols_ != other.cols_)
		{
			return *this;
		}

		if (arm_mat_sub_f32(&detail::AsArm(ArmBuffer()), &detail::AsArm(other.ArmBuffer()),
		                    &detail::AsArm(ArmBuffer())) != ARM_MATH_SUCCESS)
		{
			detail::SubInPlace(data_.data(), other.data_.data(), Size());
		}
		return *this;
	}

	Matrix& operator*=(float value)
	{
		if (arm_mat_scale_f32(&detail::AsArm(ArmBuffer()), value,
		                      &detail::AsArm(ArmBuffer())) != ARM_MATH_SUCCESS)
		{
			detail::ScaleInPlace(data_.data(), value, Size());
		}
		return *this;
	}

	Matrix& operator/=(float value)
	{
		constexpr float epsilon = 1e-6f;
		if (std::abs(value) < epsilon)
		{
			return *this;
		}

		return *this *= static_cast<float>(1) / value;
	}

	Matrix operator+(const Matrix& other) const
	{
		if (rows_ != other.rows_ || cols_ != other.cols_)
		{
			return Matrix();
		}

		Matrix result(*this);
		result += other;
		return result;
	}

	Matrix operator-(const Matrix& other) const
	{
		if (rows_ != other.rows_ || cols_ != other.cols_)
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
		if (arm_mat_mult_f32(&detail::AsArm(ArmBuffer()), &detail::AsArm(other.ArmBuffer()),
		                     &detail::AsArm(result.ArmBuffer())) != ARM_MATH_SUCCESS)
		{
			detail::MultiplyInto(result.data_.data(), data_.data(), other.data_.data(),
			                     rows_, cols_, other.cols_);
		}
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
		if (arm_mat_trans_f32(&detail::AsArm(ArmBuffer()),
		                      &detail::AsArm(result.ArmBuffer())) != ARM_MATH_SUCCESS)
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

	Matrix Inverted() const
	{
		if (rows_ != cols_)
		{
			return Matrix();
		}

		Matrix result(rows_, cols_);
		arm_matrix_instance_f32 destination;
		arm_mat_init_f32(&destination, static_cast<uint16_t>(rows_), static_cast<uint16_t>(cols_),
		                 result.data_.data());

		if (arm_mat_inverse_f32(&detail::AsArm(ArmBuffer()), &destination) != ARM_MATH_SUCCESS)
		{
			return Zeros<float, N>(rows_, cols_);
		}

		result.SyncArm();
		return result;
	}

private:
	/// @brief 让 CMSIS 描述符指向当前数据缓冲区
	void SyncArm()
	{
		static_assert(sizeof(arm_matrix_instance_f32) <= detail::ArmMatrixStorageSize,
		              "CMSIS matrix descriptor exceeds fixed storage");
		static_assert(alignof(arm_matrix_instance_f32) <= alignof(std::max_align_t),
		              "CMSIS matrix descriptor alignment exceeds fixed storage");

		arm_mat_init_f32(&detail::AsArm(ArmBuffer()), static_cast<uint16_t>(rows_),
		                 static_cast<uint16_t>(cols_), data_.data());
	}

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
	etl::vector<float, N> data_;
	alignas(std::max_align_t) std::array<std::uint8_t, detail::ArmMatrixStorageSize> armBuffer_{};
};

#endif // ALGO_ENABLE_ARM_MATH

template <typename T, std::size_t N>
Matrix<T, N> Zeros(int rows, int cols)
{
	Matrix<T, N> result(rows, cols);
	etl::fill(result.Data(), result.Data() + result.Size(), T(0));
	return result;
}

template <typename T, std::size_t N>
Matrix<T, N> Ones(int rows, int cols)
{
	Matrix<T, N> result(rows, cols);
	etl::fill(result.Data(), result.Data() + result.Size(), T(1));
	return result;
}

template <typename T, std::size_t N>
Matrix<T, N> Eye(int dimension)
{
	Matrix<T, N> result = Zeros<T, N>(dimension, dimension);
	for (int i = 0; i < dimension; ++i)
	{
		result[i][i] = T(1);
	}
	return result;
}

template <typename T, std::size_t N>
Matrix<T, N> Trans(const Matrix<T, N>& matrix)
{
	return matrix.Transposed();
}

template <typename T, std::size_t N>
float Trace(const Matrix<T, N>& matrix)
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

template <typename T, std::size_t N>
float Norm(const Matrix<T, N>& matrix)
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

template <typename T, std::size_t N>
Matrix<T, N> Inv(const Matrix<T, N>& matrix)
{
	return matrix.Inverted();
}

template <typename T, std::size_t N>
Matrix<T, N> Hat(const Matrix<T, N>& matrix)
{
	if (matrix.Rows() != 3 || matrix.Cols() != 1)
	{
		return Matrix<T, N>();
	}

	Matrix<T, N> result(3, 3);
	result[0][1] = -matrix[2][0];
	result[0][2] = matrix[1][0];
	result[1][0] = matrix[2][0];
	result[1][2] = -matrix[0][0];
	result[2][0] = -matrix[1][0];
	result[2][1] = matrix[0][0];
	return result;
}

template <typename T, std::size_t N>
Matrix<T, N> Vee(const Matrix<T, N>& matrix)
{
	if (matrix.Rows() != 3 || matrix.Cols() != 3)
	{
		return Matrix<T, N>();
	}

	Matrix<T, N> result(3, 1);
	result[0][0] = matrix[2][1];
	result[1][0] = matrix[0][2];
	result[2][0] = matrix[1][0];
	return result;
}

template <typename T, std::size_t N>
Matrix<T, N> Cross(const Matrix<T, N>& left, const Matrix<T, N>& right)
{
	return Hat(left) * right;
}

} // namespace robotpilots::algorithm
