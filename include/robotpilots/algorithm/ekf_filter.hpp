#pragma once

#include <functional>
#include "robotpilots/algorithm/matrix.hpp"

namespace robotpilots::algorithm
{

/**
 * @brief 扩展卡尔曼滤波算法类
 * 
 */

class Ekf final
{
public:
	Ekf() = default;

	//状态函数F(x，u)
	using StateFunc =
		std::function<Matrix<float>(Matrix<float> &, Matrix<float> &, float, Matrix<float> &, Matrix<float> &)>;

	//观测函数H(x)
	using MeasureFunc = 
		std::function<Matrix<float>(const Matrix<float> &)>;

	//协方差p
	using PFunc = std::function<void(Matrix<float> &)>;
	// 雅可比函数类型
	using JacobianF =
		std::function<void(Matrix<float> &, Matrix<float> &, float, Matrix<float> &)>;

	using JacobianH = std::function<void(const Matrix<float> &, Matrix<float> &)>;

	// 卡方检验
	using Chi = std::function<bool(Matrix<float> &, Matrix<float> &, const Matrix<float> &, const Matrix<float> &, const Matrix<float> &, float )>;

	// 状态估计函数
	using Xhat = std::function<void(Matrix<float> &, const Matrix<float> &, Matrix<float> &,float)>;

	// 卡方更新k
	using Kfunc = std::function<void(Matrix<float> &)>;

	struct Info
	{
		bool isitialized = false;	   ///< 是否完成初始化
		Matrix<float> filtered_value; ///< 滤波输出
	} Ekf_Info;

	// 继承基类初始化结构体
	struct InitParam
	{
		bool Chi_Set = false; ///< 是否设置了卡方检验标志
		StateFunc f;   // 状态转移非线性函数
		MeasureFunc h; // 观测非线性函数
		JacobianF jacF; // 状态雅可比
		JacobianH jacH; //观测雅可比
		PFunc p_func; // 协方差更新函数
		Chi chi;	// 卡方检验函数
		Xhat Xhat_func; // 状态估计函数
		Kfunc k_func; // 增益更新函数
	};

	EAppStatus UpdateHandler_() override;

private:

	float Chi_Square;         //卡方值
	Matrix<float> Chi_Square_Mat; // 卡方检验矩阵
	bool Chi_Set_ = false; ///< 是否设置了卡方检验标志
	bool Chi_Pass = false;    // 卡方检验是否通过

	//后验估计更新函数
	Xhat Xhat_func_;

	//卡方是否通过的函数
	Chi chi_;

	//增益更新函数
	Kfunc k_func_;

	// 非线性函数
	StateFunc f_;

	MeasureFunc h_;

	//雅可比矩阵计算函数
	JacobianF jacF_;

	JacobianH jacH_;

	//协方差更新函数
	PFunc p_func_;

	void Ekf_Predict();
	void Ekf_Update();
	
	/* ========= 工具函数 ========= */

	bool QuaternionEKF_Chi(void);// 卡方验证阈值

	/*=========== end =================*/

	void algo_EKF_jacF_update(void);

	void algo_EKF_jacH_update(void);

	void Algo_EKf_Pminus_Update(void);

	void Algo_EKf_Xhatminus_Update(void);
	
	void Algo_EKf_K_Update(void);
	
	void Algo_EKf_Xhat_Update(void);
	
};

} // namespace robotpilots::algorithm

#endif // ALGO_IMU_EKF_HPP
