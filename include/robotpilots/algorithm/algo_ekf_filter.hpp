#ifndef ALGO_IMU_EKF_HPP
#define ALGO_IMU_EKF_HPP

#include "algo_filter_common.hpp"
#include "algo_kf_filter.hpp"
#include <functional>

namespace robotpilots::algorithm
{

/**
 * @brief 扩展卡尔曼滤波算法类
 * 
 */
template<uint16_t N>
class CAlgo_Ekf final: public CAlgo_Kf<N>
{
public:

	CAlgo_Ekf() = default;
	~CAlgo_Ekf() noexcept = default;

	//状态函数F(x，u)
	using StateFunc =
		std::function<Matrixt<float, N>(Matrixt<float, N> &, Matrixt<float, N> &, float, Matrixt<float, N> &, Matrixt<float, N> &)>;

	//观测函数H(x)
	using MeasureFunc = 
		std::function<Matrixt<float, N>(const Matrixt<float, N> &)>;

	//协方差p
	using PFunc = std::function<void(Matrixt<float, N> &)>;
	// 雅可比函数类型
	using JacobianF =
		std::function<void(Matrixt<float, N> &, Matrixt<float, N> &, float, Matrixt<float, N> &)>;

	using JacobianH = std::function<void(const Matrixt<float, N> &, Matrixt<float, N> &)>;

	// 卡方检验
	using Chi = std::function<bool(Matrixt<float, N> &, Matrixt<float, N> &, const Matrixt<float, N> &, const Matrixt<float, N> &, const Matrixt<float, N> &, float )>;

	// 状态估计函数
	using Xhat = std::function<void(Matrixt<float, N> &, const Matrixt<float, N> &, Matrixt<float, N> &,float)>;

	// 卡方更新k
	using Kfunc = std::function<void(Matrixt<float, N> &)>;

	struct SAlgoEKfInfo
	{
		bool initialized = false;	   ///< 是否完成初始化
		Matrixt<float, N> filtered_value; ///< 滤波输出
	} Ekf_Info;

	// 继承基类初始化结构体
	struct SAlgoEKfInitParam : public CAlgo_Kf<N>::SAlgoKfInitParam
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

	/*main task*/
	EAppStatus InitAlgo_(CFilterBase::SFilterInitParam_Base &param) override;

	EAppStatus UpdateHandler_() override;

private:

	float Chi_Square;         //卡方值
	Matrixt<float, N> Chi_Square_Mat; // 卡方检验矩阵
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

namespace robotpilots::algorithm
{

template<uint16_t N>
EAppStatus CAlgo_Ekf<N>::InitAlgo_(CFilterBase::SFilterInitParam_Base &param)
{
    if (param.AlgoID != EAlgoID::ALGO_IMU_EKF) {
        return APP_ERROR;
    }

    EAppStatus status = CAlgo_Kf<N>::InitAlgo_(param);
    if (status != APP_OK) {
        return status;
    }

    auto &ekf_param = static_cast<SAlgoEKfInitParam&>(param);

    // 校验必需回调是否已设置，避免运行期调用空 std::function 抛异常
    if (!ekf_param.f || !ekf_param.h || !ekf_param.jacF ||
        !ekf_param.jacH || !ekf_param.p_func || !ekf_param.Xhat_func) {
        return APP_ERROR;
    }
    if (ekf_param.Chi_Set && (!ekf_param.chi || !ekf_param.k_func)) {
        return APP_ERROR;
    }

    this->Chi_Set_ = ekf_param.Chi_Set;
    this->f_ = ekf_param.f;
    this->h_ = ekf_param.h;
    this->jacF_ = ekf_param.jacF;
    this->jacH_ = ekf_param.jacH;
    this->p_func_ = ekf_param.p_func;
    this->Xhat_func_ = ekf_param.Xhat_func;
    if (this->Chi_Set_) {
        this->chi_ = ekf_param.chi;
        this->k_func_ = ekf_param.k_func;
    }

    this->Chi_Square_Mat = zeros<float, N>(1, 1);
    this->Chi_Square = 0.0f;
    this->Ekf_Info.initialized = true;
    return APP_OK;
}

template<uint16_t N>
void CAlgo_Ekf<N>::algo_EKF_jacF_update()
{
    this->jacF_(this->xhat, this->u, this->DT, this->F);
    this->FT = trans(this->F);
}

template<uint16_t N>
void CAlgo_Ekf<N>::algo_EKF_jacH_update()
{
    this->jacH_(this->xhatMinus, this->H);
    this->HT = trans(this->H);
}

template<uint16_t N>
void CAlgo_Ekf<N>::Algo_EKf_Xhatminus_Update()
{
    this->xhatMinus = this->f_(this->xhat, this->u, this->DT, this->F, this->B);
}

template<uint16_t N>
void CAlgo_Ekf<N>::Algo_EKf_Pminus_Update()
{
    this->p_func_(this->Pminus);
    CAlgo_Kf<N>::Algo_Kf_Pminus_Update();
}

template<uint16_t N>
void CAlgo_Ekf<N>::Algo_EKf_K_Update(void)
{
    this->K = this->Pminus * this->HT * this->S_inv;
    this->k_func_(this->K);
}

template<uint16_t N>
void CAlgo_Ekf<N>::Algo_EKf_Xhat_Update()
{
    Matrixt<float, N> Correct = this->K * (this->z - this->h_(this->xhatMinus));
    this->Xhat_func_(this->xhat, this->xhatMinus, Correct, this->DT);
}

template<uint16_t N>
bool CAlgo_Ekf<N>::QuaternionEKF_Chi()
{
    Matrixt<float, N> Y = this->z - this->h_(this->xhatMinus);
    this->S = this->H * this->Pminus * this->HT + this->R;
    this->S_inv = inv(this->S);
    this->Chi_Square_Mat = trans(Y) * this->S_inv * Y;
    this->Chi_Square = this->Chi_Square_Mat[0][0];
    return this->chi_(this->K, this->P, this->Pminus, this->H, this->R, this->Chi_Square);
}

template<uint16_t N>
void CAlgo_Ekf<N>::Ekf_Predict()
{
    Algo_EKf_Xhatminus_Update();
    algo_EKF_jacF_update();
    Algo_EKf_Pminus_Update();
}

template<uint16_t N>
void CAlgo_Ekf<N>::Ekf_Update()
{
    algo_EKF_jacH_update();

    if (this->measurement_valid_num_ > 0) {
        if (this->Chi_Set_) {
            this->Chi_Pass = QuaternionEKF_Chi();
            if (this->Chi_Pass) {
                Algo_EKf_K_Update();
                Algo_EKf_Xhat_Update();
                CAlgo_Kf<N>::Algo_Kf_P_Update();
            } else {
                this->xhat = this->xhatMinus;
                this->P = this->Pminus;
            }
        } else {
            Algo_EKf_Xhat_Update();
            CAlgo_Kf<N>::Algo_Kf_P_Update();
        }
    } else {
        this->xhat = this->xhatMinus;
        this->P = this->Pminus;
    }

    this->Kf_Info.filtered_value = this->xhat;
    this->Ekf_Info.filtered_value = this->xhat;
}

template<uint16_t N>
EAppStatus CAlgo_Ekf<N>::UpdateHandler_()
{
    this->z = this->measured_vector_;
    Ekf_Predict();
    Ekf_Update();
    return APP_OK;
}

} // namespace robotpilots::algorithm

#endif // ALGO_IMU_EKF_HPP

