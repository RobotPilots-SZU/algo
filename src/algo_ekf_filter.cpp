#include "robotpilots/algorithm/algo_ekf_filter.hpp"

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
    this->Ekf_Info.isitialized = true;
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
