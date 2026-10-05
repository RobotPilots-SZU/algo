/**
 * @file algo_kf_filter.hpp
 * @author sllllr (2997708711@qq.com)
 * @brief 卡尔曼滤波
 * @version 1.0
 * @date 2026-01-17
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef ALGO_KF_FILTER_HPP
#define ALGO_KF_FILTER_HPP

#include "algo_filter_common.hpp"
#include "rp_matrix.hpp"

namespace robotpilots::algorithm{

/**
 * @brief 卡尔曼滤波算法类
 * 
 */
template<uint16_t N>
class CAlgo_Kf: public CFilterBase{
public:
    // 继承基类初始化结构体
    struct SAlgoKfInitParam : public SFilterInitParam_Base{
        
        float_t DT = 0.0f;                          ///< 调度周期
        uint8_t z_size = 0;                         ///< 观测量维度
        uint8_t u_size = 0;                         ///< 输入值维度
        uint8_t x_size = 0;                         ///< 状态量维度
        bool use_auto_adjustment = false;               ///< 是否开启H, R, K矩阵的动态调整
        etl::vector<uint8_t, 16> measurement_map;           ///< 观测量到状态量的映射
        etl::vector<float, 16> measurement_degree;          ///< 观测量与状态量的缩放关系(用于构建H)
        etl::vector<float, 16> r_diagonal_elements;         ///< R矩阵的对角线元素(用于构建R)
        etl::vector<float, 16> state_min_variance;          ///< P矩阵对角线元素的最小值，防止过度收敛
    };
    
    // 卡尔曼滤波信息结构体+实例
    struct SAlgoKfInfo{
        bool isitialized = false;                   ///< 是否完成初始化
        Matrixt<float, N> filtered_value;              ///< 滤波输出
    } Kf_Info;

    
    CAlgo_Kf() = default;  ///< 默认构造函数

    explicit CAlgo_Kf(SFilterInitParam_Base &param){
        InitAlgo_(param);
    }   ///< 带参的构造函数，用初始化结构体构造

    // 模块析构函数
	~CAlgo_Kf() noexcept = default;

    /**
     * @brief 设置状态转移矩阵
     * @param 状态转移矩阵F
     * 
     */
    void Set_F(const Matrixt<float, N>& F) { this->F = F; this->FT = trans(this->F);}

    /**
     * @brief 设置控制矩阵
     * @param 控制矩阵B
     * 
     */
    void Set_B(const Matrixt<float, N>& B) { this->B = B; }

    /**
     * @brief 设置过程噪声协方差矩阵
     * @param 过程噪声协方差矩阵Q
     * 
     */
    void Set_Q(const Matrixt<float, N>& Q) { this->Q = Q; }

    void Set_H(const Matrixt<float, N>& H_in);
    void Set_R(const Matrixt<float, N>& R_in);

    /**
     * @brief 设置原始测量向量 (在读取传感器数据处调用)
     * @param 所有传感器的最新读数 measured_vector
     */
    void Set_Measured_Vector(const Matrixt<float, N>& measured_vector) {
        if (measured_vector.rows() == z_size_ && measured_vector.cols() == 1) {
            this->measured_vector_ = measured_vector;
        }
    }

    /**
     * @brief 设置初始状态
     * @param 状态估计向量xhat_in
     */
    void Set_xhat(const Matrixt<float, N>& xhat_in) { this->xhat = xhat_in; }

    /**
     * @brief 设置调度周期
     * @param 调度周期dt
     */
    void Set_DT(float dt) { this->DT = dt; }

    /**
     * @brief 设置后验估计协方差矩阵，使系统刚开始更快收敛
     * @param 后验估计协方差矩阵P
     */
    void Set_P(const Matrixt<float, N>& P_in) { this->P = P_in; }
    
    virtual EAppStatus InitAlgo_(SFilterInitParam_Base &param) ;   ///< 初始化

    virtual EAppStatus UpdateHandler_() ;    ///< 更新

protected:
    float_t DT = 0.0f; ///< 调度周期
    // Matrixs
    Matrixt<float, N> xhat;            ///< 后验最优估计
    Matrixt<float, N> xhatMinus;       ///< 先验估计
    Matrixt<float, N> Pminus;          ///< 先验估计协方差矩阵
    Matrixt<float, N> P;               ///< 后验估计协方差矩阵
    Matrixt<float, N> F, FT;           ///< 状态转移矩阵及其转置
    Matrixt<float, N> B;               ///< 控制矩阵
    Matrixt<float, N> Q;               ///< 过程噪声协方差矩阵
    Matrixt<float, N> H, HT;           ///< 观测矩阵及其转置
    Matrixt<float, N> R;               ///< 测量噪声协方差矩阵
    Matrixt<float, N> K;               ///< 卡尔曼增益矩阵
    Matrixt<float, N> S;               ///< 观测残差协方差矩阵
    Matrixt<float, N> S_inv;           
    Matrixt<float, N> z;               ///< 观测向量
    Matrixt<float, N> u;               ///< 输入向量

    // Size
    uint8_t u_size_;                ///< 输入量维度
    uint8_t x_size_;                ///< 状态量维度
    uint8_t z_size_;                ///< 观测量维度

    // Auto Adjustment related
    bool use_auto_adjustment_;                      ///< 是否启用动态调整
    uint8_t measurement_valid_num_;                 ///< 当前周期有效测量数
    Matrixt<float, N> measured_vector_;                ///< 原始测量向量(由传感器填充)
    etl::vector<uint8_t, 16> measurement_map_;          ///< 观测量->状态量缩放倍数
    etl::vector<float, 16> measurement_degree_;         ///< 观测矩阵H构建系数
    etl::vector<float, 16> r_diagonal_elements_;        ///< 过程误差R构建系数
    etl::vector<float, 16> state_min_variance_;         ///< P的最小方差

    void Algo_Kf_Xhatminus_Update(void);
    void Algo_Kf_Pminus_Update(void);
    void Algo_Kf_K_Update(void);
    void Algo_Kf_Xhat_Update(void);
    void Algo_Kf_P_Update(void);
    void Algo_Kf_Adjustment(void);

};
    
} // namespace robotpilots::algorithm

#define deg2rad(x) ((x) * 0.017453292519943295769236907684886)
#define rad2deg(x) ((x) * 57.295779513082320876798154814105)

namespace robotpilots::algorithm{
    
/**
 * @brief 初始化卡尔曼滤波
 * @retval EAppStatus
 * 
 */
template<uint16_t N>
EAppStatus CAlgo_Kf<N>::InitAlgo_(SFilterInitParam_Base &param){

    // 检查param是否正确
    if (param.AlgoID != EAlgoID::ALGO_KF && param.AlgoID != EAlgoID::ALGO_IMU_EKF) {
        return APP_ERROR;
    }

    // 类型转换
    auto &kfparam = static_cast<SAlgoKfInitParam&>(param);

    // 维度合法性校验：必须非零且不超过模板容量 N
    if (kfparam.x_size == 0 || kfparam.z_size == 0 ||
        kfparam.x_size > N || kfparam.z_size > N || kfparam.u_size > N) {
        return APP_ERROR;
    }
    // 各矩阵元素总数(rows*cols)不得超出容器容量 N
    if (static_cast<uint32_t>(kfparam.x_size) * kfparam.x_size > N ||
        static_cast<uint32_t>(kfparam.z_size) * kfparam.z_size > N ||
        static_cast<uint32_t>(kfparam.x_size) * kfparam.z_size > N ||
        (kfparam.u_size > 0 && static_cast<uint32_t>(kfparam.x_size) * kfparam.u_size > N)) {
        return APP_ERROR;
    }

    DT = kfparam.DT;
    u_size_ = kfparam.u_size;
    x_size_ = kfparam.x_size;
    z_size_ = kfparam.z_size;
    use_auto_adjustment_ = kfparam.use_auto_adjustment;
    if(use_auto_adjustment_){
    measurement_degree_ = kfparam.measurement_degree;
    measurement_map_ = kfparam.measurement_map;
    r_diagonal_elements_ = kfparam.r_diagonal_elements;
    state_min_variance_ = kfparam.state_min_variance;
    }else{
        measurement_valid_num_ = z_size_;
    }
    
    // 初始化卡尔曼黄金五式参与运算的矩阵
    xhat = zeros<float, N>(x_size_, 1);
    xhatMinus = zeros<float, N>(x_size_, 1);
    F = zeros<float, N>(x_size_, x_size_);
    FT = zeros<float, N>(x_size_, x_size_);
    P = eye<float, N>(x_size_);
    Pminus = zeros<float, N>(x_size_, x_size_);
    Q = zeros<float, N>(x_size_, x_size_);
    if (u_size_ > 0) {      // 系统有输入的时候才初始化控制矩阵和输入向量
        B = zeros<float, N>(x_size_, u_size_);
        u = zeros<float, N>(u_size_, 1);
    }
    H = zeros<float, N>(z_size_, x_size_);
    HT = zeros<float, N>(x_size_, z_size_);
    R = zeros<float, N>(z_size_, z_size_);
    K = zeros<float, N>(x_size_, z_size_);
    S = zeros<float, N>(z_size_, z_size_);
    S_inv = zeros<float, N>(z_size_, z_size_);
    z = zeros<float, N>(z_size_, 1);
    measured_vector_ = zeros<float, N>(z_size_,1);
    if (use_auto_adjustment_)
    {
    if (measurement_map_.size() != z_size_)return APP_ERROR;
    if (measurement_degree_.size() != z_size_)return APP_ERROR;
    if (r_diagonal_elements_.size() != z_size_) return APP_ERROR;
    if (!state_min_variance_.empty() &&state_min_variance_.size() != x_size_)return APP_ERROR;
    }
        // 检查维度
    Kf_Info.isitialized = true;
    return APP_OK;
    }

/**
 * @brief 卡尔曼滤波更新
 * @retval EAppStatus
 */
template<uint16_t N>
EAppStatus CAlgo_Kf<N>::UpdateHandler_() {
    // 动态调整(若需要)
    Algo_Kf_Adjustment();

    // 预测
    Algo_Kf_Xhatminus_Update();
    Algo_Kf_Pminus_Update();

    // 有效观测才做测量更新
    if (measurement_valid_num_ > 0) {
        Algo_Kf_K_Update();
        Algo_Kf_Xhat_Update();
        Algo_Kf_P_Update();
    } else {
        xhat = xhatMinus;
        P = Pminus;
    }

    // 4. P限幅
    if (!state_min_variance_.empty()) {
        for (uint8_t i = 0; i < x_size_; ++i) {
            if (P[i][i] < state_min_variance_[i]) {
                P[i][i] = state_min_variance_[i];
            }
        }
    }

    Kf_Info.filtered_value = xhat;
    return APP_OK;
}

/**
 * @brief 更新先验估计
 * 
 */
template<uint16_t N>
void CAlgo_Kf<N>::Algo_Kf_Xhatminus_Update(){
    if (u_size_ > 0) {
    xhatMinus = F * xhat + B * u;
    } 
    else {
        xhatMinus = F * xhat;
    }   // 是否有输入量决定X先验协方差的计算方式
}

/**
 * @brief 更新先验估计协方差
 * 
 */
template<uint16_t N>
void CAlgo_Kf<N>::Algo_Kf_Pminus_Update(){
    Pminus = F * P * FT + Q;
}

/**
 * @brief 更新卡尔曼增益
 * 
 */
template<uint16_t N>
void CAlgo_Kf<N>::Algo_Kf_K_Update(void){
    S = H * Pminus * HT + R;
    S_inv = inv(S);
    K = Pminus * HT * S_inv;
}

/**
 * @brief 更新后验最优估计
 * 
 */
template<uint16_t N>
void CAlgo_Kf<N>::Algo_Kf_Xhat_Update(void){
    xhat = xhatMinus + K * (z - H * xhatMinus);
}

/**
 * @brief 更新后验估计协方差
 * 
 */
template<uint16_t N>
void CAlgo_Kf<N>::Algo_Kf_P_Update(void){
    Matrixt<float, N>I;
    I = eye<float, N>(x_size_);
    Matrixt<float, N> A = I - K * H;
    P = A * Pminus * trans(A) + K * R * trans(K);
}

/**
 * @brief 动态调整
 * 
 * 
 */
template<uint16_t N>
void CAlgo_Kf<N>::Algo_Kf_Adjustment() {
    if (!use_auto_adjustment_) {
        // 不用动态调整，直接用原始尺寸
        z = measured_vector_;
        measurement_valid_num_ = z_size_;
        // H、R由外部设置
        return;
    }

    measurement_valid_num_ = 0;
    etl::vector<uint8_t, 16> valid_indices;
    for (uint8_t i = 0; i < z_size_; ++i) {
        if (!std::isnan(measured_vector_[i][0])) {
            valid_indices.push_back(i);
            measurement_valid_num_++;
        }
    }
    if (measurement_valid_num_ == 0) return;

    // 动态重构 z, H, R
    z = Matrixt<float, N>(measurement_valid_num_, 1);
    H = Matrixt<float, N>(measurement_valid_num_, x_size_);
    R = Matrixt<float, N>(measurement_valid_num_, measurement_valid_num_);

    etl::fill(H.get_data(), H.get_data() + H.size(), 0.0f);
    etl::fill(R.get_data(), R.get_data() + R.size(), 0.0f);

    for (uint8_t i = 0; i < measurement_valid_num_; ++i) {
        uint8_t idx = valid_indices[i];
        z[i][0] = measured_vector_[idx][0];
        if (measurement_map_[idx] == 0) continue;
        uint8_t state_idx = measurement_map_[idx] - 1;
        if (state_idx < x_size_) {
            H[i][state_idx] = measurement_degree_[idx];
        }
        R[i][i] = r_diagonal_elements_[idx];
    }
    // 清空 measured_vector_
    etl::fill(measured_vector_.get_data(), measured_vector_.get_data() + measured_vector_.size(), NAN);
    HT = trans(H);
}

/**
 * @brief 更新观测矩阵
 * 
 */
template<uint16_t N>
void CAlgo_Kf<N>::Set_H(const Matrixt<float, N>& H_in){
    if (H_in.rows() == z_size_ && H_in.cols() == x_size_) {
        H = H_in;
        HT = trans(H);
    }
}

/**
 * @brief 更新观测噪声协方差
 * 
 */
template<uint16_t N>
void CAlgo_Kf<N>::Set_R(const Matrixt<float, N>& R_in){
    if (R_in.rows() == z_size_ && R_in.cols() == z_size_) {
        R = R_in;
    }
}
    
} // namespace robotpilots::algorithm

#endif // ALGO_KF_FILTER_HPP
