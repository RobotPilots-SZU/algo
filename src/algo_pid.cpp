/**
 * @file algo_pid.cpp
 * @author cuteelaina (1105549920@qq.com)
 * @brief PID 控制器实现
 * @version 1.0
 * @date 2026-05-28
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "robotpilots/algorithm/algo_pid.hpp"

#include <algorithm>


namespace robotpilots::algorithm {
/**
 * @brief 初始化PID
 * 
 * @param pStructInitParam 
 * @return EAppStatus 
 */
EAppStatus CAlgoPid::InitPID(const SAlgoInitParam_Pid *pStructInitParam){
    // 检查参数是否为空
    if(pStructInitParam == nullptr) return APP_ERROR;
    // 检查状态
    if(PidStatus == APP_BUSY) return APP_ERROR;

    auto &param = *pStructInitParam;

    // 先检查：tickRate 不能为 0，否则积分计算会除零
    if(param.tickRate == 0) return APP_ERROR;

    // 初始化pid参数
    tickRate_ = param.tickRate;
    kp_ = param.kp;
    ki_ = param.ki;
    kd_ = param.kd;
    inputDeadband_ = param.Input_deadband;
    maxOutput_ = param.maxOutput;
    maxIntegral_ = param.maxIntegral;
    errorMode_ = param.errorMode;
    input_integralSeparation_ = param.input_integralSeparation;
    MachineModeErrorRange_ = param.MachineModeErrorRange;
    sustainable_output_ = param.sustainable_output;
    sustainable_time_ = param.sustainable_time;
    recover_time_ = param.recover_time;

    // 重置一次pid
    ResetPidController();

    PidStatus = APP_OK;

    return APP_OK;
}

/**
 * @brief 根据目标值和测量值更新PID控制器并返回输出值
 * 
 */
float_t CAlgoPid::UpdatePidController(const float_t &target, 
                                      const float_t &measure){
    float_t output = 0.0f;

    // 检查状态
    if(PidStatus == APP_RESET) return output;
    if(PidStatus == APP_BUSY) return output;

    // 计算误差
    auto error = CalcError_(target, measure);

    // 计算输出
    output = CalcOutput_(error, SPidInfo_);

    return output;
}

/**
 * @brief 重置PID控制器
 * 
 */
EAppStatus CAlgoPid::ResetPidController(){
    // 仅在计算中(BUSY)禁止重置；RESET/OK 状态都允许
    if(PidStatus == APP_BUSY) return APP_ERROR;

    // 真正清零内部状态：积分、微分、上次误差、输出、超时/恢复计数
    SPidInfo_ = SPidInfo{};

    return APP_OK;
}

/**
 * @brief 计算误差
 * 
 * @param target 
 * @param measure 
 * @return DataBuffer<float_t> 
 */
float_t CAlgoPid::CalcError_(const float_t &target, 
                             const float_t &measure){
    float_t error = 0.0f;

    // 检查状态
    if(PidStatus == APP_RESET) return error;

    switch (errorMode_)
    {
        case EPidErrorMode::NORMAL:
                error = target - measure;
            break;
        
        case EPidErrorMode::ANGLE:
                error = target - measure;
                // 将误差限制在-180 - 180内,并处理经过零点情况
                if(std::fabs(error) > 180.0f){
                    error -= std::copysign(360.0f, error);
                }
            break;

        case EPidErrorMode::MACHINE:
                error = target - measure;
                // 将误差限制在-4096到4096内,并处理经过零点情况
                if(std::fabs(error) > static_cast<float_t>(MachineModeErrorRange_) / 2){
                    error -= std::copysign(static_cast<float_t>(MachineModeErrorRange_), error);
                }
            break;
        
        default:
            break;
    }

    return error;
}

/**
 * @brief 计算输出
 * 
 * @param error 
 * @param threadInfo 
 * @return float_t 
 */
float_t CAlgoPid::CalcOutput_(const float_t error, SPidInfo &info){

    

    // 检查死区
    if(inputDeadband_ > 0.0f && std::fabs(error) < inputDeadband_){
        info.integral = 0.0f;
        info.lastError = 0.0f;
        return 0.0f;
    }

    // 计算积分分离
    if(input_integralSeparation_ > 0.0f && std::fabs(error) > input_integralSeparation_){
        info.integral = 0.0f;
    }
    else
    {
        // 计算积分
        // 一秒钟增加error的平均值，方便直观估计输出以进行调参
        info.integral += error / static_cast<float_t>(tickRate_);
        info.integral = std::clamp(info.integral, -maxIntegral_, maxIntegral_);
    }
    

    // 计算微分
    // 进行低通滤波
    info.derivative = info.derivative + 0.1f * ((error - info.lastError) - info.derivative);

    // 计算输出
    info.pOut = kp_ * error;
    info.iOut = ki_ * info.integral;
    info.dOut = kd_ * info.derivative;

    // 计算并限制总输出
    info.output = info.pOut + info.iOut + info.dOut;
    info.output = std::clamp(info.output, -maxOutput_, maxOutput_);

    // 可持续输出
    if(sustainable_output_ > 0.0f){
        // 注意：output == 0 时此处 sign 取 -1（零无方向）。当前无实际影响，
        // 因 output==0 时 |output| > sustainable_output_ 恒为 false，不会进入限幅分支。
        int8_t sign = info.output > 0.0f ? 1 : -1;
        // 计算可持续时间
        if(std::fabs(info.output) > sustainable_output_){
            // 注意：Time_exceed 为 uint32_t，若 sustainable_time_ 配为 UINT32_MAX
            // 会溢出回绕导致限幅永不触发；当前未做饱和处理。
            info.Time_exceed++;
            if(info.Time_exceed >= sustainable_time_){
                info.output = sustainable_output_ * sign;
                info.is_limit = true;
            }
        }
        else {
            if (info.Time_exceed > 0) {
                info.Time_exceed--;
            }
        }
        
        if (info.is_limit) {
            // 注意：Time_recover 为 uint32_t，若 recover_time_ 配为 UINT32_MAX
            // 会溢出回绕导致限幅无法解除；当前未做饱和处理。
            info.Time_recover++;
            if (info.Time_recover >= recover_time_) {
                info.Time_exceed = 0;
                info.Time_recover = 0;
                info.is_limit = false;
            }
        }
    }

    // 更新上一次的误差
    info.lastError = error;

    return info.output;
}

} // namespace robotpilots::algorithm
