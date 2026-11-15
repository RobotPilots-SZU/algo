#include "robotpilots/algorithm/matrix.hpp"

// Matrix<float, N> 的成员函数因为 N 是模板参数，无法在本文件里按任意 N 定义，
// 已全部内联在 matrix.hpp（由 ALGO_ENABLE_ARM_MATH 决定是否走 CMSIS-DSP）。
// 本文件保留以兼容构建脚本的源文件列表。
