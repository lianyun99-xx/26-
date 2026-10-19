//#include "kinematics.h"
//
//#include <math.h>
//
//__attribute__((always_inline))
//static inline float fast_acosf(float x) {
//    float neg = (x < 0.0f);
//    x = fabsf(x);
//    float r = -0.0187293f;
//    r = r * x + 0.0742610f;
//    r = r * x - 0.2121144f;
//    r = r * x + 1.5707288f;
//    r = r * sqrtf(1.0f - x);
//    r = r - 2.0f * neg * r;
//    return neg * 3.14159265358979323846f + r;
//}
//// ==========================================================
//// 【趴地校准初始角度设置】 (请根据你的机器狗实际硬件测量并修改！)
//// 假设垂直向下为 0 弧度，逆时针/向前为正。
//// ==========================================================
//// 例如：如果大腿上电时向前倾斜 45度，就是 45 * PI / 180 = 0.785f 弧度
//#define CALIB_THETA1 -0.47f
//// 例如：如果小腿上电时向后折叠 90度，就是 -90 * PI / 180 = -1.570f 弧度
//#define CALIB_THETA2 2.06f
////#define CALIB_THETA1 -0.35f
////#define CALIB_THETA2 1.8f
//// ==========================================================
//// 全局零位记忆中枢（正逆解共享）
//// ==========================================================
//float global_pos_offset[10] = {0.0f};
//int global_has_offset[10] = {0};
//int last_is_calibrated = 0;
//extern int is_calibrated;
//
//// ==========================================================
//// 正运动学 (Forward Kinematics)
//// ==========================================================
//Currentpos ForwardKinematics(MotorController *ctrl, int hip_id, int knee_id) {
//    MotorData_t *motor_s = &ctrl->datas[hip_id];
//    MotorData_t *motor_r = &ctrl->datas[knee_id];
//    Currentpos currentpos = {0.0f, 0.0f};
//
//    float L1 = 0.185f;
//    float L2 = 0.215f;
//    float theta1 = 0.0f;
//    float theta2 = 0.0f;
//
//    // 校准防呆逻辑（上升沿检测）
//    if (is_calibrated && !last_is_calibrated) {
//        for (int i = 0; i < 10; i++) {
//            global_has_offset[i] = 0;
//        }
//    }
//    last_is_calibrated = is_calibrated;
//
//    // 抓取并锁定初始零位
//    if (is_calibrated) {
//        if (fabsf(motor_s->Pos) > 0.001f || fabsf(motor_r->Pos) > 0.001f) {
//            if (global_has_offset[hip_id] == 0) {
//                global_pos_offset[hip_id] = motor_s->Pos;
//                global_has_offset[hip_id] = 1;
//            }
//            if (global_has_offset[knee_id] == 0) {
//                global_pos_offset[knee_id] = motor_r->Pos;
//                global_has_offset[knee_id] = 1;
//            }
//        } else {
//            return currentpos;
//        }
//    }
//
//    // 没抓到零位偏置前，不解算，返回0
//    if (!global_has_offset[hip_id] || !global_has_offset[knee_id]) {
//        return currentpos;
//    }
//
//    float offset_s = global_pos_offset[hip_id];
//    float offset_r = global_pos_offset[knee_id];
//
//    float delta_t1 = 0.0f;
//    float delta_t2 = 0.0f;
//
//    // --- 1. 计算电机相对于“初始趴地姿态”转动的角度变化量 ---
//    if (hip_id == 1 || hip_id == 6) {
//        delta_t1 = (motor_s->Pos - offset_s) / 6.33f;
//        delta_t2 = (offset_r - motor_r->Pos) / 6.33f;
//    }
//    else if (hip_id == 7 || hip_id == 3) {
//        delta_t1 = (offset_s - motor_s->Pos) / 6.33f;
//        delta_t2 = (motor_r->Pos - offset_r) / 6.33f;
//    }
//
//    // --- 2. 真实绝对角度 = 角度变化量 + 初始趴地的固定物理角度 ---
//    theta1 = delta_t1 + CALIB_THETA1;
//    theta2 = delta_t2 + CALIB_THETA2;
//
//    // --- 3. 标准正运动学解算XY坐标 ---
//    currentpos.X = L1 * sinf(theta1) + L2 * sinf(theta2 + theta1 + 0.215f * (float)M_PI);
//    currentpos.Y = L1 * cosf(theta1) + L2 * cosf(theta2 + theta1 + 0.215f * (float)M_PI);
//
//    return currentpos;
//}
//
//// ==========================================================
//// 逆运动学 (Inverse Kinematics)
//// ==========================================================
//LegAngles InverseKinematics(float X, float Y, MotorController* ctrl, int hip_id, int knee_id, float kp, float kd) {
//    LegAngles angles = {0.0f, 0.0f};
//
//    // 安全防护：没有零位记忆就不乱发指令
//    if (!global_has_offset[hip_id] || !global_has_offset[knee_id]) {
//        return angles;
//    }
//
//    float L1 = 0.185f;
//    float L2 = 0.215f;
//    float L3 = sqrtf(X*X + Y*Y);
//
//    // 结构限制防护 (防 NaN)
//    if (L3 > L1 + L2) L3 = L1 + L2 - 0.001f;
//    if (L3 < fabsf(L1 - L2)) L3 = fabsf(L1 - L2) + 0.001f;
//
//    // 余弦定理计算
//    float cos_val1 = (L3*L3 - L1*L1 - L2*L2) / (2.0f * L1 * L2);
//    if (cos_val1 > 1.0f) cos_val1 = 1.0f;
//    if (cos_val1 < -1.0f) cos_val1 = -1.0f;
//    float theta2_ext = acosf(cos_val1);
//
//    float cos_val2 = (L3*L3 + L1*L1 - L2*L2) / (2.0f * L1 * L3);
//    if (cos_val2 > 1.0f) cos_val2 = 1.0f;
//    if (cos_val2 < -1.0f) cos_val2 = -1.0f;
//    float theta4 = acosf(cos_val2);
//
//    // 【坐标系归一】：计算出目标坐标对应的绝对理想物理角度
//    float t1_target = atan2f(X, Y) - theta4;
//    float t2_target = theta2_ext - 0.215f * (float)M_PI;
//
//    float offset_s = global_pos_offset[hip_id];
//    float offset_r = global_pos_offset[knee_id];
//
//    float pos_s = 0.0f;
//    float pos_r = 0.0f;
//
////     --- 1. 计算要让电机跑的相对量 (目标绝对角度 - 初始趴地物理角度) ---
//    float delta_t1_target = t1_target - CALIB_THETA1;
//    float delta_t2_target = t2_target - CALIB_THETA2;
//
//    // --- 2. 将角度差值严格按正解公式反推回电机位置 ---
//    if (hip_id == 1 || hip_id == 6) {
//        pos_s = delta_t1_target * 6.33f + offset_s;
//        pos_r = offset_r - delta_t2_target * 6.33f;
//    }
//    else if (hip_id == 7 || hip_id == 3) {
//        pos_s = offset_s - delta_t1_target * 6.33f;
//        pos_r = delta_t2_target * 6.33f + offset_r;
//    }
//
//    angles.theta1 = pos_s;
//    angles.theta2 = pos_r;
//
//    return angles;
//}
//
//LegState_t Calculate_Leg_State(MotorController *ctrl, int hip_id, int knee_id) {
//    LegState_t state = {0};
//    MotorData_t *motor_s = &ctrl->datas[hip_id];
//    MotorData_t *motor_r = &ctrl->datas[knee_id];
//
//    // 1. 定义连杆长度 (单位：米)
//    float L1 = 0.185f;
//    float L2 = 0.215f;
//
//    // 防呆保护：如果没有校准零点，直接返回 0
//    if (!global_has_offset[hip_id] || !global_has_offset[knee_id]) return state;
//
//    float offset_s = global_pos_offset[hip_id];
//    float offset_r = global_pos_offset[knee_id];
//
//    float theta1 = 0.0f, theta2 = 0.0f;
//    float omega1 = 0.0f, omega2 = 0.0f;
//
//    // 2. 解算真实物理角度和角速度 (单位: rad 和 rad/s)，与 ForwardKinematics 保持一致
//    if (hip_id == 1 || hip_id == 6) {
//        theta1 = (motor_s->Pos - offset_s) / 6.33f + CALIB_THETA1;
//        theta2 = (offset_r - motor_r->Pos) / 6.33f + CALIB_THETA2;
//        omega1 =  motor_s->true_W;
//        omega2 = -motor_r->true_W;
//    } else if (hip_id == 7 || hip_id == 3) {
//        theta1 = (offset_s - motor_s->Pos) / 6.33f + CALIB_THETA1;
//        theta2 = (motor_r->Pos - offset_r) / 6.33f + CALIB_THETA2;
//        omega1 = -motor_s->true_W;
//        omega2 =  motor_r->true_W;
//    }
//
//    // 3. 正运动学计算末端坐标 X, Y (单位：m)
//    float alpha = theta2 + theta1 + 0.215f * (float)M_PI;
//    state.pos_vel.X = L1 * sinf(theta1) + L2 * sinf(alpha);
//    state.pos_vel.Y = L1 * cosf(theta1) + L2 * cosf(alpha);
//
//    // 4. 计算雅可比矩阵
//    state.jacobian.J11 = state.pos_vel.Y;
//    state.jacobian.J12 = L2 * cosf(alpha);
//    state.jacobian.J21 = -state.pos_vel.X;
//    state.jacobian.J22 = -L2 * sinf(alpha);
//
//    // 5. 核心：雅可比乘角速度，得到末端真实线速度 (Vx, Vy)
//    state.pos_vel.Vx = state.jacobian.J11 * omega1 + state.jacobian.J12 * omega2;
//    state.pos_vel.Vy = state.jacobian.J21 * omega1 + state.jacobian.J22 * omega2;
//
//    return state;
//}
//
//
//
//
//
//
////悬空状态启动
////#include "kinematics.h"
////#include <math.h>
////__attribute__((always_inline))
////static inline float fast_acosf(float x) {
////    float neg = (x < 0.0f);
////    x = fabsf(x);
////    float r = -0.0187293f;
////    r = r * x + 0.0742610f;
////    r = r * x - 0.2121144f;
////    r = r * x + 1.5707288f;
////    r = r * sqrtf(1.0f - x);
////    r = r - 2.0f * neg * r;
////    return neg * 3.14159265358979323846f + r;
////}
//////#define PI 3.141f
////float global_pos_offset[10] = {0.0f};
////int global_has_offset[10] = {0};
////int last_is_calibrated = 0;
////extern int is_calibrated;
////Currentpos ForwardKinematics(MotorController *ctrl, int hip_id, int knee_id) {
////    MotorData_t *motor_s = &ctrl->datas[hip_id];
////    MotorData_t *motor_r = &ctrl->datas[knee_id];
////    Currentpos currentpos = {0.0f, 0.0f};
////
////    float L1 = 0.1850f;
////    float L2 = 0.2150f;
////    float theta1 = 0.0f;
////    float theta2 = 0.0f;
////
////    // 校准防呆逻辑（上升沿检测）
////    if (is_calibrated && !last_is_calibrated) {
////        for (int i = 0; i < 10; i++) {
////            global_has_offset[i] = 0;
////        }
////    }
////    last_is_calibrated = is_calibrated;
////
////    if (is_calibrated) {
////        if (fabsf(motor_s->Pos) > 0.001f || fabsf(motor_r->Pos) > 0.001f) {
////            if (global_has_offset[hip_id] == 0) {
////                global_pos_offset[hip_id] = motor_s->Pos;
////                global_has_offset[hip_id] = 1;
////            }
////            if (global_has_offset[knee_id] == 0) {
////                global_pos_offset[knee_id] = motor_r->Pos;
////                global_has_offset[knee_id] = 1;
////            }
////        } else {
////            return currentpos;
////        }
////    }
////
////    if (!global_has_offset[hip_id] || !global_has_offset[knee_id]) {
////        return currentpos;
////    }
////
////    float offset_s = global_pos_offset[hip_id];
////    float offset_r = global_pos_offset[knee_id];
////
////    // --- 已验证的正确正解公式 ---
////    if (hip_id == 1) {
////        theta1 = (motor_s->Pos - offset_s) / 6.33f ;
////        theta2 = (offset_r - motor_r->Pos) / 6.33f ;
////    }
////    else if (hip_id == 6) {
////        theta1 = (motor_s->Pos - offset_s) / 6.33f;
////        theta2 = (offset_r - motor_r->Pos) / 6.33f;
////    }
////    else if (hip_id == 7) {
////        theta1 = (offset_s - motor_s->Pos) / 6.33f;
////        theta2 = (motor_r->Pos - offset_r) / 6.33f;
////    }
////    else if (hip_id ==3) {
////        theta1 = (offset_s - motor_s->Pos) / 6.33f;
////        theta2 = (motor_r->Pos - offset_r) / 6.33f;
////    }
////
////    currentpos.X = L1 * sinf(theta1) + L2 * sinf(theta2 + theta1 + 0.215f * (float)M_PI);
////    currentpos.Y = L1 * cosf(theta1) + L2 * cosf(theta2 + theta1 + 0.215f * (float)M_PI);
////
////    return currentpos;
////}
////LegAngles InverseKinematics(float X, float Y, MotorController* ctrl, int hip_id, int knee_id, float kp, float kd) {
////    LegAngles angles = {0.0f, 0.0f};
////
////    // 1. 安全防护：没有零位记忆就不乱发指令
////    if (!global_has_offset[hip_id] || !global_has_offset[knee_id]) {
////        return angles;
////    }
////
////    float L1 = 0.185f;
////    float L2 = 0.215f;
////    float L3 = sqrtf(X*X + Y*Y);
////
////    // 2. 结构限制防护 (防 NaN)
////    if (L3 > L1 + L2) L3 = L1 + L2 - 0.001f;
////    if (L3 < fabsf(L1 - L2)) L3 = fabsf(L1 - L2) + 0.001f;
////
////    // 3. 余弦定理计算
////    float cos_val1 = (L3*L3 - L1*L1 - L2*L2) / (2.0f * L1 * L2);
////    if (cos_val1 > 1.0f) cos_val1 = 1.0f;
////    if (cos_val1 < -1.0f) cos_val1 = -1.0f;
////    float theta2_ext = acosf(cos_val1);
////
////    float cos_val2 = (L3*L3 + L1*L1 - L2*L2) / (2.0f * L1 * L3);
////    if (cos_val2 > 1.0f) cos_val2 = 1.0f;
////    if (cos_val2 < -1.0f) cos_val2 = -1.0f;
////    float theta4 = acosf(cos_val2);
////
////    // 4. 【坐标系归一】：无视象限，直接求出目标绝对角度
////    float t1_target = atan2f(X, Y) - theta4;
////    float t2_target = theta2_ext - 0.215f * (float)M_PI;
////
////    // 获取当前腿共享的零位校准值
////    float offset_s = global_pos_offset[hip_id];
////    float offset_r = global_pos_offset[knee_id];
////
////    float pos_s = 0.0f;
////    float pos_r = 0.0f;
////
////    // 5. 【完全对称逆向映射】：根据你最新的纯净正解反推
////    if (hip_id == 1 || hip_id == 6) {
////        pos_s = t1_target * 6.33f + offset_s;
////        pos_r = offset_r - t2_target * 6.33f;
////    }
////    else if (hip_id == 7 || hip_id == 3) {
////        pos_s = offset_s - t1_target * 6.33f;
////        pos_r = t2_target * 6.33f + offset_r;
////    }
////
////    angles.theta1 = pos_s;
////    angles.theta2 = pos_r;
////
////
////    return angles;
////}
////LegState_t Calculate_Leg_State(MotorController *ctrl, int hip_id, int knee_id) {
////    LegState_t state = {0};
////    MotorData_t *motor_s = &ctrl->datas[hip_id];
////    MotorData_t *motor_r = &ctrl->datas[knee_id];
////
////    // 1. 定义连杆长度 (单位：米)
////    float L1 = 0.185f;
////    float L2 = 0.215f;
////
////    // 防呆保护：如果没有校准零点，直接返回 0
////    if (!global_has_offset[hip_id] || !global_has_offset[knee_id]) return state;
////
////    float offset_s = global_pos_offset[hip_id];
////    float offset_r = global_pos_offset[knee_id];
////
////    float theta1 = 0.0f, theta2 = 0.0f;
////    float omega1 = 0.0f, omega2 = 0.0f;
////    if (hip_id == 1 || hip_id == 6) {
////        theta1 = (motor_s->Pos - offset_s) / 6.33f;
////        theta2 = (offset_r - motor_r->Pos) / 6.33f;
////        omega1 =  motor_s->true_W;   // 获取真实角速度
////        omega2 = -motor_r->true_W;   // 注意安装镜像导致的符号反转
////    } else if (hip_id == 7 || hip_id == 3) {
////        theta1 = (offset_s - motor_s->Pos) / 6.33f;
////        theta2 = (motor_r->Pos - offset_r) / 6.33f;
////        omega1 = -motor_s->true_W;
////        omega2 =  motor_r->true_W;
////    }
////
////    float alpha = theta2 + theta1 + 0.215f * (float)M_PI;
////    state.pos_vel.X = L1 * sinf(theta1) + L2 * sinf(alpha);
////    state.pos_vel.Y = L1 * cosf(theta1) + L2 * cosf(alpha);
////
////    // 4. 计算雅可比矩阵
////    state.jacobian.J11 = state.pos_vel.Y;
////    state.jacobian.J12 = L2 * cosf(alpha);
////    state.jacobian.J21 = -state.pos_vel.X;
////    state.jacobian.J22 = -L2 * sinf(alpha);
////
////    // 5. 核心：雅可比乘角速度，得到末端真实线速度！(Vx, Vy)
////    state.pos_vel.Vx = state.jacobian.J11 * omega1 + state.jacobian.J12 * omega2;
////    state.pos_vel.Vy = state.jacobian.J21 * omega1 + state.jacobian.J22 * omega2;
////
////    return state;
////}
