#ifndef MODULE_CHASSIS_MODEL_H
#define MODULE_CHASSIS_MODEL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "app_config.h"
#include "LQR.h"
#include "PID.h"

#include <stdint.h>

#define MODULE_CHASSIS_MODEL_PI 3.14159265358979323846f
#define MODULE_CHASSIS_MODEL_HALF_PI 1.57079632679489661923f
#define MODULE_CHASSIS_CONTROL_STATE_COUNT 10U
#define MODULE_CHASSIS_CONTROL_OUTPUT_COUNT 4U

typedef enum
{
    MODULE_CHASSIS_LEG_LEFT = 0,
    MODULE_CHASSIS_LEG_RIGHT,
    MODULE_CHASSIS_LEG_COUNT,
} module_chassis_leg_side_t;

typedef enum
{
    MODULE_CHASSIS_LEG_JOINT_FRONT = 0,
    MODULE_CHASSIS_LEG_JOINT_BACK,
    MODULE_CHASSIS_LEG_JOINT_COUNT,
} module_chassis_leg_joint_t;

typedef enum
{
    MODULE_CHASSIS_STATE_FORWARD_POSITION = 0,  /* s：自然坐标系下机器人水平方向位移，单位 m。 */
    MODULE_CHASSIS_STATE_FORWARD_VELOCITY,      /* dot_s：s 的导数，单位 m/s。 */
    MODULE_CHASSIS_STATE_YAW,                   /* fai：偏航角 yaw，单位 rad。 */
    MODULE_CHASSIS_STATE_YAW_RATE,              /* dot_fai：偏航角速度，单位 rad/s。 */
    MODULE_CHASSIS_STATE_LEFT_LEG_ANGLE,        /* theta_l：左虚拟腿倾斜角，单位 rad。 */
    MODULE_CHASSIS_STATE_LEFT_LEG_ANGLE_RATE,   /* dot_theta_l：左虚拟腿倾斜角速度，单位 rad/s。 */
    MODULE_CHASSIS_STATE_RIGHT_LEG_ANGLE,       /* theta_r：右虚拟腿倾斜角，单位 rad。 */
    MODULE_CHASSIS_STATE_RIGHT_LEG_ANGLE_RATE,  /* dot_theta_r：右虚拟腿倾斜角速度，单位 rad/s。 */
    MODULE_CHASSIS_STATE_BODY_PITCH,            /* theta_b：机体俯仰角，单位 rad。 */
    MODULE_CHASSIS_STATE_BODY_PITCH_RATE,       /* dot_theta_b：机体俯仰角速度，单位 rad/s。 */
} module_chassis_control_state_index_t;

typedef enum
{
    MODULE_CHASSIS_CONTROL_LEFT_WHEEL_TORQUE = 0,
    MODULE_CHASSIS_CONTROL_RIGHT_WHEEL_TORQUE,
    MODULE_CHASSIS_CONTROL_LEFT_LEG_TORQUE,     /* 左虚拟腿摆动力矩；正值使左腿角 theta_l 增大。 */
    MODULE_CHASSIS_CONTROL_RIGHT_LEG_TORQUE,    /* 右虚拟腿摆动力矩；正值使右腿角 theta_r 增大。 */
} module_chassis_control_output_index_t;

typedef struct
{
    float link1LengthM;          /* 五连杆主动杆 1 长度，单位 m。 */
    float link2LengthM;          /* 五连杆从动杆 2 长度，单位 m。 */
    float link3LengthM;          /* 五连杆从动杆 3 长度，单位 m。 */
    float link4LengthM;          /* 五连杆主动杆 4 长度，单位 m。 */
    float frameJointDistanceM;   /* 左右固定铰点间距，单位 m。 */
    float minLegLengthM;         /* 几何计算允许的最小虚拟腿长，单位 m。 */
} module_chassis_leg_geometry_config_t;

typedef struct
{
    uint8_t motorIndex;          /* 对应 module_chassis_input_t.dmMotors 的下标。 */
    float angleOffsetRad;        /* 电机反馈角到几何角的零位偏置，单位 rad。 */
    float angleScale;            /* 电机反馈角到几何角的方向系数。 */
    float torqueScale;           /* 几何关节力矩到电机命令力矩的方向系数。 */
} module_chassis_joint_map_t;

typedef struct
{
    module_chassis_leg_geometry_config_t geometry;
    module_chassis_joint_map_t joints[MODULE_CHASSIS_LEG_JOINT_COUNT];
    float targetLegLengthM;      /* 当前固定腿长目标，单位 m。 */
} module_chassis_leg_config_t;

typedef struct
{
    uint8_t bodyPitchRateGyroIndex;  /* 机体俯仰角速度使用的 gyroRadps 下标。 */
    uint8_t rollRateGyroIndex;       /* 横滚角速度使用的 gyroRadps 下标。 */
    uint8_t yawRateGyroIndex;        /* 偏航角速度使用的 gyroRadps 下标。 */
    float bodyPitchAngleScale;
    float bodyPitchRateScale;
    float rollAngleScale;
    float rollRateScale;
    float yawAngleScale;
    float yawRateScale;
    uint8_t forwardAccelerationIndex; /* dot_s 卡尔曼融合使用的 motionAccMps2 下标。 */
    float forwardAccelerationScale;   /* 自然系运动加速度到 dot_s 正方向的符号系数。 */
} module_chassis_imu_map_config_t;

typedef struct
{
    float radiusM;                /* 轮半径，单位 m。 */
    float halfTrackM;             /* 轮距一半，单位 m，用于后续转向模型。 */
    float leftVelocityScale;      /* 左轮 ESC rpm 到轮角速度后的方向系数。 */
    float rightVelocityScale;     /* 右轮 ESC rpm 到轮角速度后的方向系数。 */
    float torqueLimitNm;          /* 轮端力矩限幅，单位 N*m；小于等于 0 表示不允许输出。 */
    float torqueToCurrentRaw;     /* 轮端力矩到 DJI 原始电流命令的换算系数。 */
    int16_t currentLimitRaw;      /* DJI 原始电流命令限幅。 */
} module_chassis_wheel_config_t;

typedef struct
{
    uint8_t enabled;               /* 前进速度卡尔曼融合开关；关闭时直接使用轮腿几何速度。 */
    float initialCovariance[4];    /* 速度卡尔曼初始 P，状态为 [dot_s, ddot_s]。 */
    float processNoise[4];         /* 速度卡尔曼 Q。 */
    float measurementNoise[4];     /* 速度卡尔曼 R，测量为 [原始前进速度, IMU 前向加速度]。 */
    float positionIntegralVelocityLimitMps; /* 速度积分为 s 时允许的最大速度绝对值。 */
} module_chassis_velocity_kalman_config_t;

typedef struct
{
    uint8_t jointTorqueOutputEnabled;    /* 非零 DM 力矩输出调试开关。 */
    uint8_t wheelCurrentOutputEnabled;   /* 非零 DJI 电流输出调试开关。 */
    float jointTorqueLimitNm;            /* DM 髋关节力矩限幅，单位 N*m。 */
} module_chassis_output_config_t;

typedef enum
{
    MODULE_CHASSIS_LQR_K_LENGTH_FIXED = 0,         /* 使用固定腿长计算 lqrK，适合早期定腿长方向调试。 */
    MODULE_CHASSIS_LQR_K_LENGTH_MEASURED,          /* 使用实时几何腿长计算 lqrK，适合变腿长控制。 */
} module_chassis_lqr_k_length_source_t;

typedef struct
{
    uint8_t enabled;                               /* LQR 拟合开关；关闭时使用 fixedLqrK 固定 K 矩阵。 */
    module_chassis_lqr_k_length_source_t lqrKLengthSource; /* lqrK 拟合使用的腿长来源。 */
    float minFitLegLengthM;                        /* K 拟合有效最小腿长，单位 m。 */
    float maxFitLegLengthM;                        /* K 拟合有效最大腿长，单位 m。 */
    float fixedLeftLegLengthM;                     /* 定腿长调试时使用的左腿长，单位 m。 */
    float fixedRightLegLengthM;                    /* 定腿长调试时使用的右腿长，单位 m。 */
    float lqrKFitCoefficients[MODULE_CHASSIS_CONTROL_OUTPUT_COUNT][MODULE_CHASSIS_CONTROL_STATE_COUNT][ALGORITHM_LQR_POLY22_COEFFICIENT_COUNT]; /* lqrK(row,col) 的 poly22 拟合系数。 */
} module_chassis_lqr_config_t;

typedef struct
{
    module_chassis_leg_config_t legs[MODULE_CHASSIS_LEG_COUNT];
    module_chassis_imu_map_config_t imu;
    module_chassis_wheel_config_t wheel;
    module_chassis_velocity_kalman_config_t velocityKalman;
    algorithm_pid_config_t legLengthPid;
    algorithm_pid_config_t rollPid;
    module_chassis_lqr_config_t lqr;
    module_chassis_output_config_t output;
    float legVerticalAngleOffsetRad;     /* 虚拟腿竖直参考角，单位 rad。 */
    float targetRollRad;                 /* 目标横滚角，单位 rad。 */
    float baseSupportForceN;             /* 基础虚拟支撑力，单位 N。 */
    float leftSupportForceFeedforwardN;  /* 左腿支撑力前馈，单位 N。 */
    float rightSupportForceFeedforwardN; /* 右腿支撑力前馈，单位 N。 */
    float defaultDtSec;                         /* 默认控制周期，单位 s。 */
    float minDtSec;                             /* 控制周期下限，单位 s；异常过小时使用默认周期。 */
    float maxDtSec;                             /* 控制周期上限，单位 s；异常过大时使用默认周期。 */
    float targetState[MODULE_CHASSIS_CONTROL_STATE_COUNT]; /* LQR/MPC 风格控制目标状态，顺序见 module_chassis_control_state_index_t。 */
    float fixedLqrK[MODULE_CHASSIS_CONTROL_OUTPUT_COUNT][MODULE_CHASSIS_CONTROL_STATE_COUNT]; /* 离线计算得到的固定 K 矩阵。 */
} module_chassis_model_config_t;

/**
 * @brief 获取当前轮腿机器人模型的默认控制参数。
 *
 * 机械结构只按当前五连杆轮腿实现，后续改尺寸时应集中修改该配置。
 */
const module_chassis_model_config_t *Module_Chassis_Model_GetDefaultConfig(void);

#ifdef __cplusplus
}
#endif

#endif
