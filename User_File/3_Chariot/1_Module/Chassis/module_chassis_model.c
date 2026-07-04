#include "module_chassis_model.h"

/*
 * 本文件集中保存当前轮腿底盘的软件坐标、硬件映射和极性配置。
 *
 * 一、IMU 软件坐标约定
 * - input->imu.rollRad / pitchRad / yawRad 已由 IMU 任务解算为机体系姿态角。
 * - 当前按 IMU 任务输出约定：gyroRadps[0] 为绕机体 X 轴横滚角速度，
 *   gyroRadps[1] 为绕机体 Y 轴俯仰角速度，gyroRadps[2] 为绕机体 Z 轴偏航角速度。
 * - gyroRadps[0] 当前配置为 roll rate，gyroRadps[1] 当前配置为 pitch rate，
 *   gyroRadps[2] 当前配置为 yaw rate。
 * - bodyPitchAngleScale、rollAngleScale、yawAngleScale 用于修正 IMU 安装方向。
 *   当前均为 +1，表示控制器直接使用 IMU 任务给出的角度正方向。
 * - motionAccMps2 为 IMU 任务去重力并转到自然坐标系后的运动加速度。
 *   forwardAccelerationIndex / forwardAccelerationScale 定义其中哪一轴作为 dot_s
 *   卡尔曼融合的前向加速度输入。
 * - 如果实机安装方向与控制状态定义相反，只允许在本文件对应 scale 中改符号，
 *   不在控制器、任务层或 LQR 系数表里临时反号。
 *
 * 二、轮电机软件极性
 * - 左轮反馈来自 DJI 数组下标 0，对应 CAN 反馈 ID 0x201。
 * - 右轮反馈来自 DJI 数组下标 1，对应 CAN 反馈 ID 0x202。
 * - leftVelocityScale / rightVelocityScale 把 ESC rpm 转成控制器使用的轮端角速度正方向。
 *   当前 scale 均为 +1，表示 ESC rpm 为正时轮端角速度为正，并使 dot_s 向正方向增加。
 * - torqueToCurrentRaw 把控制器输出的轮端力矩转换为 DJI 原始电流命令。
 *   后续标定时，正轮端力矩应定义为驱动 dot_s 向正方向加速；当前为 0，表示轮电机非零输出未开放。
 *
 * 三、髋关节 DM 电机软件极性
 * - motorIndex 使用 device_motor_dm.h 中的数组下标：
 *   0 左前髋、1 左后髋、2 右前髋、3 右后髋。
 * - 当前 CAN ID 映射在 app_config.h：
 *   左前髋 0x002，左后髋 0x001，右前髋 0x004，右后髋 0x003。
 * - 五连杆几何角 phi1/phi4 使用腿部平面坐标：x 轴为几何前向参考，
 *   y 轴为几何上方参考，角度正方向为 x 轴转向 y 轴的数学正方向。
 * - angleOffsetRad 和 angleScale 把 DM 反馈角转换为五连杆几何角。
 *   angleScale = -1 表示 DM 反馈角增加时，几何角按相反方向增加。
 * - torqueScale 把 VMC 求得的几何关节力矩转换为 DM MIT 力矩通道命令。
 *   torqueScale = -1 表示几何关节正力矩映射为 DM 负力矩命令。
 * - angleScale 和 torqueScale 是实机低力矩调试时最先要确认的极性参数。
 */

static const module_chassis_model_config_t chassisDefaultModelConfig = {
    .legs = {
        [MODULE_CHASSIS_LEG_LEFT] = {
            .geometry = {
                .link1LengthM = 0.215f,
                .link2LengthM = 0.258f,
                .link3LengthM = 0.258f,
                .link4LengthM = 0.215f,
                .frameJointDistanceM = 0.0f,
                .minLegLengthM = 0.05f,
            },
            .joints = {
                [MODULE_CHASSIS_LEG_JOINT_FRONT] = {
                    /* 左前髋：motorIndex 0，对应 MOTOR_DM_LEFT_FRONT，CAN ID 0x002。 */
                    .motorIndex = 0U,
                    /* DM 反馈角先加 pi，再取反得到五连杆前主动杆 phi1。 */
                    .angleOffsetRad = MODULE_CHASSIS_MODEL_PI,
                    .angleScale = -1.0f,
                    /* VMC 前髋几何力矩取反后写入 DM MIT torque。 */
                    .torqueScale = -1.0f,
                },
                [MODULE_CHASSIS_LEG_JOINT_BACK] = {
                    /* 左后髋：motorIndex 1，对应 MOTOR_DM_LEFT_BACK，CAN ID 0x001。 */
                    .motorIndex = 1U,
                    /* DM 反馈角直接取反得到五连杆后主动杆 phi4。 */
                    .angleOffsetRad = 0.0f,
                    .angleScale = -1.0f,
                    /* VMC 后髋几何力矩取反后写入 DM MIT torque。 */
                    .torqueScale = -1.0f,
                },
            },
            .targetLegLengthM = 0.25f,
        },
        [MODULE_CHASSIS_LEG_RIGHT] = {
            .geometry = {
                .link1LengthM = 0.215f,
                .link2LengthM = 0.258f,
                .link3LengthM = 0.258f,
                .link4LengthM = 0.215f,
                .frameJointDistanceM = 0.0f,
                .minLegLengthM = 0.05f,
            },
            .joints = {
                [MODULE_CHASSIS_LEG_JOINT_FRONT] = {
                    /* 右前髋：motorIndex 2，对应 MOTOR_DM_RIGHT_FRONT，CAN ID 0x004。 */
                    .motorIndex = 2U,
                    /* 右腿沿用同一套五连杆几何角定义，左右安装差异通过 scale 处理。 */
                    .angleOffsetRad = MODULE_CHASSIS_MODEL_PI,
                    .angleScale = -1.0f,
                    /* 当前右前髋几何力矩同样取反后写入 DM MIT torque。 */
                    .torqueScale = -1.0f,
                },
                [MODULE_CHASSIS_LEG_JOINT_BACK] = {
                    /* 右后髋：motorIndex 3，对应 MOTOR_DM_RIGHT_BACK，CAN ID 0x003。 */
                    .motorIndex = 3U,
                    /* DM 反馈角直接取反得到五连杆后主动杆 phi4。 */
                    .angleOffsetRad = 0.0f,
                    .angleScale = -1.0f,
                    /* 当前右后髋几何力矩同样取反后写入 DM MIT torque。 */
                    .torqueScale = -1.0f,
                },
            },
            .targetLegLengthM = 0.25f,
        },
    },
    .imu = {
        /*
         * 当前 IMU 轴向映射：
         * gyro[0] -> roll rate，gyro[1] -> body pitch rate，gyro[2] -> yaw rate。
         * 所有 angle/rate scale 均为 +1，表示暂不反转 IMU 任务输出的姿态正方向。
         */
        .bodyPitchRateGyroIndex = 1U,
        .rollRateGyroIndex = 0U,
        .yawRateGyroIndex = 2U,
        .bodyPitchAngleScale = 1.0f,
        .bodyPitchRateScale = 1.0f,
        .rollAngleScale = 1.0f,
        .rollRateScale = 1.0f,
        .yawAngleScale = 1.0f,
        .yawRateScale = 1.0f,
        .forwardAccelerationIndex = 0U,
        .forwardAccelerationScale = 1.0f,
    },
    .wheel = {
        .radiusM = 0.10f,
        .halfTrackM = 0.1965f,
        /*
         * DJI 左右轮反馈数组下标固定为 0/1。
         * scale 为 +1 表示 ESC rpm 正方向直接作为控制器轮端角速度正方向。
         */
        .leftVelocityScale = 1.0f,
        .rightVelocityScale = 1.0f,
        /*
         * 轮端力矩到 DJI 电流的换算尚未实机标定。
         * torqueLimitNm/currentLimitRaw 为 0 时，轮电机保持零电流输出。
         */
        .torqueLimitNm = 0.0f,
        .torqueToCurrentRaw = 0.0f,
        .currentLimitRaw = 0,
    },
    .velocityKalman = {
        .enabled = 1U,
        /*
         * 参考 SPR 的速度融合结构：
         * x = [dot_s, ddot_s]，z = [轮腿几何原始速度, IMU 前向运动加速度]。
         * F 中的 dt 由控制器按本轮实际周期写入，因此这里不保存固定 F。
         */
        .initialCovariance = {
            1.0f, 0.0f,
            0.0f, 1.0f,
        },
        .processNoise = {
            0.1f, 0.0f,
            0.0f, 0.1f,
        },
        .measurementNoise = {
            100.0f, 0.0f,
            0.0f, 1.0e12f,
        },
        .positionIntegralVelocityLimitMps = 0.1f,
    },
    .legLengthPid = {
        .kp = 400.0f,
        .ki = 2.0f,
        .kd = 8000.0f,
        .integralLimit = 50.0f,
        .outputLimit = 300.0f,
    },
    .rollPid = {
        .kp = 3000.0f,
        .ki = 1.0f,
        .kd = 100.0f,
        .integralLimit = 30.0f,
        .outputLimit = 300.0f,
    },
    .lqr = {
        .enabled = 1U,
        .lqrKLengthSource = MODULE_CHASSIS_LQR_K_LENGTH_FIXED,
        .minFitLegLengthM = 0.15f,
        .maxFitLegLengthM = 0.35f,
        .fixedLeftLegLengthM = 0.25f,
        .fixedRightLegLengthM = 0.25f,
        /*
         * K(row,col) 使用 MATLAB poly22 系数：
         * p00 + p10*leftLegLength + p01*rightLegLength
         * + p20*leftLegLength^2 + p11*leftLegLength*rightLegLength
         * + p02*rightLegLength^2。
         * 当前系数来自已有 MATLAB 输出；若重新生成 0.15~0.35 m 范围系数，只替换本表。
         */
        .lqrKFitCoefficients = {
            [MODULE_CHASSIS_CONTROL_LEFT_WHEEL_TORQUE] = {
                [MODULE_CHASSIS_STATE_FORWARD_POSITION] = {-1.3234f, -1.9453f, 1.6626f, 4.8831f, -5.6891f, 1.0139f},
                [MODULE_CHASSIS_STATE_FORWARD_VELOCITY] = {-7.6994f, -1.4783f, 15.042f, 20.99f, -44.595f, 1.5428f},
                [MODULE_CHASSIS_STATE_YAW] = {-152.92f, 83.871f, -71.285f, -103.26f, 3.6875f, 86.762f},
                [MODULE_CHASSIS_STATE_YAW_RATE] = {-20.254f, 13.973f, -11.695f, -16.855f, 1.6244f, 14.372f},
                [MODULE_CHASSIS_STATE_LEFT_LEG_ANGLE] = {-34.165f, -73.688f, 34.495f, 97.998f, -10.286f, -54.874f},
                [MODULE_CHASSIS_STATE_LEFT_LEG_ANGLE_RATE] = {-2.1215f, -9.1199f, 4.9015f, 5.7824f, -3.9746f, -4.9331f},
                [MODULE_CHASSIS_STATE_RIGHT_LEG_ANGLE] = {-23.668f, 23.189f, -38.537f, -59.188f, 28.886f, 65.787f},
                [MODULE_CHASSIS_STATE_RIGHT_LEG_ANGLE_RATE] = {-1.6902f, 3.6341f, -5.288f, -3.4481f, -2.5243f, 6.529f},
                [MODULE_CHASSIS_STATE_BODY_PITCH] = {-100.46f, 133.41f, 125.05f, -161.02f, 16.828f, -146.83f},
                [MODULE_CHASSIS_STATE_BODY_PITCH_RATE] = {-3.789f, 5.7114f, 5.7979f, -4.6568f, -4.6315f, -5.2809f},
            },
            [MODULE_CHASSIS_CONTROL_RIGHT_WHEEL_TORQUE] = {
                [MODULE_CHASSIS_STATE_FORWARD_POSITION] = {-1.3234f, 1.6626f, -1.9453f, 1.0139f, -5.6891f, 4.8831f},
                [MODULE_CHASSIS_STATE_FORWARD_VELOCITY] = {-7.6994f, 15.042f, -1.4783f, 1.5428f, -44.595f, 20.99f},
                [MODULE_CHASSIS_STATE_YAW] = {152.92f, 71.285f, -83.871f, -86.762f, -3.6875f, 103.26f},
                [MODULE_CHASSIS_STATE_YAW_RATE] = {20.254f, 11.695f, -13.973f, -14.372f, -1.6244f, 16.855f},
                [MODULE_CHASSIS_STATE_LEFT_LEG_ANGLE] = {-23.668f, -38.537f, 23.189f, 65.787f, 28.886f, -59.188f},
                [MODULE_CHASSIS_STATE_LEFT_LEG_ANGLE_RATE] = {-1.6902f, -5.288f, 3.6341f, 6.529f, -2.5243f, -3.4481f},
                [MODULE_CHASSIS_STATE_RIGHT_LEG_ANGLE] = {-34.165f, 34.495f, -73.688f, -54.874f, -10.286f, 97.998f},
                [MODULE_CHASSIS_STATE_RIGHT_LEG_ANGLE_RATE] = {-2.1215f, 4.9015f, -9.1199f, -4.9331f, -3.9746f, 5.7824f},
                [MODULE_CHASSIS_STATE_BODY_PITCH] = {-100.46f, 125.05f, 133.41f, -146.83f, 16.828f, -161.02f},
                [MODULE_CHASSIS_STATE_BODY_PITCH_RATE] = {-3.789f, 5.7979f, 5.7114f, -5.2809f, -4.6315f, -4.6568f},
            },
            [MODULE_CHASSIS_CONTROL_LEFT_LEG_TORQUE] = {
                [MODULE_CHASSIS_STATE_FORWARD_POSITION] = {0.17577f, 2.0837f, -2.2709f, -2.6948f, 0.48182f, 2.1801f},
                [MODULE_CHASSIS_STATE_FORWARD_VELOCITY] = {0.9936f, 8.7965f, -11.408f, -12.985f, 4.4663f, 11.466f},
                [MODULE_CHASSIS_STATE_YAW] = {-10.428f, -20.157f, -4.948f, 36.103f, -19.834f, 12.698f},
                [MODULE_CHASSIS_STATE_YAW_RATE] = {-1.7101f, -4.8711f, -1.8786f, 7.517f, -6.0799f, 3.1898f},
                [MODULE_CHASSIS_STATE_LEFT_LEG_ANGLE] = {19.063f, 9.1258f, 4.5482f, 7.4727f, 45.51f, -18.01f},
                [MODULE_CHASSIS_STATE_LEFT_LEG_ANGLE_RATE] = {1.4704f, 2.4036f, 0.33937f, 2.1008f, 5.023f, -1.3049f},
                [MODULE_CHASSIS_STATE_RIGHT_LEG_ANGLE] = {-5.7809f, -20.444f, -19.528f, 41.5f, -54.433f, 5.7709f},
                [MODULE_CHASSIS_STATE_RIGHT_LEG_ANGLE_RATE] = {-0.56498f, -2.2252f, -1.8392f, 3.5152f, -5.397f, -2.5115f},
                [MODULE_CHASSIS_STATE_BODY_PITCH] = {-73.53f, -60.329f, 17.952f, 79.705f, 6.1753f, -27.54f},
                [MODULE_CHASSIS_STATE_BODY_PITCH_RATE] = {-2.8907f, -4.6372f, 2.0337f, 4.7791f, 1.0743f, -2.3467f},
            },
            [MODULE_CHASSIS_CONTROL_RIGHT_LEG_TORQUE] = {
                [MODULE_CHASSIS_STATE_FORWARD_POSITION] = {0.17577f, -2.2709f, 2.0837f, 2.1801f, 0.48182f, -2.6948f},
                [MODULE_CHASSIS_STATE_FORWARD_VELOCITY] = {0.9936f, -11.408f, 8.7965f, 11.466f, 4.4663f, -12.985f},
                [MODULE_CHASSIS_STATE_YAW] = {10.428f, 4.948f, 20.157f, -12.698f, 19.834f, -36.103f},
                [MODULE_CHASSIS_STATE_YAW_RATE] = {1.7101f, 1.8786f, 4.8711f, -3.1898f, 6.0799f, -7.517f},
                [MODULE_CHASSIS_STATE_LEFT_LEG_ANGLE] = {-5.7809f, -19.528f, -20.444f, 5.7709f, -54.433f, 41.5f},
                [MODULE_CHASSIS_STATE_LEFT_LEG_ANGLE_RATE] = {-0.56498f, -1.8392f, -2.2252f, -2.5115f, -5.397f, 3.5152f},
                [MODULE_CHASSIS_STATE_RIGHT_LEG_ANGLE] = {19.063f, 4.5482f, 9.1258f, -18.01f, 45.51f, 7.4727f},
                [MODULE_CHASSIS_STATE_RIGHT_LEG_ANGLE_RATE] = {1.4704f, 0.33937f, 2.4036f, -1.3049f, 5.023f, 2.1008f},
                [MODULE_CHASSIS_STATE_BODY_PITCH] = {-73.53f, 17.952f, -60.329f, -27.54f, 6.1753f, 79.705f},
                [MODULE_CHASSIS_STATE_BODY_PITCH_RATE] = {-2.8907f, 2.0337f, -4.6372f, -2.3467f, 1.0743f, 4.7791f},
            },
        },
    },
    .output = {
        .jointTorqueOutputEnabled = 0U,
        .wheelCurrentOutputEnabled = 0U,
        .jointTorqueLimitNm = 0.0f,
    },
    .legVerticalAngleOffsetRad = MODULE_CHASSIS_MODEL_HALF_PI,
    .targetRollRad = 0.0f,
    .baseSupportForceN = -30.0f,
    .leftSupportForceFeedforwardN = 0.0f,
    .rightSupportForceFeedforwardN = 0.0f,
    .defaultDtSec = APP_CONFIG_IMU_DEFAULT_DT_SEC,
    .minDtSec = 0.0002f,
    .maxDtSec = 0.02f,
    .targetState = {0.0f},
    .fixedLqrK = {{0.0f}},
};

const module_chassis_model_config_t *Module_Chassis_Model_GetDefaultConfig(void)
{
    return &chassisDefaultModelConfig;
}
