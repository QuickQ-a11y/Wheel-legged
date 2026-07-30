#include "chassis_config.h"

/*
 * 整车业务坐标使用右手系：X 前、Y 左、Z 上。
 * IMU 任务已经完成坐标转换，本文件中的 scale 只匹配控制模型正方向。
 * DM 顺序为左前、左后、右前、右后；DJI 顺序为左轮、右轮。
 */
const chassis_config_t chassis_config = {
    /* 左右腿尺寸相同，但电机索引和后续实机标定值分别保存。 */
    .leg = {
        [CHASSIS_LEFT] = {
            .geometry = {
                .link1_m = 0.215f,
                .link2_m = 0.258f,
                .link3_m = 0.258f,
                .link4_m = 0.215f,
                .frame_joint_distance_m = 0.0f,
                .min_leg_length_m = 0.05f,
            },
            .joint = {
                [CHASSIS_JOINT_FRONT] = {
                    .motor_index = 0U,
                    .angle_offset_rad = CHASSIS_PI,
                    .angle_scale = -1.0f,
                    .torque_scale = -1.0f,
                },
                [CHASSIS_JOINT_BACK] = {
                    .motor_index = 1U,
                    .angle_offset_rad = 0.0f,
                    .angle_scale = -1.0f,
                    .torque_scale = -1.0f,
                },
            },
            .target_leg_length_m = 0.25f,
        },
        [CHASSIS_RIGHT] = {
            .geometry = {
                .link1_m = 0.215f,
                .link2_m = 0.258f,
                .link3_m = 0.258f,
                .link4_m = 0.215f,
                .frame_joint_distance_m = 0.0f,
                .min_leg_length_m = 0.05f,
            },
            .joint = {
                [CHASSIS_JOINT_FRONT] = {
                    .motor_index = 2U,
                    .angle_offset_rad = CHASSIS_PI,
                    .angle_scale = -1.0f,
                    .torque_scale = -1.0f,
                },
                [CHASSIS_JOINT_BACK] = {
                    .motor_index = 3U,
                    .angle_offset_rad = 0.0f,
                    .angle_scale = -1.0f,
                    .torque_scale = -1.0f,
                },
            },
            .target_leg_length_m = 0.25f,
        },
    },
    /* IMU任务已输出整车右手系数据，此处只匹配控制模型的轴和正方向。 */
    .imu = {
        .pitch_rate_axis = 1U,
        .roll_rate_axis = 0U,
        .yaw_rate_axis = 2U,
        .pitch_angle_scale = 1.0f,
        .pitch_rate_scale = 1.0f,
        .roll_angle_scale = 1.0f,
        .roll_rate_scale = 1.0f,
        .yaw_angle_scale = 1.0f,
        .yaw_rate_scale = 1.0f,
        .forward_accel_axis = 0U,
        .forward_accel_scale = 1.0f,
    },
    /* 轮速用于车体速度观测，轮力矩请求最终换算为DJI原始电流。 */
    .wheel = {
        .radius_m = 0.10f,
        .half_track_m = 0.1965f,
        .left_speed_scale = 1.0f,
        .right_speed_scale = 1.0f,
        /*
         * 当前轮电机：24 V 直驱，传动比 1，转矩常数 0.02 N*m/A。
         * 空载 9400 rpm/0.6 A，额定 9085 rpm/0.16 N*m/10 A。
         * C620 的 16384 对应 20 A：
         * torque_to_current = 16384 / (20 * 0.02) = 40960 count/(N*m)
         * current_limit = 16384 * 10 / 20 = 8192 count
         * 更换轮电机时在此处重算转矩换算、连续转矩和额定电流限幅。
         */
        .torque_limit_nm = 0.16f,
        .torque_to_current = 40960.0f,
        .current_limit = 8192,
    },
    /* 状态为前进速度和前进加速度，矩阵按2x2行优先顺序填写。 */
    .speed_kalman = {
        .enabled = 1U,
        .initial_covariance = {
            1.0f, 0.0f,
            0.0f, 1.0f,
        },
        .process_noise = {
            0.1f, 0.0f,
            0.0f, 0.1f,
        },
        .measurement_noise = {
            100.0f, 0.0f,
            0.0f, 1.0e12f,
        },
        .position_speed_limit_mps = 0.1f,
    },
    /* 腿长PID输出作为虚拟支撑力修正，反馈速度直接作为阻尼项。 */
    .leg_length_pid = {
        .kp = 400.0f,
        .ki = 2.0f,
        .kd = 8000.0f,
        .integralLimit = 50.0f,
        .outputLimit = 300.0f,
    },
    /* roll PID输出以左右腿差动支撑力的形式作用。 */
    .roll_pid = {
        .kp = 3000.0f,
        .ki = 1.0f,
        .kd = 100.0f,
        .integralLimit = 30.0f,
        .outputLimit = 300.0f,
    },
    /* 倒地转腿、小板凳准备和关节串级位置控制参数。 */
    .recovery = {
        /*
         * 动作阶段参考 SPR，两端腿长改为本工程当前 0.15~0.35 m 工作范围。
         * 串级 PID 和 1 N*m 请求限幅是输出封锁阶段的保守调试初值，后续按实机修改。
         */
        .bench_leg_length_m = 0.15f,
        .extended_leg_length_m = 0.35f,
        .bench_phi0_rad = CHASSIS_HALF_PI,
        .rotate_offset_rad = 0.30f,
        .lagging_rotate_offset_rad = 0.60f,
        .leg_difference_threshold_rad = 0.80f,
        .ready_theta_min_rad = 0.50f,
        .ready_theta_max_rad = 1.40f,
        .direct_prepare_pitch_rad = 0.80f,
        .ready_pitch_rad = 0.30f,
        .direct_phi0_min_rad = 0.70f,
        .direct_phi0_max_rad = 3.00f,
        .leg_length_tolerance_m = 0.02f,
        .leg_angle_tolerance_rad = 0.10f,
        .stable_time_s = 0.10f,
        .fallen_timeout_s = 5.0f,
        .prepare_timeout_s = 3.0f,
        .standing_length_rate_mps = 0.10f,
        .standing_pitch_limit_rad = 1.60f,
        .standing_phi0_min_rad = 0.40f,
        .standing_phi0_max_rad = 2.80f,
        .joint_torque_limit_nm = 1.0f,
        .joint_angle_pid = {
            .kp = 4.0f,
            .ki = 0.0f,
            .kd = 0.0f,
            .integralLimit = 0.0f,
            .outputLimit = 1.0f,
        },
        .joint_speed_pid = {
            .kp = 1.0f,
            .ki = 0.0f,
            .kd = 0.0f,
            .integralLimit = 0.0f,
            .outputLimit = 1.0f,
        },
    },
    /* 四路输出、十个状态分别保存一组双腿长poly22系数。 */
    .lqr = {
        .enabled = 1U,
        .length_source = CHASSIS_K_LENGTH_FIXED,
        .min_leg_length_m = 0.15f,
        .max_leg_length_m = 0.35f,
        .fixed_left_length_m = 0.25f,
        .fixed_right_length_m = 0.25f,
        /*
         * MATLAB poly22 顺序：p00、p10、p01、p20、p11、p02。
         * 两个输入依次为左腿长和右腿长，单位 m。
         */
        .coefficients = {
            [CHASSIS_OUTPUT_LEFT_WHEEL] = {
                [CHASSIS_STATE_S] = {-1.3234f, -1.9453f, 1.6626f, 4.8831f, -5.6891f, 1.0139f},
                [CHASSIS_STATE_DOT_S] = {-7.6994f, -1.4783f, 15.042f, 20.99f, -44.595f, 1.5428f},
                [CHASSIS_STATE_FAI] = {-152.92f, 83.871f, -71.285f, -103.26f, 3.6875f, 86.762f},
                [CHASSIS_STATE_DOT_FAI] = {-20.254f, 13.973f, -11.695f, -16.855f, 1.6244f, 14.372f},
                [CHASSIS_STATE_THETA_L] = {-34.165f, -73.688f, 34.495f, 97.998f, -10.286f, -54.874f},
                [CHASSIS_STATE_DOT_THETA_L] = {-2.1215f, -9.1199f, 4.9015f, 5.7824f, -3.9746f, -4.9331f},
                [CHASSIS_STATE_THETA_R] = {-23.668f, 23.189f, -38.537f, -59.188f, 28.886f, 65.787f},
                [CHASSIS_STATE_DOT_THETA_R] = {-1.6902f, 3.6341f, -5.288f, -3.4481f, -2.5243f, 6.529f},
                [CHASSIS_STATE_THETA_B] = {-100.46f, 133.41f, 125.05f, -161.02f, 16.828f, -146.83f},
                [CHASSIS_STATE_DOT_THETA_B] = {-3.789f, 5.7114f, 5.7979f, -4.6568f, -4.6315f, -5.2809f},
            },
            [CHASSIS_OUTPUT_RIGHT_WHEEL] = {
                [CHASSIS_STATE_S] = {-1.3234f, 1.6626f, -1.9453f, 1.0139f, -5.6891f, 4.8831f},
                [CHASSIS_STATE_DOT_S] = {-7.6994f, 15.042f, -1.4783f, 1.5428f, -44.595f, 20.99f},
                [CHASSIS_STATE_FAI] = {152.92f, 71.285f, -83.871f, -86.762f, -3.6875f, 103.26f},
                [CHASSIS_STATE_DOT_FAI] = {20.254f, 11.695f, -13.973f, -14.372f, -1.6244f, 16.855f},
                [CHASSIS_STATE_THETA_L] = {-23.668f, -38.537f, 23.189f, 65.787f, 28.886f, -59.188f},
                [CHASSIS_STATE_DOT_THETA_L] = {-1.6902f, -5.288f, 3.6341f, 6.529f, -2.5243f, -3.4481f},
                [CHASSIS_STATE_THETA_R] = {-34.165f, 34.495f, -73.688f, -54.874f, -10.286f, 97.998f},
                [CHASSIS_STATE_DOT_THETA_R] = {-2.1215f, 4.9015f, -9.1199f, -4.9331f, -3.9746f, 5.7824f},
                [CHASSIS_STATE_THETA_B] = {-100.46f, 125.05f, 133.41f, -146.83f, 16.828f, -161.02f},
                [CHASSIS_STATE_DOT_THETA_B] = {-3.789f, 5.7979f, 5.7114f, -5.2809f, -4.6315f, -4.6568f},
            },
            [CHASSIS_OUTPUT_LEFT_LEG] = {
                [CHASSIS_STATE_S] = {0.17577f, 2.0837f, -2.2709f, -2.6948f, 0.48182f, 2.1801f},
                [CHASSIS_STATE_DOT_S] = {0.9936f, 8.7965f, -11.408f, -12.985f, 4.4663f, 11.466f},
                [CHASSIS_STATE_FAI] = {-10.428f, -20.157f, -4.948f, 36.103f, -19.834f, 12.698f},
                [CHASSIS_STATE_DOT_FAI] = {-1.7101f, -4.8711f, -1.8786f, 7.517f, -6.0799f, 3.1898f},
                [CHASSIS_STATE_THETA_L] = {19.063f, 9.1258f, 4.5482f, 7.4727f, 45.51f, -18.01f},
                [CHASSIS_STATE_DOT_THETA_L] = {1.4704f, 2.4036f, 0.33937f, 2.1008f, 5.023f, -1.3049f},
                [CHASSIS_STATE_THETA_R] = {-5.7809f, -20.444f, -19.528f, 41.5f, -54.433f, 5.7709f},
                [CHASSIS_STATE_DOT_THETA_R] = {-0.56498f, -2.2252f, -1.8392f, 3.5152f, -5.397f, -2.5115f},
                [CHASSIS_STATE_THETA_B] = {-73.53f, -60.329f, 17.952f, 79.705f, 6.1753f, -27.54f},
                [CHASSIS_STATE_DOT_THETA_B] = {-2.8907f, -4.6372f, 2.0337f, 4.7791f, 1.0743f, -2.3467f},
            },
            [CHASSIS_OUTPUT_RIGHT_LEG] = {
                [CHASSIS_STATE_S] = {0.17577f, -2.2709f, 2.0837f, 2.1801f, 0.48182f, -2.6948f},
                [CHASSIS_STATE_DOT_S] = {0.9936f, -11.408f, 8.7965f, 11.466f, 4.4663f, -12.985f},
                [CHASSIS_STATE_FAI] = {10.428f, 4.948f, 20.157f, -12.698f, 19.834f, -36.103f},
                [CHASSIS_STATE_DOT_FAI] = {1.7101f, 1.8786f, 4.8711f, -3.1898f, 6.0799f, -7.517f},
                [CHASSIS_STATE_THETA_L] = {-5.7809f, -19.528f, -20.444f, 5.7709f, -54.433f, 41.5f},
                [CHASSIS_STATE_DOT_THETA_L] = {-0.56498f, -1.8392f, -2.2252f, -2.5115f, -5.397f, 3.5152f},
                [CHASSIS_STATE_THETA_R] = {19.063f, 4.5482f, 9.1258f, -18.01f, 45.51f, 7.4727f},
                [CHASSIS_STATE_DOT_THETA_R] = {1.4704f, 0.33937f, 2.4036f, -1.3049f, 5.023f, 2.1008f},
                [CHASSIS_STATE_THETA_B] = {-73.53f, 17.952f, -60.329f, -27.54f, 6.1753f, 79.705f},
                [CHASSIS_STATE_DOT_THETA_B] = {-2.8907f, 2.0337f, -4.6372f, -2.3467f, 1.0743f, 4.7791f},
            },
        },
    },
    /* 最终物理输出开关；关闭不影响请求量和控制中间量计算。 */
    .output = {
        /*
         * 当前髋关节电机：24 V，额定连续转矩 3.5 N*m。
         * 11/12.5 N*m 峰值未配置持续时间保护，因此暂不作为运行限幅。
         * MIT 协议映射量程在 app_config.h 中配置，当前为 +/-40 N*m。
         */
        .joint_enabled = 0U,
        .wheel_enabled = 0U,
        .joint_torque_limit_nm = 3.5f,
    },
    /* 整车公共目标、支撑力前馈、控制周期边界和固定K备用表。 */
    .leg_vertical_offset_rad = CHASSIS_HALF_PI,
    .target_roll_rad = 0.0f,
    .base_support_force_n = -30.0f,
    .left_support_feedforward_n = 0.0f,
    .right_support_feedforward_n = 0.0f,
    .default_dt_s = APP_CTRL_DT_S,
    .min_dt_s = 0.0002f,
    .max_dt_s = 0.02f,
    .target_state = {0.0f},
    .fixed_lqr_k = {{0.0f}},
};
