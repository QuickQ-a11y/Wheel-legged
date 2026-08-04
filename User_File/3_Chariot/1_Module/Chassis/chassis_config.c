#include "chassis_config.h"

/*
 * 整车业务坐标使用右手系：X 前、Y 左、Z 上。
 * IMU 任务已经完成坐标转换，本文件中的 scale 匹配控制模型正方向，ratio
 * 描述同步带传动的角度比例。
 * DM 顺序为左前、左后、右前、右后；DJI 顺序为左轮、右轮。
 */
const Chassis_Config_t Chassis_Config = {
    /* 左右腿尺寸相同，但电机索引和后续实机标定值分别保存。 */
    .leg = {
        [CHASSIS_LEFT] = {
            .geometry = {
                .l1 = 0.13787f,
                .l2 = 0.15817f,
                .l3 = 0.15817f,
                .l4 = 0.13787f,
                .l5 = 0.0f,
            },
            .joint = {
                [CHASSIS_JOINT_PHI1] = {
                    .motor_index = 1U,
                    .angle_offset_rad = CHASSIS_HALF_PI,
                    .scale = 1.0f,
                    .ratio = 0.75f,
                },
                [CHASSIS_JOINT_PHI4] = {
                    .motor_index = 0U,
                    .angle_offset_rad = CHASSIS_HALF_PI,
                    .scale = 1.0f,
                    .ratio = 0.75f,
                },
            },
            .target_L0 = 0.25f,
        },
        [CHASSIS_RIGHT] = {
            .geometry = {
                .l1 = 0.13787f,
                .l2 = 0.15817f,
                .l3 = 0.15817f,
                .l4 = 0.13787f,
                .l5 = 0.0f,
            },
            .joint = {
                [CHASSIS_JOINT_PHI1] = {
                    .motor_index = 3U,
                    .angle_offset_rad = CHASSIS_HALF_PI,
                    .scale = -1.0f,
                    .ratio = 0.75f,
                },
                [CHASSIS_JOINT_PHI4] = {
                    .motor_index = 2U,
                    .angle_offset_rad = CHASSIS_HALF_PI,
                    .scale = -1.0f,
                    .ratio = 0.75f,
                },
            },
            .target_L0 = 0.25f,
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
    /* 左右scale同时约束反馈和命令，避免同一轮方向在两处独立维护。 */
    .wheel = {
        .R = 0.10f,
        .half_track = 0.1965f,
        .left_scale = 1.0f,
        .right_scale = 1.0f,
        /*
         * 当前轮电机：24 V 直驱，传动比 1，转矩常数 0.02 N*m/A。
         * 空载 9400 rpm/0.6 A，额定 9085 rpm/0.16 N*m/10 A。
         * C620 的 16384 对应 20 A：
         * T_to_I = 16384 / (20 * 0.02) = 40960 count/(N*m)
         * I_limit = 16384 * 10 / 20 = 8192 count
         * 更换轮电机时在此处重算转矩换算、连续转矩和额定电流限幅。
         */
        .T_limit = 0.16f,
        .T_to_I = 40960.0f,
        .I_limit = 8192,
    },
    /* 状态为前进速度和前进加速度，矩阵按2x2行优先顺序填写。 */
    .speed_kalman = {
        .enable_flag = 1U,
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
            /*
             * 加速度权重暂时保持关闭量级，待采集实车噪声后再标定Q/R；
             * 当前不能描述为已经完成IMU加速度融合。
             */
            0.0f, 1.0e12f,
        },
        .position_d_s_limit = 0.1f,
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
         * 动作阶段参考 SPR，目标限制在本机构约0.296 m的最大可达腿长内。
         * 串级 PID 和 1 N*m 请求限幅是输出封锁阶段的保守调试初值，后续按实机修改。
         */
        .bench_L0 = 0.15f,
        .extend_L0 = 0.29f,
        .bench_phi0 = CHASSIS_HALF_PI,
        .rotate_phi0 = 0.30f,
        .lag_phi0 = 0.60f,
        .theta_diff = 0.80f,
        .theta_min = 0.50f,
        .theta_max = 1.40f,
        .direct_pitch = 0.80f,
        .ready_pitch = 0.30f,
        .phi0_min = 0.70f,
        .phi0_max = 3.00f,
        .L0_tol = 0.02f,
        .angle_tol = 0.10f,
        .stable_time = 0.10f,
        .fallen_timeout = 5.0f,
        .prepare_timeout = 3.0f,
        .L0_rate = 0.10f,
        .pitch_limit = 1.60f,
        .stand_phi0_min = 0.40f,
        .stand_phi0_max = 2.80f,
        .joint_T_limit = 1.0f,
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
    /* 小陀螺保留速度和姿态反馈，关闭位移与航向角位置反馈。 */
    .top = {
        .max_d_s = 0.25f,
        .max_d_fai = 2.0f,
        .scale = {
            [CHASSIS_STATE_S] = 0.0f,
            [CHASSIS_STATE_DOT_S] = 1.0f,
            [CHASSIS_STATE_FAI] = 0.0f,
            [CHASSIS_STATE_DOT_FAI] = 1.0f,
            [CHASSIS_STATE_THETA_L] = 1.0f,
            [CHASSIS_STATE_DOT_THETA_L] = 1.0f,
            [CHASSIS_STATE_THETA_R] = 1.0f,
            [CHASSIS_STATE_DOT_THETA_R] = 1.0f,
            [CHASSIS_STATE_THETA_B] = 1.0f,
            [CHASSIS_STATE_DOT_THETA_B] = 1.0f,
        },
    },
    /* 当前均为输出封锁阶段的保守调试初值。 */
    .step = {
        .approach_L0 = 0.29f,
        .retract_L0 = 0.15f,
        .approach_d_s = 0.10f,
        .contact_T_req = 0.12f,
        .contact_T_fb = 0.08f,
        .contact_theta = 0.30f,
        .contact_time = 0.05f,
        .peak_theta = 0.80f,
        .recover_theta = 0.0f,
        .L0_tol = 0.02f,
        .angle_tol = 0.10f,
        .stable_time = 0.10f,
        .prepare_timeout = 3.0f,
        .approach_timeout = 10.0f,
        .climb_timeout = 3.0f,
        .recover_timeout = 2.0f,
        .leg_angle_pid = {
            .kp = 4.0f,
            .ki = 0.0f,
            .kd = 0.2f,
            .integralLimit = 0.0f,
            .outputLimit = 1.0f,
        },
    },
    /* 质量来自现有MATLAB名义模型，仅用于生成Watch估计量。 */
    .observer = {
        .gravity_mps2 = 9.81f,
        .body_mass_kg = 10.0f,
        .leg_mass_kg = 0.5f,
        .wheel_mass_kg = 1.0f,
        .body_cg_to_hip_m = 0.04f,
        .residual_filter_s = 0.02f,
        .turn_filter_s = 0.05f,
        .normal_force_filter_s = 0.02f,
        .slip_speed_enter_mps = 0.20f,
        .slip_speed_exit_mps = 0.10f,
        .slip_yaw_enter_radps = 0.80f,
        .slip_yaw_exit_radps = 0.40f,
        .slip_delta_enter_mps = 0.03f,
        .slip_delta_exit_mps = 0.015f,
        .slip_enter_s = 0.05f,
        .slip_exit_s = 0.20f,
        .off_force_ratio = 0.20f,
        .land_force_ratio = 0.35f,
        .off_hold_s = 0.03f,
        .land_hold_s = 0.05f,
        .turn_force_limit_ratio = 0.50f,
    },
    /*
     * 四路输出、十个状态分别保存一组双腿长poly22系数。
     * 现有MATLAB脚本实际采样0.12~0.31 m；重新生成系数前按真实
     * 采样边界限幅，禁止0.31 m以上继续外推。
     */
    .lqr = {
        .L0_min = 0.12f,
        .L0_max = 0.31f,
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
        .joint_flag = 0U,
        .wheel_flag = 0U,
        .joint_T_limit = 3.5f,
    },
    /* 整车公共目标、支撑力前馈和控制周期边界。 */
    .phi0_offset = CHASSIS_HALF_PI,
    .roll_target = 0.0f,
    .F0_base = -30.0f,
    .F0_left = 0.0f,
    .F0_right = 0.0f,
    .default_dt = APP_CTRL_DT_S,
    .dt_min = 0.0002f,
    .dt_max = 0.02f,
    .target = {0.0f},
};
