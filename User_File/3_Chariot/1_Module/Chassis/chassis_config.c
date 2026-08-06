#include "chassis_config.h"

/*
 * 整车业务坐标使用右手系：X 前、Y 左、Z 上。
 * IMU 任务已经完成坐标转换，本文件中的 scale 匹配控制模型正方向，ratio
 * 描述同步带传动的角度比例。
 * DM 顺序为0左前、1左后、2右前、3右后；DJI 顺序为0左轮、1右轮。
 * 另外，前髋关节电机统一控制phi4对应的主动杆，后髋关节电机统一控制phi1对应的主动杆。
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
        .lateral_accel_axis = 1U,
        .vertical_accel_axis = 2U,
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
        .kp = 4.0f,
        .ki = 0.0f,
        .kd = 0.0f,
        .integralLimit = 5.0f,
        .outputLimit = 10.0f,
    },
    /* roll PID输出以左右腿差动支撑力的形式作用。 */
    .roll_pid = {
        .kp = 3.0f,
        .ki = 0.0f,
        .kd = 0.0f,
        .integralLimit = 5.0f,
        .outputLimit = 10.0f,
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
        .joint_T_limit = 3.5f,
        /* 板凳模式下用左右摇杆分别微调两条腿，范围保守收在机构可达区内。 */
        .bench_L0_rate = 0.80f,
        .bench_phi0_rate = 3.20f,
        .bench_L0_min = 0.09f,
        .bench_L0_max = 0.25f,
        .joint_angle_pid = {
            .kp = 8.0f,
            .ki = 0.0f,
            .kd = 0.0f,
            .integralLimit = 0.0f,
            .outputLimit = 5.0f,
        },
        .joint_speed_pid = {
            .kp = 2.0f,
            .ki = 0.0f,
            .kd = 0.0f,
            .integralLimit = 0.0f,
            .outputLimit = 5.0f,
        },
    },
    /* 小陀螺保留速度和姿态反馈，关闭位移与航向角位置反馈。 */
    .top = {
        .max_d_s = 0.25f,
        .max_d_fai = 2.0f,
        .scale = {
            [CHASSIS_STATE_S] = 0.0f,
            [CHASSIS_STATE_D_S] = 1.0f,
            [CHASSIS_STATE_FAI] = 0.0f,
            [CHASSIS_STATE_D_FAI] = 1.0f,
            [CHASSIS_STATE_THETA_L] = 1.0f,
            [CHASSIS_STATE_D_THETA_L] = 1.0f,
            [CHASSIS_STATE_THETA_R] = 1.0f,
            [CHASSIS_STATE_D_THETA_R] = 1.0f,
            [CHASSIS_STATE_THETA_B] = 1.0f,
            [CHASSIS_STATE_D_THETA_B] = 1.0f,
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
     * 当前MATLAB脚本实际采样0.11~0.25 m，运行时按该边界限幅，
     * 禁止在采样范围外继续外推。
     */
    .lqr = {
        .L0_min = 0.11f,
        .L0_max = 0.25f,
        /*
         * MATLAB poly22 顺序：p00、p10、p01、p20、p11、p02。
         * 两个输入依次为左腿长和右腿长，单位 m。
         */
        .coefficients = {
            [CHASSIS_OUTPUT_LEFT_WHEEL] = {
                [CHASSIS_STATE_S] = {-1.2125f, -0.75857f, 1.4572f, 4.7369f, -9.7781f, 2.298f},
                [CHASSIS_STATE_D_S] = {-7.8236f, 7.3862f, 17.967f, 21.668f, -85.008f, 7.924f},
                [CHASSIS_STATE_FAI] = {-155.05f, 38.018f, -31.416f, -39.098f, 1.2544f, 32.948f},
                [CHASSIS_STATE_D_FAI] = {-20.27f, 8.2017f, -7.3579f, -8.3126f, -0.716f, 8.9617f},
                [CHASSIS_STATE_THETA_L] = {-22.158f, -82.33f, 30.231f, 117.52f, -13.138f, -46.914f},
                [CHASSIS_STATE_D_THETA_L] = {-2.5816f, -5.7576f, 8.3528f, 8.172f, -15.318f, -5.717f},
                [CHASSIS_STATE_THETA_R] = {-20.115f, 32.773f, -50.669f, -55.369f, -10.951f, 82.976f},
                [CHASSIS_STATE_D_THETA_R] = {-2.5054f, 7.026f, -2.6441f, -2.4952f, -17.992f, 7.386f},
                [CHASSIS_STATE_THETA_B] = {-154.37f, 100.43f, 80.457f, -82.492f, 47.532f, -68.928f},
                [CHASSIS_STATE_D_THETA_B] = {-2.4906f, 2.5371f, 2.5951f, -1.7526f, -2.9345f, -2.0565f},
            },
            [CHASSIS_OUTPUT_RIGHT_WHEEL] = {
                [CHASSIS_STATE_S] = {-1.2125f, 1.4572f, -0.75857f, 2.298f, -9.7781f, 4.7369f},
                [CHASSIS_STATE_D_S] = {-7.8236f, 17.967f, 7.3862f, 7.924f, -85.008f, 21.668f},
                [CHASSIS_STATE_FAI] = {155.05f, 31.416f, -38.018f, -32.948f, -1.2544f, 39.098f},
                [CHASSIS_STATE_D_FAI] = {20.27f, 7.3579f, -8.2017f, -8.9617f, 0.716f, 8.3126f},
                [CHASSIS_STATE_THETA_L] = {-20.115f, -50.669f, 32.773f, 82.976f, -10.951f, -55.369f},
                [CHASSIS_STATE_D_THETA_L] = {-2.5054f, -2.6441f, 7.026f, 7.386f, -17.992f, -2.4952f},
                [CHASSIS_STATE_THETA_R] = {-22.158f, 30.231f, -82.33f, -46.914f, -13.138f, 117.52f},
                [CHASSIS_STATE_D_THETA_R] = {-2.5816f, 8.3528f, -5.7576f, -5.717f, -15.318f, 8.172f},
                [CHASSIS_STATE_THETA_B] = {-154.37f, 80.457f, 100.43f, -68.928f, 47.532f, -82.492f},
                [CHASSIS_STATE_D_THETA_B] = {-2.4906f, 2.5951f, 2.5371f, -2.0565f, -2.9345f, -1.7526f},
            },
            [CHASSIS_OUTPUT_LEFT_LEG] = {
                [CHASSIS_STATE_S] = {0.27653f, 3.6527f, -3.0729f, -6.5234f, 0.9018f, 3.7349f},
                [CHASSIS_STATE_D_S] = {1.9305f, 13.996f, -17.857f, -31.846f, 16.038f, 20.3f},
                [CHASSIS_STATE_FAI] = {-1.9765f, -34.802f, -0.28435f, 52.274f, -18.155f, 11.331f},
                [CHASSIS_STATE_D_FAI] = {-0.30364f, -6.8657f, -0.1079f, 8.9093f, -3.4484f, 0.7038f},
                [CHASSIS_STATE_THETA_L] = {22.554f, 4.889f, -1.8511f, -3.463f, 13.151f, -5.1166f},
                [CHASSIS_STATE_D_THETA_L] = {1.9959f, 3.8029f, -2.3357f, -3.0134f, 3.4635f, 3.0271f},
                [CHASSIS_STATE_THETA_R] = {-2.5045f, -38.323f, 19.851f, 59.608f, -17.956f, -47.186f},
                [CHASSIS_STATE_D_THETA_R] = {-0.12988f, -3.1528f, -1.1109f, 0.99243f, 2.4422f, -2.645f},
                [CHASSIS_STATE_THETA_B] = {-53.483f, -49.159f, -26.229f, 56.988f, 6.0632f, 15.025f},
                [CHASSIS_STATE_D_THETA_B] = {-0.82815f, -1.6525f, -0.65872f, 1.4711f, 1.1762f, 0.42385f},
            },
            [CHASSIS_OUTPUT_RIGHT_LEG] = {
                [CHASSIS_STATE_S] = {0.27653f, -3.0729f, 3.6527f, 3.7349f, 0.9018f, -6.5234f},
                [CHASSIS_STATE_D_S] = {1.9305f, -17.857f, 13.996f, 20.3f, 16.038f, -31.846f},
                [CHASSIS_STATE_FAI] = {1.9765f, 0.28435f, 34.802f, -11.331f, 18.155f, -52.274f},
                [CHASSIS_STATE_D_FAI] = {0.30364f, 0.1079f, 6.8657f, -0.7038f, 3.4484f, -8.9093f},
                [CHASSIS_STATE_THETA_L] = {-2.5045f, 19.851f, -38.323f, -47.186f, -17.956f, 59.608f},
                [CHASSIS_STATE_D_THETA_L] = {-0.12988f, -1.1109f, -3.1528f, -2.645f, 2.4422f, 0.99243f},
                [CHASSIS_STATE_THETA_R] = {22.554f, -1.8511f, 4.889f, -5.1166f, 13.151f, -3.463f},
                [CHASSIS_STATE_D_THETA_R] = {1.9959f, -2.3357f, 3.8029f, 3.0271f, 3.4635f, -3.0134f},
                [CHASSIS_STATE_THETA_B] = {-53.483f, -26.229f, -49.159f, 15.025f, 6.0632f, 56.988f},
                [CHASSIS_STATE_D_THETA_B] = {-0.82815f, -0.65872f, -1.6525f, 0.42385f, 1.1762f, 1.4711f},
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
        .joint_flag = 1U,
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
