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
    /* 小轮腿 */
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
            .target_L0 = 0.15f,
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
            .target_L0 = 0.15f,
        },
    },
    /* 大轮腿 */
    // .leg = {
    //     [CHASSIS_LEFT] = {
    //         .geometry = {
    //             .l1 = 0.13787f,
    //             .l2 = 0.15817f,
    //             .l3 = 0.15817f,
    //             .l4 = 0.13787f,
    //             .l5 = 0.0f,
    //         },
    //         .joint = {
    //             [CHASSIS_JOINT_PHI1] = {
    //                 .motor_index = 1U,
    //                 .angle_offset_rad = CHASSIS_HALF_PI,
    //                 .scale = 1.0f,
    //                 .ratio = 1.0f,
    //             },
    //             [CHASSIS_JOINT_PHI4] = {
    //                 .motor_index = 0U,
    //                 .angle_offset_rad = CHASSIS_HALF_PI,
    //                 .scale = 1.0f,
    //                 .ratio = 1.0f,
    //             },
    //         },
    //         .target_L0 = 0.25f,
    //     },
    //     [CHASSIS_RIGHT] = {
    //         .geometry = {
    //             .l1 = 0.13787f,
    //             .l2 = 0.15817f,
    //             .l3 = 0.15817f,
    //             .l4 = 0.13787f,
    //             .l5 = 0.0f,
    //         },
    //         .joint = {
    //             [CHASSIS_JOINT_PHI1] = {
    //                 .motor_index = 3U,
    //                 .angle_offset_rad = CHASSIS_HALF_PI,
    //                 .scale = -1.0f,
    //                 .ratio = 1.0f,
    //             },
    //             [CHASSIS_JOINT_PHI4] = {
    //                 .motor_index = 2U,
    //                 .angle_offset_rad = CHASSIS_HALF_PI,
    //                 .scale = -1.0f,
    //                 .ratio = 1.0f,
    //             },
    //         },
    //         .target_L0 = 0.25f,
    //     },
    // },
    /*
     * 整车质量与质心参数，与生成K的 ABK_LQR.m 保持一一对应，
     * 重力前馈和全部力类观测阈值都由这里推出。
     * 换机器人时本块和 leg.geometry、wheel 一起改，按比例定义的阈值自动跟随。
     *
     * 整车合计 3.043 + 2*1.054 + 2*0.455 = 6.061 kg，单腿静载 29.7 N。
     */
    .model = {
        .gravity = 9.81f,     /* g_ac */
        .body_mass = 3.043f,  /* m_b_ac */
        .leg_mass = 1.054f,   /* m_l_ac */
        .wheel_mass = 0.455f, /* m_w_ac */
        .cg_to_hip = 0.060f,  /* l_c_ac */
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
    /* 小轮腿配置 */
    .wheel = {
        .R = 0.052f,
        .half_track = 0.1965f,
        /*
         * 当前轮电机：24 V 配减速箱，传动比 268/17，电机转矩常数约0.01562
         * N*m/A（对应输出轴等效0.2463 N*m/A）。
         * C620 反馈的speed_rpm是电机转子转速，轮轴角速度=speed_rpm/gear_ratio；
         * 直驱轮电机改这里为1.0。
         */
        .gear_ratio = 268.0f / 17.0f,
        .left_scale = -1.0f,
        .right_scale = 1.0f,
        /*
         * C620 的 16384 对应 20 A，故 T_to_I = 16384/(20*0.2463f) = 3326.02517 count/(N*m)，
         * 已按gear_ratio折算到轮轴等效转矩常数，T_to_I不再重复乘gear_ratio。
         *
         * 轮通道只保留 T_limit 一个限幅点，电调命令由 T_limit*T_to_I 推出。
         * 当前纯电机额定扭矩（最大连续转矩）为0.16N*m， 0.16*268/17 N*m
         * 换电机时改 T_to_I 和 T_limit，并自行确认换算结果不超过 16384。
         */
        .T_limit = 2.522f,
        .T_to_I = 3326.02517f,
    },
    // /* 大轮腿配置 */
    // .wheel = {
    //     .R = 0.10f,
    //     .half_track = 0.1965f,
    //     .gear_ratio = 268.0f / 17.0f,
    //     .left_scale = 1.0f,
    //     .right_scale = 1.0f,
    //     /*
    //      * 当前轮电机：24 V 配减速箱，传动比 268/17，转矩常数 0.2463f N*m/A。
    //      * C620 的 16384 对应 20 A，故 T_to_I = 16384/(20*0.2463f) = 3326.02517 count/(N*m)。
    //      *
    //      * 轮通道只保留 T_limit 一个限幅点，电调命令由 T_limit*T_to_I 推出。
    //      * 当前纯电机额定扭矩（最大连续转矩）为0.16N*m， 0.16*268/17 N*m
    //      * 换电机时改 T_to_I 和 T_limit，并自行确认换算结果不超过 16384。
    //      */
    //     .T_limit = 2.522f,
    //     .T_to_I = 3326.02517f,
    // },
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
        .kp = 350.0f,
        .ki = 0.0f,
        .kd = 0.0f,
        .integralLimit = 5.0f,
        .outputLimit = 30.0f,
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
        .recover_theta = 0.0f,
        .L0_tol = 0.02f,
        .angle_tol = 0.10f,
        .stable_time = 0.10f,
        .prepare_timeout = 3.0f,
        .approach_timeout = 10.0f,
        .climb_timeout = 3.0f,
        .recover_timeout = 2.0f,
        /*
         * 两段摆腿结构取自HERO_LEG的磕台阶控制，角度按本车机构缩小：
         * 原车腿杆角用到1.32/1.22 rad，本车站立phi0保护范围换算到相对角
         * 只有约-1.17~+1.23 rad，因此后摆保持角收到1.00/0.90。
         * 力矩按原车6.0/-15.0的比例缩放到本车关节限幅3.5 N*m量级。
         */
        .back_phi0_max = 1.00f,
        .back_phi0_hold = 0.90f,
        .back_Tp = 0.80f,
        .back_theta_exit = 0.90f,
        .front_theta_max = 0.95f,
        .front_phi0_hold = 0.85f,
        .front_Tp = -2.00f,
        .front_theta_exit = 0.45f,
        .home_phi0 = 0.15f,
        /* 相对站立的1.60/0.40/2.80适当放宽，避免磕台阶瞬间误判倒地。 */
        .pitch_limit = 1.80f,
        .phi0_min = 0.30f,
        .phi0_max = 2.90f,
        .leg_angle_pid = {
            .kp = 4.0f,
            .ki = 0.0f,
            .kd = 0.2f,
            .integralLimit = 0.0f,
            .outputLimit = 1.0f,
        },
    },
    /*
     * 只读观测阈值。凡是随整车重量或执行器能力等比缩放的量一律写成比例：
     * 力类以单腿静载 0.5*model.mass*model.gravity 为基准，
     * 力矩类以 wheel.T_limit 为基准。换机器人时改 model 和 wheel 即可跟随，
     * 不需要重算这些比例。角度、时间和速度是运动学量，换车需要单独整定。
     */
    .observer = {
        .residual_filter_s = 0.02f,
        .turn_filter_s = 0.05f,
        .normal_force_filter_s = 0.02f,
        .slip_gate_yaw = 0.90f,
        /* 闸门速度差应高于常用行驶速度，避免正常加减速误判。 */
        .slip_gate_v = 0.60f,
        .slip_v_enter = 0.20f,
        .slip_yaw_enter = 0.80f,
        /* dt为1 ms，0.02 m/s对应约20 m/s^2的轮加速度突变。 */
        .slip_dv_enter = 0.02f,
        .slip_enter_s = 0.05f,
        .off_force_ratio = 0.20f,
        .land_force_ratio = 0.35f,
        .off_hold_s = 0.03f,
        .land_hold_s = 0.05f,
        .off_F_comp_ratio = 0.28f,
        .turn_v_diff = 0.20f,
        .turn_force_limit_ratio = 0.50f,
        .stuck_T_ratio = 0.75f,
        .stuck_theta_enter = 0.50f,
        .stuck_theta_exit = 0.15f,
        .stuck_time = 0.05f,
        /* 补偿力约0.5 s爬到上限。 */
        .stuck_F0_coef_ratio = 0.38f,
        .stuck_F0_max_ratio = 0.19f,
        .stuck_L0_coef = 0.06f,
        .stuck_L0_max = 0.04f,
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
         * 误差限幅在进K点乘之前生效，防止位移积累或姿态瞬时越界时
         * 单一状态项主导四路输出。只限位置类，速度类留0表示不限幅。
         * 这里是输出封锁阶段的保守初值，实机站起来后按需要放宽。
         */
        .error_limit = {
            [CHASSIS_STATE_S] = 0.30f,        /* m */
            [CHASSIS_STATE_D_S] = 0.0f,
            [CHASSIS_STATE_FAI] = 0.50f,      /* rad */
            [CHASSIS_STATE_D_FAI] = 0.0f,
            [CHASSIS_STATE_THETA_L] = 0.30f,  /* rad */
            [CHASSIS_STATE_D_THETA_L] = 0.0f,
            [CHASSIS_STATE_THETA_R] = 0.30f,  /* rad */
            [CHASSIS_STATE_D_THETA_R] = 0.0f,
            [CHASSIS_STATE_THETA_B] = 0.20f,  /* rad */
            [CHASSIS_STATE_D_THETA_B] = 0.0f,
        },
        /*
         * 直接粘贴 MATLAB ABK_LQR.m 的 K_Fit_Coefficients 输出，整块替换本花括号内容。
         * 每行6个系数，顺序 p00、p10、p01、p20、p11、p02；两个输入依次为左腿长和右腿长，单位 m。
         * 40行按输出优先排列，每10行对应一路输出的十个状态，顺序必须与
         * CHASSIS_OUTPUT_* 和 CHASSIS_STATE_* 枚举一致：
         *   行 1~10 左轮力矩
         *   行11~20 右轮力矩
         *   行21~30 左腿摆力矩
         *   行31~40 右腿摆力矩
         * 状态顺序: s, d_s, fai, d_fai, theta_l, d_theta_l, theta_r, d_theta_r, theta_b, d_theta_b
         */
        .coefficients = {
            /* ---- 左轮力矩 ---- */
               -1.2118,    -2.9102,     3.6044,     7.8047,    -10.055,   -0.48894,  /* s */
               -7.8202,    -2.7233,     28.056,     37.496,    -86.151,    -6.7412,  /* d_s */
               -154.27,     94.937,    -39.494,    -135.45,     24.803,     59.311,  /* fai */
               -20.374,     22.265,    -14.852,     -24.24,     -2.758,     24.881,  /* d_fai */
               -26.118,    -97.667,     22.107,     140.46,    -20.104,    -34.288,  /* theta_l */
               -2.7408,    -8.0339,     9.3281,      9.564,    -13.891,    -8.4993,  /* d_theta_l */
               -16.153,     36.579,    -31.026,    -61.176,    -4.6762,     53.922,  /* theta_r */
               -2.3451,     5.7889,   -0.11228,    0.82895,    -19.852,       5.89,  /* d_theta_r */
               -154.38,     141.35,     39.585,    -111.96,      51.18,    -43.166,  /* theta_b */
               -2.4907,     2.8444,     2.2885,    -1.7187,    -2.8814,    -2.1441,  /* d_theta_b */
            /* ---- 右轮力矩 ---- */
               -1.2118,     3.6044,    -2.9102,   -0.48894,    -10.055,     7.8047,  /* s */
               -7.8202,     28.056,    -2.7233,    -6.7412,    -86.151,     37.496,  /* d_s */
                154.27,     39.494,    -94.937,    -59.311,    -24.803,     135.45,  /* fai */
                20.374,     14.852,    -22.265,    -24.881,      2.758,      24.24,  /* d_fai */
               -16.153,    -31.026,     36.579,     53.922,    -4.6762,    -61.176,  /* theta_l */
               -2.3451,   -0.11228,     5.7889,       5.89,    -19.852,    0.82895,  /* d_theta_l */
               -26.118,     22.107,    -97.667,    -34.288,    -20.104,     140.46,  /* theta_r */
               -2.7408,     9.3281,    -8.0339,    -8.4993,    -13.891,      9.564,  /* d_theta_r */
               -154.38,     39.585,     141.35,    -43.166,      51.18,    -111.96,  /* theta_b */
               -2.4907,     2.2885,     2.8444,    -2.1441,    -2.8814,    -1.7187,  /* d_theta_b */
            /* ---- 左腿摆力矩 ---- */
               0.27605,     3.5179,    -2.9356,    -6.5794,     1.2394,     3.4528,  /* s */
                1.9282,     13.345,    -17.194,    -31.829,     17.434,     18.884,  /* d_s */
               -9.8147,    -86.375,     3.1326,     132.02,    -41.156,     19.923,  /* fai */
               -1.6015,    -18.228,    0.23373,     23.826,    -8.5089,    0.44332,  /* d_fai */
                22.487,      1.283,    -2.2456,    0.61767,     10.133,    -2.8034,  /* theta_l */
                1.9937,     3.4972,    -2.2957,     -2.986,     3.5042,     2.9229,  /* d_theta_l */
               -2.4392,    -34.755,     20.296,     54.242,    -13.514,    -49.651,  /* theta_r */
              -0.12837,     -3.019,    -0.9756,    0.74909,     2.9617,    -2.8849,  /* d_theta_r */
               -53.478,    -48.242,     -27.18,     58.635,     1.1854,     18.293,  /* theta_b */
               -0.8281,    -1.6396,   -0.67206,     1.5161,     1.0988,    0.45671,  /* d_theta_b */
            /* ---- 右腿摆力矩 ---- */
               0.27605,    -2.9356,     3.5179,     3.4528,     1.2394,    -6.5794,  /* s */
                1.9282,    -17.194,     13.345,     18.884,     17.434,    -31.829,  /* d_s */
                9.8147,    -3.1326,     86.375,    -19.923,     41.156,    -132.02,  /* fai */
                1.6015,   -0.23373,     18.228,   -0.44332,     8.5089,    -23.826,  /* d_fai */
               -2.4392,     20.296,    -34.755,    -49.651,    -13.514,     54.242,  /* theta_l */
              -0.12837,    -0.9756,     -3.019,    -2.8849,     2.9617,    0.74909,  /* d_theta_l */
                22.487,    -2.2456,      1.283,    -2.8034,     10.133,    0.61767,  /* theta_r */
                1.9937,    -2.2957,     3.4972,     2.9229,     3.5042,     -2.986,  /* d_theta_r */
               -53.478,     -27.18,    -48.242,     18.293,     1.1854,     58.635,  /* theta_b */
               -0.8281,   -0.67206,    -1.6396,    0.45671,     1.0988,     1.5161,  /* d_theta_b */
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
        .wheel_flag = 1U,
        .joint_T_limit = 3.5f,
    },
    /* 整车公共目标、支撑力前馈和控制周期边界。 */
    .phi0_offset = CHASSIS_HALF_PI,
    .roll_target = 0.0f,
    /*
     * 重力前馈按 0.5*mass*gravity*cos(theta) 计算，随腿摆角投影。
     * 1.0表示按实测质量足额补偿；实机若发现腿长稳态偏差，先调这个系数，
     * 不要回头改 model.mass。
     */
    .F0_gravity_scale = 1.0f,
    .F0_left = 0.0f,
    .F0_right = 0.0f,
    .default_dt = APP_CTRL_DT_S,
    .dt_min = 0.0002f,
    .dt_max = 0.02f,
    .target = {0.0f},
};
