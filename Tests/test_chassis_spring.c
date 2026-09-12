#include "chassis_vmc.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

#define TEST_TOLERANCE 2.0e-5f
#define TEST_DERIVATIVE_TOLERANCE 2.0e-3f

/* 数值求导步长，m。取值要远大于五连杆逆解的往返误差，又远小于腿长量程。 */
#define TEST_DIFF_STEP 5.0e-4f

/* 本文件自带的一组气弹簧尺寸，只用于验算公式，与实机配置无关。 */
static const Chassis_Spring_Config_t test_spring = {
    .enable_flag = 1U,
    .force = 180.0f,
    .r1 = 0.180f,
    .r2 = 0.050f,
    .mount_offset = 0.0f,
    .free_length = 0.0f,
    .stroke = 0.0f,
};

static float motor_position(const Chassis_Leg_Config_t *config,
                            Chassis_Joint_t joint,
                            float phi)
{
    return (phi - config->joint[joint].angle_offset_rad) /
           (config->joint[joint].scale *
            config->joint[joint].ratio);
}

static void calculate_target_pose(const Chassis_Leg_Config_t *config,
                                  float target_L0,
                                  float target_phi0,
                                  Chassis_Leg_t *leg)
{
    Chassis_Leg_t current_leg = {
        .phi1 = 2.7f,
        .phi4 = 0.4f,
    };
    VMC_Joint_Target_t target;

    assert(VMC_Inverse_Calc(config,
                            &current_leg,
                            target_L0,
                            target_phi0,
                            &target) == 1U);
    VMC_State_Calc(config,
                   motor_position(config, CHASSIS_JOINT_PHI1, target.phi1),
                   motor_position(config, CHASSIS_JOINT_PHI4, target.phi4),
                   0.0f,
                   0.0f,
                   leg);
    assert(leg->valid_flag == 1U);
}

/*
 * 由五连杆的笛卡尔构型直接量出气弹簧两铰点中心距，作为独立参照。
 *
 * 这一份【不走余弦定理】：B 和 C 按 VMC_State_Calc 自己的构造方式
 * （B = 铰点 + l1*(cos phi1, sin phi1)，C = B + l2*(cos phi2, sin phi2)）
 * 重建，膝角由向量叉乘/点乘取出。它验的正是"闭式里的 alpha 确实等于真实
 * 机构的膝角"，照抄公式再算一遍是验不出这一点的。
 *
 * mount_offset 整体放在 P 一侧：P 从 B->O 方向朝 B->C 方向转过该角度，
 * 于是三角形 P-B-Q 的夹角就是 alpha - mount_offset，与闭式口径一致。
 */
static float spring_length_cartesian(const Chassis_Leg_Config_t *config,
                                     const Chassis_Spring_Config_t *spring,
                                     const Chassis_Leg_t *leg)
{
    float pivot_x = -config->geometry.l5 * 0.5f;
    float b_x = pivot_x + config->geometry.l1 * cosf(leg->phi1);
    float b_y = config->geometry.l1 * sinf(leg->phi1);
    float c_x = b_x + config->geometry.l2 * cosf(leg->phi2);
    float c_y = b_y + config->geometry.l2 * sinf(leg->phi2);
    /* O 是两个主动杆输出铰点的中点，也是虚拟腿的原点。 */
    float bo_x = -b_x;
    float bo_y = -b_y;
    float bc_x = c_x - b_x;
    float bc_y = c_y - b_y;
    float signed_alpha = atan2f((bo_x * bc_y) - (bo_y * bc_x),
                                (bo_x * bc_x) + (bo_y * bc_y));
    float turn = (signed_alpha >= 0.0f) ? spring->mount_offset
                                        : -spring->mount_offset;
    float ang_bp = atan2f(bo_y, bo_x) + turn;
    float ang_bq = atan2f(bc_y, bc_x);
    float p_x = b_x + spring->r1 * cosf(ang_bp);
    float p_y = b_y + spring->r1 * sinf(ang_bp);
    float q_x = b_x + spring->r2 * cosf(ang_bq);
    float q_y = b_y + spring->r2 * sinf(ang_bq);

    return sqrtf(((p_x - q_x) * (p_x - q_x)) +
                 ((p_y - q_y) * (p_y - q_y)));
}

/* 开关关闭或腿状态无解时恒为零，整条补偿链路等价于不存在。 */
static void test_disabled(const Chassis_Leg_Config_t *config)
{
    Chassis_Spring_Config_t spring = test_spring;
    Chassis_Leg_t leg;

    calculate_target_pose(config, 0.20f, CHASSIS_HALF_PI, &leg);

    spring.enable_flag = 0U;
    assert(VMC_Spring_Force_Calc(&spring, config, &leg) == 0.0f);

    spring.enable_flag = 1U;
    assert(VMC_Spring_Force_Calc(&spring, config, &leg) != 0.0f);

    leg.valid_flag = 0U;
    assert(VMC_Spring_Force_Calc(&spring, config, &leg) == 0.0f);
}

/*
 * 虚功核对：F0_spring/force 必须等于 ds/dL0，而 s 由笛卡尔构型独立量出。
 * 同时跑 mount_offset 为零和非零两组，确认偏置的符号口径没有反。
 */
static void test_virtual_work(const Chassis_Leg_Config_t *config)
{
    static const float lengths[] = {0.12f, 0.15f, 0.20f, 0.25f, 0.30f, 0.34f};
    static const float offsets[] = {0.0f, 0.2443f, -0.15f};
    Chassis_Spring_Config_t spring = test_spring;
    Chassis_Leg_t leg_center;
    Chassis_Leg_t leg_plus;
    Chassis_Leg_t leg_minus;
    uint32_t length_index;
    uint32_t offset_index;

    for (offset_index = 0U;
         offset_index < sizeof(offsets) / sizeof(offsets[0]);
         offset_index++)
    {
        spring.mount_offset = offsets[offset_index];
        for (length_index = 0U;
             length_index < sizeof(lengths) / sizeof(lengths[0]);
             length_index++)
        {
            float L0 = lengths[length_index];
            float analytic;
            float numeric;

            calculate_target_pose(config, L0, CHASSIS_HALF_PI, &leg_center);
            calculate_target_pose(config, L0 + TEST_DIFF_STEP,
                                  CHASSIS_HALF_PI, &leg_plus);
            calculate_target_pose(config, L0 - TEST_DIFF_STEP,
                                  CHASSIS_HALF_PI, &leg_minus);

            analytic = VMC_Spring_Force_Calc(&spring, config, &leg_center) /
                       spring.force;
            /* 用逆解实际落到的腿长做差商，不用请求值，免得吃到往返误差。 */
            numeric =
                (spring_length_cartesian(config, &spring, &leg_plus) -
                 spring_length_cartesian(config, &spring, &leg_minus)) /
                (leg_plus.L0 - leg_minus.L0);

            assert(fabsf(analytic - numeric) < TEST_DERIVATIVE_TOLERANCE);
        }
    }
}

/*
 * 气弹簧没有 Tp 分量：固定腿长扫腿摆角，机构上量出的气弹簧长度必须不变。
 *
 * 这条结论是"扣除只加在 F0、Tp 一律不动"的全部依据，而它依赖 l5 = 0
 * （两根主动杆共轴，三角形 O-B-C 的三边就是 l1、l2、L0）。所以这里
 * 顺带把 l5 改成非零，确认那时长度【确实会】随腿摆角变——换机构必须重新推导。
 */
static void test_no_swing_component(const Chassis_Leg_Config_t *base_config)
{
    static const float swings[] = {1.25f, CHASSIS_HALF_PI, 1.90f};
    Chassis_Leg_Config_t config = *base_config;
    Chassis_Leg_t leg;
    float reference_s;
    float reference_force;
    uint32_t index;

    assert(config.geometry.l5 == 0.0f);

    calculate_target_pose(&config, 0.25f, swings[0], &leg);
    reference_s = spring_length_cartesian(&config, &test_spring, &leg);
    reference_force = VMC_Spring_Force_Calc(&test_spring, &config, &leg);

    for (index = 1U; index < sizeof(swings) / sizeof(swings[0]); index++)
    {
        calculate_target_pose(&config, 0.25f, swings[index], &leg);
        assert(fabsf(spring_length_cartesian(&config, &test_spring, &leg) -
                     reference_s) < TEST_TOLERANCE);
        assert(fabsf(VMC_Spring_Force_Calc(&test_spring, &config, &leg) -
                     reference_force) < TEST_TOLERANCE);
    }

    /* l5 不为零时 O-B-C 不再是那个三角形，闭式失效，这里确认它真的会变。 */
    config.geometry.l5 = 0.06f;
    calculate_target_pose(&config, 0.25f, swings[0], &leg);
    reference_s = spring_length_cartesian(&config, &test_spring, &leg);
    calculate_target_pose(&config, 0.25f, swings[2], &leg);
    assert(fabsf(spring_length_cartesian(&config, &test_spring, &leg) -
                 reference_s) > TEST_TOLERANCE);
}

/*
 * 方向与单调性：正的 force 必须给出正的 F0（伸腿撑地方向），
 * 且杠杆比随腿长单调增——这正是"气弹簧只在某一个腿长上刚好托平"的原因。
 */
static void test_sign_and_monotonic(const Chassis_Leg_Config_t *config)
{
    Chassis_Spring_Config_t spring = test_spring;
    Chassis_Leg_t leg;
    float last_force = -1.0f;
    float L0;

    for (L0 = 0.12f; L0 < 0.341f; L0 += 0.02f)
    {
        float force;

        calculate_target_pose(config, L0, CHASSIS_HALF_PI, &leg);
        force = VMC_Spring_Force_Calc(&spring, config, &leg);
        assert(force > 0.0f);
        assert(force > last_force);
        last_force = force;
    }

    /* force 取负表示装配方向与推导相反，整条曲线跟着翻号。 */
    spring.force = -test_spring.force;
    calculate_target_pose(config, 0.20f, CHASSIS_HALF_PI, &leg);
    assert(VMC_Spring_Force_Calc(&spring, config, &leg) < 0.0f);
}

/*
 * 实机配置的行程校核：s(L0) 在整个腿长区间内必须落在
 * [free_length - stroke, free_length] 之内。顶到行程两端是硬限位，
 * 气弹簧模型在那里完全失效，必须在上车前排除。
 *
 * 尺寸尚未录入(free_length 为 0)时跳过校核，改为打印实测要用的对照表。
 */
static void test_shipped_config(void)
{
    const Chassis_Spring_Config_t *spring = &Chassis_Config.spring;
    const Chassis_Leg_Config_t *config = &Chassis_Config.leg[CHASSIS_LEFT];
    Chassis_Leg_t leg;
    float L0;

    if (spring->free_length <= 0.0f)
    {
        printf("  气弹簧尺寸未录入(free_length=0)，跳过行程校核。\n");
        printf("  录入 force/r1/r2/mount_offset 后本用例会自动开始校核。\n");
        return;
    }

    for (L0 = Chassis_Config.lqr.L0_min;
         L0 < (Chassis_Config.lqr.L0_max + TEST_TOLERANCE);
         L0 += 0.01f)
    {
        float s;

        calculate_target_pose(config, L0, CHASSIS_HALF_PI, &leg);
        s = spring_length_cartesian(config, spring, &leg);
        assert(s <= spring->free_length);
        assert(s >= (spring->free_length - spring->stroke));
    }
}

int main(void)
{
    const Chassis_Leg_Config_t *config = &Chassis_Config.leg[CHASSIS_LEFT];

    test_disabled(config);
    test_virtual_work(config);
    test_no_swing_component(config);
    test_sign_and_monotonic(config);
    test_shipped_config();

    printf("test_chassis_spring PASS\n");
    return 0;
}
