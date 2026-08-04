#include "device_motor_dm.h"

#include "fdcan.h"

#include <assert.h>
#include <math.h>
#include <string.h>

#define TEST_PI 3.14159265358979323846f
#define TEST_TOLERANCE 2.0e-2f

FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;

static uint32_t test_tick;
static uint32_t captured_identifier[MOTOR_DM_COUNT];
static uint8_t captured_data[MOTOR_DM_COUNT][APP_DM_FRAME_LEN];
static uint32_t captured_count;

uint32_t HAL_GetTick(void)
{
    return test_tick;
}

void CAN_Task_UpdateTxFrame(FDCAN_HandleTypeDef *handle,
                            uint32_t identifier,
                            const uint8_t *data,
                            uint8_t length)
{
    assert(handle == &hfdcan1);
    assert(length == APP_DM_FRAME_LEN);
    assert(captured_count < MOTOR_DM_COUNT);
    captured_identifier[captured_count] = identifier;
    memcpy(captured_data[captured_count], data, APP_DM_FRAME_LEN);
    captured_count++;
}

static uint16_t floatToUint(float value,
                              float minimum,
                              float maximum,
                              uint8_t bits)
{
    return (uint16_t)((value - minimum) *
                      (float)((1UL << bits) - 1UL) /
                      (maximum - minimum));
}

static void makeFeedback(uint8_t motor_id,
                          uint8_t state,
                          float position_rad,
                          float velocity_radps,
                          float torque_nm,
                          uint8_t data[APP_DM_FRAME_LEN])
{
    uint16_t position_raw = floatToUint(position_rad,
                                          APP_DM_PMIN,
                                          APP_DM_PMAX,
                                          16U);
    uint16_t velocity_raw = floatToUint(velocity_radps,
                                          APP_DM_VEL_MIN,
                                          APP_DM_VEL_MAX,
                                          12U);
    uint16_t torque_raw = floatToUint(torque_nm,
                                         APP_DM_TOR_MIN,
                                         APP_DM_TOR_MAX,
                                        12U);

    memset(data, 0, APP_DM_FRAME_LEN);
    data[0] = (uint8_t)((state << 4U) | (motor_id & 0x0FU));
    data[1] = (uint8_t)(position_raw >> 8U);
    data[2] = (uint8_t)position_raw;
    data[3] = (uint8_t)(velocity_raw >> 4U);
    data[4] = (uint8_t)(((velocity_raw & 0x0FU) << 4U) |
                        (torque_raw >> 8U));
    data[5] = (uint8_t)torque_raw;
    data[6] = 51U;
    data[7] = 52U;
}

static void assertNear(float actual, float expected)
{
    assert(fabsf(actual - expected) <= TEST_TOLERANCE);
}

static void testFeedbackRoutingAndRawMapping(void)
{
    const motor_dm_index_t indices[MOTOR_DM_COUNT] = {
        MOTOR_DM_LEFT_FRONT,
        MOTOR_DM_LEFT_BACK,
        MOTOR_DM_RIGHT_FRONT,
        MOTOR_DM_RIGHT_BACK,
    };
    const uint32_t feedback_ids[MOTOR_DM_COUNT] = {
        APP_DM_LF_FB,
        APP_DM_LB_FB,
        APP_DM_RF_FB,
        APP_DM_RB_FB,
    };
    uint8_t data[APP_DM_FRAME_LEN];
    uint32_t index;

    Motor_DM_Init();
    for (index = 0U; index < MOTOR_DM_COUNT; index++)
    {
        motor_dm_state_t state = {0};
        uint8_t motor_id = (uint8_t)(index + 1U);

        test_tick++;
        makeFeedback(motor_id, 1U, 1.0f, 3.0f, 2.0f, data);
        Motor_DM_UpdateFeedback(APP_CAN_BUS_FDCAN1,
                                feedback_ids[index],
                                data);
        Motor_DM_GetState(indices[index], &state);
        assert(state.feedbackCount == 1U);
        assert(state.state == 1U);
        assert(state.mosTemperature == 51U);
        assert(state.rotorTemperature == 52U);
        assertNear(state.positionWrappedRad, 1.0f);
        assertNear(state.positionRad, 1.0f);
        assertNear(state.velocityRadps, 3.0f);
        assertNear(state.torqueNm, 2.0f);
    }

    makeFeedback(1U, 1U, 0.0f, 0.0f, 0.0f, data);
    Motor_DM_UpdateFeedback(APP_CAN_BUS_FDCAN1,
                            APP_DM_RB_FB + 1U,
                            data);
    {
        motor_dm_state_t state = {0};
        Motor_DM_GetState(MOTOR_DM_LEFT_FRONT, &state);
        assert(state.feedbackCount == 1U);
    }

    Motor_DM_Init();
    makeFeedback(2U, 1U, 0.0f, 0.0f, 0.0f, data);
    Motor_DM_UpdateFeedback(APP_CAN_BUS_FDCAN1,
                            APP_DM_LF_FB,
                            data);
    for (index = 0U; index < MOTOR_DM_COUNT; index++)
    {
        motor_dm_state_t state = {0};
        Motor_DM_GetState((motor_dm_index_t)index, &state);
        assert(state.feedbackCount == 0U);
    }
}

static void testPositionAcrossPiAndTimeout(void)
{
    uint8_t data[APP_DM_FRAME_LEN];
    motor_dm_state_t state = {0};

    Motor_DM_Init();
    test_tick = 1U;
    makeFeedback(1U, 1U, TEST_PI - 0.02f, 0.0f, 0.0f, data);
    Motor_DM_UpdateFeedback(APP_CAN_BUS_FDCAN1,
                            APP_DM_LF_FB,
                            data);
    test_tick++;
    makeFeedback(1U, 1U, TEST_PI + 0.03f, 0.0f, 0.0f, data);
    Motor_DM_UpdateFeedback(APP_CAN_BUS_FDCAN1,
                            APP_DM_LF_FB,
                            data);
    Motor_DM_GetState(MOTOR_DM_LEFT_FRONT, &state);
    assertNear(state.positionRad, TEST_PI + 0.03f);

    test_tick += APP_DM_TIMEOUT_TICKS + 1U;
    makeFeedback(1U, 1U, -0.25f, 0.0f, 0.0f, data);
    Motor_DM_UpdateFeedback(APP_CAN_BUS_FDCAN1,
                            APP_DM_LF_FB,
                            data);
    Motor_DM_GetState(MOTOR_DM_LEFT_FRONT, &state);
    assertNear(state.positionRad, -0.25f);

    Motor_DM_Init();
    test_tick = 1U;
    makeFeedback(1U, 1U, -TEST_PI + 0.02f, 0.0f, 0.0f, data);
    Motor_DM_UpdateFeedback(APP_CAN_BUS_FDCAN1,
                            APP_DM_LF_FB,
                            data);
    test_tick++;
    makeFeedback(1U, 1U, -TEST_PI - 0.03f, 0.0f, 0.0f, data);
    Motor_DM_UpdateFeedback(APP_CAN_BUS_FDCAN1,
                            APP_DM_LF_FB,
                            data);
    Motor_DM_GetState(MOTOR_DM_LEFT_FRONT, &state);
    assertNear(state.positionRad, -TEST_PI - 0.03f);
}

static void testTorqueOnlyTransmit(void)
{
    motor_dm_command_t positive_command = {.torqueNm = 1.0f};
    motor_dm_command_t negative_command = {.torqueNm = -1.0f};
    uint16_t positive_raw = floatToUint(1.0f,
                                        APP_DM_TOR_MIN,
                                        APP_DM_TOR_MAX,
                                        12U);
    uint16_t negative_raw = floatToUint(-1.0f,
                                        APP_DM_TOR_MIN,
                                        APP_DM_TOR_MAX,
                                        12U);
    uint8_t positive_found = 0U;
    uint8_t negative_found = 0U;
    uint32_t index;

    Motor_DM_Init();
    Motor_DM_SetSafe(0U);
    Motor_DM_SetCommand(MOTOR_DM_LEFT_BACK, &positive_command);
    Motor_DM_SetCommand(MOTOR_DM_RIGHT_FRONT, &negative_command);
    captured_count = 0U;
    Motor_DM_UpdateTxFrames();
    assert(captured_count == MOTOR_DM_COUNT);

    for (index = 0U; index < captured_count; index++)
    {
        uint32_t byte_index;

        uint16_t expected_raw;

        if (captured_identifier[index] == APP_DM_LB_ID)
        {
            expected_raw = positive_raw;
            positive_found = 1U;
        }
        else if (captured_identifier[index] == APP_DM_RF_ID)
        {
            expected_raw = negative_raw;
            negative_found = 1U;
        }
        else
        {
            expected_raw = floatToUint(0.0f,
                                       APP_DM_TOR_MIN,
                                       APP_DM_TOR_MAX,
                                       12U);
        }
        for (byte_index = 0U; byte_index < 6U; byte_index++)
        {
            assert(captured_data[index][byte_index] == 0U);
        }
        assert(captured_data[index][6] == (uint8_t)(expected_raw >> 8U));
        assert(captured_data[index][7] == (uint8_t)expected_raw);
    }
    assert(positive_found != 0U);
    assert(negative_found != 0U);
}

static void testEnableThenSafeCommandTransmit(void)
{
    motor_dm_command_t command = {.torqueNm = 1.0f};
    uint16_t zero_torque_raw = floatToUint(0.0f,
                                           APP_DM_TOR_MIN,
                                           APP_DM_TOR_MAX,
                                           12U);
    uint32_t index;

    Motor_DM_Init();
    Motor_DM_SetSafe(1U);
    Motor_DM_SetCommand(MOTOR_DM_LEFT_BACK, &command);
    Motor_DM_SetEnable(1U);

    captured_count = 0U;
    Motor_DM_UpdateTxFrames();
    assert(captured_count == MOTOR_DM_COUNT);
    for (index = 0U; index < captured_count; index++)
    {
        uint32_t byte_index;

        for (byte_index = 0U; byte_index < 7U; byte_index++)
        {
            assert(captured_data[index][byte_index] == 0xFFU);
        }
        assert(captured_data[index][7] == 0xFCU);
    }

    captured_count = 0U;
    Motor_DM_UpdateTxFrames();
    assert(captured_count == MOTOR_DM_COUNT);
    for (index = 0U; index < captured_count; index++)
    {
        uint32_t byte_index;

        for (byte_index = 0U; byte_index < 6U; byte_index++)
        {
            assert(captured_data[index][byte_index] == 0U);
        }
        assert(captured_data[index][6] ==
               (uint8_t)(zero_torque_raw >> 8U));
        assert(captured_data[index][7] == (uint8_t)zero_torque_raw);
    }
}

static void testEnableRetryOnlyForDisabledMotor(void)
{
    uint8_t data[APP_DM_FRAME_LEN];
    uint32_t index;

    Motor_DM_Init();
    Motor_DM_SetSafe(1U);
    test_tick = 100U;
    Motor_DM_SetEnable(1U);

    captured_count = 0U;
    Motor_DM_UpdateTxFrames();
    assert(captured_count == MOTOR_DM_COUNT);

    makeFeedback(1U, 1U, 0.0f, 0.0f, 0.0f, data);
    Motor_DM_UpdateFeedback(APP_CAN_BUS_FDCAN1, APP_DM_LF_FB, data);
    makeFeedback(2U, 0U, 0.0f, 0.0f, 0.0f, data);
    Motor_DM_UpdateFeedback(APP_CAN_BUS_FDCAN1, APP_DM_LB_FB, data);
    makeFeedback(3U, 8U, 0.0f, 0.0f, 0.0f, data);
    Motor_DM_UpdateFeedback(APP_CAN_BUS_FDCAN1, APP_DM_RF_FB, data);
    makeFeedback(4U, 1U, 0.0f, 0.0f, 0.0f, data);
    Motor_DM_UpdateFeedback(APP_CAN_BUS_FDCAN1, APP_DM_RB_FB, data);

    test_tick += APP_DM_EN_RETRY;
    captured_count = 0U;
    Motor_DM_UpdateTxFrames();
    assert(captured_count == MOTOR_DM_COUNT);

    for (index = 0U; index < captured_count; index++)
    {
        if (captured_identifier[index] == APP_DM_LB_ID)
        {
            uint32_t byte_index;

            for (byte_index = 0U; byte_index < 7U; byte_index++)
            {
                assert(captured_data[index][byte_index] == 0xFFU);
            }
            assert(captured_data[index][7] == 0xFCU);
        }
        else
        {
            assert(captured_data[index][7] != 0xFCU);
            assert(captured_data[index][7] != 0xFEU);
        }
    }
}

int main(void)
{
    testFeedbackRoutingAndRawMapping();
    testPositionAcrossPiAndTimeout();
    testTorqueOnlyTransmit();
    testEnableThenSafeCommandTransmit();
    testEnableRetryOnlyForDisabledMotor();
    return 0;
}
