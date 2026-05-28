#include "radar_logic.h"
#include "stdio.h"
#include "st7735.h"

#define RADAR_STABLE_THRESHOLD  3
#define RADAR_DISTANCE_LIMIT_CM 80

#define RADAR_HUART_RIGHT (huart4)
#define RADAR_HUART_LEFT (huart8)

RadarDriver_t radar_right;
RadarDriver_t radar_left;
RadarLogic_t radar_logic;

void RadarLogic_Init(void)
{
    RadarDriver_Init(&radar_right, &RADAR_HUART_RIGHT);
    RadarDriver_Init(&radar_left, &RADAR_HUART_LEFT);

    RadarDriver_StartIT(&radar_right);
    RadarDriver_StartIT(&radar_left);

    radar_logic.left_candidate = 0;
    radar_logic.left_stable_count = 0;
    radar_logic.left_confirmed = 0;

    radar_logic.right_candidate = 0;
    radar_logic.right_stable_count = 0;
    radar_logic.right_confirmed = 0;
}



static void RadarLogic_UpdateSide(uint8_t current_detected,
                                  uint8_t *candidate,
                                  uint8_t *stable_count,
                                  uint8_t *confirmed)
{
    if (current_detected == *candidate)
    {
        if (*stable_count < 255)
            (*stable_count)++;
    }
    else
    {
        *candidate = current_detected;
        *stable_count = 1;
    }

    if (*stable_count >= RADAR_STABLE_THRESHOLD)
    {
        *confirmed = *candidate;
    }
}

static uint8_t RadarLogic_HasTarget(RadarDriver_t *radar)
{
    return ((radar->target_state == 2 || radar->target_state == 3) &&
            radar->distance_cm < RADAR_DISTANCE_LIMIT_CM);
}

RadarDecision RadarLogic_Update(RadarLogic_t *logic,
                                RadarDriver_t *left,
                                RadarDriver_t *right)
{
    uint8_t left_detected = 0;
    uint8_t right_detected = 0;

    if (RadarDriver_IsValid(left, 300) && left->frame_ok)
    {
        if (RadarLogic_HasTarget(left))
            left_detected = 1;
    }

    if (RadarDriver_IsValid(right, 300) && right->frame_ok)
    {
        if (RadarLogic_HasTarget(right))
            right_detected = 1;
    }

    // if (RadarDriver_IsValid(left, 300) && left->frame_ok)
    // {
    //     if (left->target_state != 0 && left->distance_cm < RADAR_DISTANCE_LIMIT_CM)
    //         left_detected = 1;
    // }

    // if (RadarDriver_IsValid(right, 300) && right->frame_ok)
    // {
    //     if (right->target_state != 0 && right->distance_cm < RADAR_DISTANCE_LIMIT_CM)
    //         right_detected = 1;
    // }

    RadarLogic_UpdateSide(left_detected,
                          &logic->left_candidate,
                          &logic->left_stable_count,
                          &logic->left_confirmed);

    RadarLogic_UpdateSide(right_detected,
                          &logic->right_candidate,
                          &logic->right_stable_count,
                          &logic->right_confirmed);

    if (logic->left_confirmed && logic->right_confirmed)
        return RADAR_BOTH;
    else if (logic->left_confirmed)
        return RADAR_LEFT;
    else if (logic->right_confirmed)
        return RADAR_RIGHT;
    else
        return RADAR_NONE;
}

Direction_t Radar_GetTurnDirection(RadarDecision decision)
{
    switch (decision)
    {
        case RADAR_LEFT:
            return Direction_RIGHT;

        case RADAR_RIGHT:
            return Direction_LEFT;

        case RADAR_BOTH:
            return Direction_FORWARD;

        case RADAR_NONE:
        default:
            return Direction_NORMAL;
    }
}


RadarDecision Radar_UpdateAndGetDecision(void)
{
    return RadarLogic_Update(&radar_logic, &radar_left, &radar_right);
}




void Debug_PrintDecision(RadarDecision d)
{
    uint8_t left_ot2 = (HAL_GPIO_ReadPin(RADAR_LEFT_OT2_GPIO_Port, RADAR_LEFT_OT2_Pin) == GPIO_PIN_SET) ? 1 : 0;
    uint8_t right_ot2 = (HAL_GPIO_ReadPin(RADAR_RIGHT_OT2_GPIO_Port, RADAR_RIGHT_OT2_Pin) == GPIO_PIN_SET) ? 1 : 0;

    uint8_t obstacle_locked = d;
    RadarDecision locked_side = !d;

    char msg[220];

    if (d == RADAR_LEFT)
    {
        sprintf(msg,
                "LEFT  Lg=%d Ls=%d Ld=%d Lc=%d | Rg=%d Rs=%d Rd=%d Rc=%d | lock=%d side=%d\r\n",
                left_ot2, radar_left.target_state, radar_left.distance_cm, radar_logic.left_confirmed,
                right_ot2, radar_right.target_state, radar_right.distance_cm, radar_logic.right_confirmed,
                obstacle_locked, locked_side);
    }
    else if (d == RADAR_RIGHT)
    {
        sprintf(msg,
                "RIGHT Lg=%d Ls=%d Ld=%d Lc=%d | Rg=%d Rs=%d Rd=%d Rc=%d | lock=%d side=%d\r\n",
                left_ot2, radar_left.target_state, radar_left.distance_cm, radar_logic.left_confirmed,
                right_ot2, radar_right.target_state, radar_right.distance_cm, radar_logic.right_confirmed,
                obstacle_locked, locked_side);
    }
    else if (d == RADAR_BOTH)
    {
        sprintf(msg,
                "BOTH  Lg=%d Ls=%d Ld=%d Lc=%d | Rg=%d Rs=%d Rd=%d Rc=%d | lock=%d side=%d\r\n",
                left_ot2, radar_left.target_state, radar_left.distance_cm, radar_logic.left_confirmed,
                right_ot2, radar_right.target_state, radar_right.distance_cm, radar_logic.right_confirmed,
                obstacle_locked, locked_side);
    }
    else
    {
        sprintf(msg,
                "NONE  Lg=%d Ls=%d Ld=%d Lc=%d | Rg=%d Rs=%d Rd=%d Rc=%d | lock=%d side=%d\r\n",
                left_ot2, radar_left.target_state, radar_left.distance_cm, radar_logic.left_confirmed,
                right_ot2, radar_right.target_state, radar_right.distance_cm, radar_logic.right_confirmed,
                obstacle_locked, locked_side);
    }

    ST7735_WriteString(1, 5, msg, ST7735_WHITE, ST7735_BLACK, 1);
    printf("%s", msg);
}