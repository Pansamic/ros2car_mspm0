#include <stdint.h>
#include "ti_msp_dl_config.h"
#include <tb6612.h>

tb6612_cb_t motor_cb[4] = {
    {
        .reverse = 0,
        .ctrl1_gpio_port = GPIO_GRP_MCTRL_PIN_MOTOR1_1_PORT,
        .ctrl1_gpio_pin = GPIO_GRP_MCTRL_PIN_MOTOR1_1_PIN,
        .ctrl2_gpio_port = GPIO_GRP_MCTRL_PIN_MOTOR1_2_PORT,
        .ctrl2_gpio_pin = GPIO_GRP_MCTRL_PIN_MOTOR1_2_PIN,
        .pwm_timer = PWM_MOTOR_12_INST,
        .channel = GPIO_PWM_MOTOR_12_C0_IDX,
        .encoder_timer = TIMER_ENCODER_INST,
        .pwm_peroid_count = 1000,
        .turndown_ratio = 30,
        .max_velocity = 6*2*3.1415926,
        .kp = 100,
        .kd = 8,
        .encoder_round_count = 13*4, // Hall sensor encoder has 13 pulses per round, and quadruplicated frequency.
        .timer_frequency = 8000000,
    },
    {
        .reverse = 0,
        .ctrl1_gpio_port = GPIO_GRP_MCTRL_PIN_MOTOR2_1_PORT,
        .ctrl1_gpio_pin = GPIO_GRP_MCTRL_PIN_MOTOR2_1_PIN,
        .ctrl2_gpio_port = GPIO_GRP_MCTRL_PIN_MOTOR2_2_PORT,
        .ctrl2_gpio_pin = GPIO_GRP_MCTRL_PIN_MOTOR2_2_PIN,
        .pwm_timer = PWM_MOTOR_12_INST,
        .channel = GPIO_PWM_MOTOR_12_C1_IDX,
        .encoder_timer = TIMER_ENCODER_INST,
        .pwm_peroid_count = 1000,
        .turndown_ratio = 30,
        .max_velocity = 6*2*3.1415926,
        .kp = 100,
        .kd = 8,
        .encoder_round_count = 13*4, // Hall sensor encoder has 13 pulses per round, and quadruplicated frequency.
        .timer_frequency = 8000000,
    },
    {
        .reverse = 0,
        .ctrl1_gpio_port = GPIO_GRP_MCTRL_PIN_MOTOR3_1_PORT,
        .ctrl1_gpio_pin = GPIO_GRP_MCTRL_PIN_MOTOR3_1_PIN,
        .ctrl2_gpio_port = GPIO_GRP_MCTRL_PIN_MOTOR3_2_PORT,
        .ctrl2_gpio_pin = GPIO_GRP_MCTRL_PIN_MOTOR3_2_PIN,
        .pwm_timer = PWM_MOTOR_34_INST,
        .channel = GPIO_PWM_MOTOR_34_C0_IDX,
        .encoder_timer = TIMER_ENCODER_INST,
        .pwm_peroid_count = 1000,
        .turndown_ratio = 30,
        .max_velocity = 6*2*3.1415926,
        .kp = 100,
        .kd = 8,
        .encoder_round_count = 13*4, // Hall sensor encoder has 13 pulses per round, and quadruplicated frequency.
        .timer_frequency = 8000000,
    },
    {
        .reverse = 0,
        .ctrl1_gpio_port = GPIO_GRP_MCTRL_PIN_MOTOR4_1_PORT,
        .ctrl1_gpio_pin = GPIO_GRP_MCTRL_PIN_MOTOR4_1_PIN,
        .ctrl2_gpio_port = GPIO_GRP_MCTRL_PIN_MOTOR4_2_PORT,
        .ctrl2_gpio_pin = GPIO_GRP_MCTRL_PIN_MOTOR4_2_PIN,
        .pwm_timer = PWM_MOTOR_34_INST,
        .channel = GPIO_PWM_MOTOR_34_C1_IDX,
        .encoder_timer = TIMER_ENCODER_INST,
        .pwm_peroid_count = 1000,
        .turndown_ratio = 30,
        .max_velocity = 6*2*3.1415926,
        .kp = 100,
        .kd = 8,
        .encoder_round_count = 13*4, // Hall sensor encoder has 13 pulses per round, and quadruplicated frequency.
        .timer_frequency = 8000000,
    },
};

/**
 * @brief Set the velocity of the motor connected to channel 1 of the TB6612.
 *
 * @param velocity unit: rad/s
 */
static void tb6612_set_velocity(tb6612_cb_t* pcb, float velocity){
    // Set the velocity of the motor connected to channel 1 of the TB6612
    pcb->target_velocity = velocity;
}

static void tb6612_brake(tb6612_cb_t* pcb){
    DL_GPIO_setPins(pcb->ctrl1_gpio_port, pcb->ctrl1_gpio_pin);
    DL_GPIO_setPins(pcb->ctrl2_gpio_port, pcb->ctrl2_gpio_pin);
    DL_Timer_setCaptureCompareValue(pcb->pwm_timer, 0, pcb->channel);
}

static void tb6612_update_control(tb6612_cb_t* pcb){
    uint32_t feed_forward;
    pcb->current_position = (float)pcb->encoder_count / (pcb->encoder_round_count * pcb->turndown_ratio) * 2 * 3.14159265f;
    pcb->current_velocity = 2 * 3.14159265f * 
        (float)pcb->timer_frequency * 
        (float)(pcb->encoder_count - pcb->previous_encoder_count) /
        (float)(pcb->encoder_round_count*pcb->turndown_ratio) /
        (float)(DL_Timer_getTimerCount(pcb->encoder_timer) - pcb->timer_count);
    pcb->previous_encoder_count = pcb->encoder_count;
    pcb->timer_count = DL_Timer_getTimerCount(pcb->encoder_timer);

    feed_forward = (uint32_t)((float)pcb->pwm_peroid_count * pcb->target_velocity / pcb->max_velocity);
    pcb->error = pcb->target_velocity - pcb->current_velocity;
    pcb->control = (uint32_t)(pcb->kp * pcb->error + pcb->kd * (pcb->error - pcb->previous_error));
    pcb->previous_error = pcb->error;
    if((pcb->target_velocity * pcb->current_velocity < 0) && (pcb->current_velocity > 0.1 || pcb->current_velocity < -0.1)){
        tb6612_brake(pcb);
    } else {
        if((pcb->control < 0 && !pcb->reverse) || (pcb->control > 0 && pcb->reverse)){
            DL_GPIO_setPins(pcb->ctrl1_gpio_port, pcb->ctrl1_gpio_pin);
            DL_GPIO_clearPins(pcb->ctrl2_gpio_port, pcb->ctrl2_gpio_pin);
        } else if((pcb->control > 0 && !pcb->reverse) || (pcb->control < 0 && pcb->reverse)){
            DL_GPIO_clearPins(pcb->ctrl1_gpio_port, pcb->ctrl1_gpio_pin);
            DL_GPIO_setPins(pcb->ctrl2_gpio_port, pcb->ctrl2_gpio_pin);
        }
    }
    DL_Timer_setCaptureCompareValue(pcb->pwm_timer, (uint32_t)(pcb->control) + feed_forward, pcb->channel);
}

void tb6612_set_velocity_motor1(float velocity){
    tb6612_set_velocity(&motor_cb[0], velocity);
}

void tb6612_set_velocity_motor2(float velocity){
    tb6612_set_velocity(&motor_cb[1], velocity);
}

void tb6612_set_velocity_motor3(float velocity){
    tb6612_set_velocity(&motor_cb[2], velocity);
}

void tb6612_set_velocity_motor4(float velocity){
    tb6612_set_velocity(&motor_cb[3], velocity);
}

void tb6612_update_control_motor1(void){
    tb6612_update_control(&motor_cb[0]);
}

void tb6612_update_control_motor2(void){
    tb6612_update_control(&motor_cb[1]);
}

void tb6612_update_control_motor3(void){
    tb6612_update_control(&motor_cb[2]);
}

void tb6612_update_control_motor4(void){
    tb6612_update_control(&motor_cb[3]);
}

float tb6612_get_velocity_motor1(void){
    return motor_cb[0].current_velocity;
}

float tb6612_get_velocity_motor2(void){
    return motor_cb[1].current_velocity;
}

float tb6612_get_velocity_motor3(void){
    return motor_cb[2].current_velocity;
}

float tb6612_get_velocity_motor4(void){
    return motor_cb[3].current_velocity;
}

float tb6612_get_position_motor1(void){
    return motor_cb[0].current_position;
}

float tb6612_get_position_motor2(void){
    return motor_cb[1].current_position;
}

float tb6612_get_position_motor3(void){
    return motor_cb[2].current_position;
}

float tb6612_get_position_motor4(void){
    return motor_cb[3].current_position;
}
