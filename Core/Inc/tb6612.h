#ifndef __TB6612_H__
#define __TB6612_H__

typedef struct tb6612_cb_t{
    uint8_t reverse;
    GPIO_Regs *ctrl1_gpio_port;
    uint32_t ctrl1_gpio_pin;
    GPIO_Regs *ctrl2_gpio_port;
    uint32_t ctrl2_gpio_pin;
    GPTIMER_Regs *pwm_timer;
    DL_TIMER_CC_INDEX channel;
    GPTIMER_Regs *encoder_timer;
    uint32_t pwm_peroid_count;
    uint16_t turndown_ratio;
    float max_velocity;
    float kp;
    float kd;
    float previous_error;
    float error;
    float control;
    volatile float target_velocity;
    volatile float current_velocity;
    volatile float target_position;
    volatile float current_position;
    volatile int32_t encoder_count;
    int32_t previous_encoder_count;
    uint32_t encoder_round_count;
    uint32_t timer_frequency;
    int32_t timer_count;
}tb6612_cb_t;

void tb6612_set_velocity_motor1(float velocity);
void tb6612_set_velocity_motor2(float velocity);
void tb6612_set_velocity_motor3(float velocity);
void tb6612_set_velocity_motor4(float velocity);

void tb6612_update_control_motor1(void);
void tb6612_update_control_motor2(void);
void tb6612_update_control_motor3(void);
void tb6612_update_control_motor4(void);

float tb6612_get_velocity_motor1(void);
float tb6612_get_velocity_motor2(void);
float tb6612_get_velocity_motor3(void);
float tb6612_get_velocity_motor4(void);

float tb6612_get_position_motor1(void);
float tb6612_get_position_motor2(void);
float tb6612_get_position_motor3(void);
float tb6612_get_position_motor4(void);

#endif