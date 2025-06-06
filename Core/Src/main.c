#include <string.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "ti_msp_dl_config.h"
#include "tb6612.h"
#include "uart.h"
#include "oled.h"

#define DEBOUNCE_TIME_MS 20 // 消抖时间
#define LONG_PRESS_TIME_MS 2000 // 长按时间阈值

typedef enum button_event_t{
    BUTTON_EVENT_NONE = 0,
    BUTTON_EVENT_SHORT_PRESS,
    BUTTON_EVENT_LONG_PRESS,
    BUTTON_EVENT_DOUBLE_CLICK,
    BUTTON_EVENT_DOUBLE_LONG_PRESS
} button_event_t;

typedef enum buzzer_event_t{
    BUZZER_EVENT_NOTICE = 0,
    BUZZER_EVENT_MUSIC,
} buzzer_event_t;

/* Task Handles */
TaskHandle_t motor_control_handle;
TaskHandle_t rgb_breath_handle;
TaskHandle_t button_handle;
TaskHandle_t buzzer_handle;
TaskHandle_t imu_handle;

/* Task Definition */
void task_motor_control(void * argument);
void task_rgb_breath(void * argument);
void task_button(void * argument);
void task_buzzer(void * argument);
void task_imu(void * argument);

int main(void){
    SYSCFG_DL_init();

    DL_SYSCTL_disableSleepOnExit();

    NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_1_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_2_INST_INT_IRQN);

    DL_Timer_startCounter(PWM_MOTOR_12_INST);
    DL_Timer_startCounter(PWM_MOTOR_34_INST);
    DL_Timer_startCounter(PWM_0_INST);
    DL_Timer_startCounter(PWM_1_INST);
    DL_Timer_startCounter(PWM_SERVO_INST);
    DL_Timer_setLoadValue(TIMER_ENCODER_INST, 0xFFFFFFFF);
    DL_Timer_startCounter(TIMER_ENCODER_INST);

    uart_init();
    uart1_send("System Initialization Start.\r\n", 30);

    OLED_Init();

    xTaskCreate(task_motor_control, "task_motor_control", 512, NULL, 3, &motor_control_handle);
    xTaskCreate(task_rgb_breath, "task_rgb_breath", 512, NULL, 2, &rgb_breath_handle);
    // xTaskCreate(task_button, "task_button", 256, NULL, 2, &button_handle);
    xTaskCreate(task_buzzer, "task_buzzer", 512, NULL, 2, &buzzer_handle);
    // xTaskCreate(task_imu, "task_imu", 256, NULL, 2, &imu_handle);
    vTaskStartScheduler();  // Start FreeRTOS scheduler

    while(1)
    {

    }

    return 0;
}

void task_motor_control(void * argument)
{
    (void) argument;
    const TickType_t xFrequency = pdMS_TO_TICKS(10);
    extern tb6612_cb_t motor_cb[4];
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    tb6612_set_velocity_motor1(2*3.1415926);
    tb6612_set_velocity_motor2(2*3.1415926);
    tb6612_set_velocity_motor3(2*3.1415926);
    tb6612_set_velocity_motor4(2*3.1415926);
    for(;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        tb6612_update_control_motor1();
        tb6612_update_control_motor2();
        tb6612_update_control_motor3();
        tb6612_update_control_motor4();

        OLED_Printf(0, 1*8, OLED_6X8, "v1:%-5.2f p1:%-7.2f", tb6612_get_velocity_motor1(), tb6612_get_position_motor1());
        OLED_Printf(0, 2*8, OLED_6X8, "v2:%-5.2f p2:%-7.2f", tb6612_get_velocity_motor2(), tb6612_get_position_motor2());
        OLED_Printf(0, 3*8, OLED_6X8, "v3:%-5.2f p3:%-7.2f", tb6612_get_velocity_motor3(), tb6612_get_position_motor3());
        OLED_Printf(0, 4*8, OLED_6X8, "v4:%-5.2f p4:%-7.2f", tb6612_get_velocity_motor4(), tb6612_get_position_motor4());

        OLED_Update();
    }
}

void task_rgb_breath(void * argument)
{
    (void) argument;
    const TickType_t xFrequency = pdMS_TO_TICKS(10);
    const uint8_t steps = 50;
    TickType_t xLastWakeTime;
    uint8_t state = 0;
    uint32_t compare_val = 0;
    uint32_t load_val = 1000;
    uint32_t count = steps;
    xLastWakeTime = xTaskGetTickCount();
    for(;;){
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        load_val = DL_Timer_getLoadValue(PWM_0_INST);
        switch(state){
        case 0:
            count--;
            compare_val = count * load_val / steps;
            DL_Timer_setCaptureCompareValue(PWM_0_INST, compare_val, GPIO_PWM_0_C1_IDX);
            DL_Timer_setCaptureCompareValue(PWM_1_INST, 1000, GPIO_PWM_1_C1_IDX);
            DL_Timer_setCaptureCompareValue(PWM_1_INST, 1000, GPIO_PWM_1_C0_IDX);
            if(count == 0){
                state = 1;
            }
            break;
        case 1:
            count++;
            compare_val = count * load_val / steps;
            DL_Timer_setCaptureCompareValue(PWM_0_INST, compare_val, GPIO_PWM_0_C1_IDX);
            DL_Timer_setCaptureCompareValue(PWM_1_INST, 1000, GPIO_PWM_1_C1_IDX);
            DL_Timer_setCaptureCompareValue(PWM_1_INST, 1000, GPIO_PWM_1_C0_IDX);
            if(count >= steps){
                state = 2;
            }
            break;
        case 2:
            count--;
            compare_val = count * 1000 / steps;
            DL_Timer_setCaptureCompareValue(PWM_0_INST, load_val, GPIO_PWM_0_C1_IDX);
            DL_Timer_setCaptureCompareValue(PWM_1_INST, compare_val, GPIO_PWM_1_C1_IDX);
            DL_Timer_setCaptureCompareValue(PWM_1_INST, 1000, GPIO_PWM_1_C0_IDX);
            if(count == 0){
                state = 3;
            }
            break;
        case 3:
            count++;
            compare_val = count * 1000 / steps;
            DL_Timer_setCaptureCompareValue(PWM_0_INST, load_val, GPIO_PWM_0_C1_IDX);
            DL_Timer_setCaptureCompareValue(PWM_1_INST, compare_val, GPIO_PWM_1_C1_IDX);
            DL_Timer_setCaptureCompareValue(PWM_1_INST, 1000, GPIO_PWM_1_C0_IDX);
            if(count >= steps){
                state = 4;
            }
            break;
        case 4:
            count--;
            compare_val = count * 1000 / steps;
            DL_Timer_setCaptureCompareValue(PWM_0_INST, load_val, GPIO_PWM_0_C1_IDX);
            DL_Timer_setCaptureCompareValue(PWM_1_INST, 1000, GPIO_PWM_1_C1_IDX);
            DL_Timer_setCaptureCompareValue(PWM_1_INST, compare_val, GPIO_PWM_1_C0_IDX);
            if(count == 0){
                state = 5;
            }
            break;
        case 5:
            count++;
            compare_val = count * 1000 / steps;
            DL_Timer_setCaptureCompareValue(PWM_0_INST, load_val, GPIO_PWM_0_C1_IDX);
            DL_Timer_setCaptureCompareValue(PWM_1_INST, 1000, GPIO_PWM_1_C1_IDX);
            DL_Timer_setCaptureCompareValue(PWM_1_INST, compare_val, GPIO_PWM_1_C0_IDX);
            if(count >= steps){
                state = 0;
            }
            break;
        default:
            break;
        }
    }
}


void task_button(void * argument)
{
    (void) argument;
    button_event_t eButtonEvent = BUTTON_EVENT_NONE;
    TickType_t xLastPressTime = 0; // Timestamp of the last button press
    TickType_t xPressDuration = 0; // Duration of the button press

    for (;;)
    {
        // Wait for a task notification
        uint32_t ulNotificationValue;
        BaseType_t xResult = xTaskNotifyWait(0, 0, &ulNotificationValue, portMAX_DELAY);

        if (xResult == pdPASS)
        {
            // Notification received, process the button event
            // Debounce: Wait for 20ms and check the button state again
            vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_TIME_MS));

            if (DL_GPIO_readPins(GPIO_GRP_BUTTON_PORT, GPIO_GRP_BUTTON_PIN_BUTTON1_PIN) == 0) // Assuming gpio_read reads the GPIO state
            {
                // Button is indeed pressed, start timing
                TickType_t xStartTime = xTaskGetTickCount();

                // Wait for the button to be released
                while (DL_GPIO_readPins(GPIO_GRP_BUTTON_PORT, GPIO_GRP_BUTTON_PIN_BUTTON1_PIN) == 0)
                {
                    vTaskDelay(pdMS_TO_TICKS(10)); // Check every 10ms
                }

                // Calculate the duration of the button press
                xPressDuration = xTaskGetTickCount() - xStartTime;

                if (xPressDuration < pdMS_TO_TICKS(LONG_PRESS_TIME_MS))
                {
                    // Short press
                    eButtonEvent = BUTTON_EVENT_SHORT_PRESS;
                }
                else
                {
                    // Long press
                    eButtonEvent = BUTTON_EVENT_LONG_PRESS;
                }

                // Detect double-click event
                if (xLastPressTime != 0 && (xTaskGetTickCount() - xLastPressTime) < pdMS_TO_TICKS(500))
                {
                    // Second press detected within 500ms
                    if (eButtonEvent == BUTTON_EVENT_SHORT_PRESS)
                    {
                        if (xPressDuration < pdMS_TO_TICKS(LONG_PRESS_TIME_MS))
                        {
                            // Fast double-click
                            eButtonEvent = BUTTON_EVENT_DOUBLE_CLICK;
                        }
                        else
                        {
                            // Double-click with long press
                            eButtonEvent = BUTTON_EVENT_DOUBLE_LONG_PRESS;
                        }
                    }
                }

                // Record the timestamp of this button press
                xLastPressTime = xTaskGetTickCount();

                // If it's not a double-click event, wait to confirm if there is a second click
                if (eButtonEvent == BUTTON_EVENT_SHORT_PRESS)
                {
                    // Wait for 500ms to confirm if there is a second click
                    vTaskDelay(pdMS_TO_TICKS(500));

                    // Check again if there is a new button event
                    if (xTaskGetTickCount() - xLastPressTime < pdMS_TO_TICKS(500))
                    {
                        // If a new button event is detected during the wait, treat it as a double-click
                        eButtonEvent = BUTTON_EVENT_DOUBLE_CLICK;
                    }
                }

                // Execute actions based on the button event
                switch (eButtonEvent)
                {
                    case BUTTON_EVENT_SHORT_PRESS:
                        xTaskNotify(buzzer_handle, BUZZER_EVENT_NOTICE, eSetValueWithOverwrite);
                        break;
                    case BUTTON_EVENT_LONG_PRESS:
                        break;
                    case BUTTON_EVENT_DOUBLE_CLICK:
                        break;
                    case BUTTON_EVENT_DOUBLE_LONG_PRESS:
                        break;
                    default:
                        break;
                }
            }
        }
    }
}
void task_buzzer(void * argument)
{
    (void) argument;
    BaseType_t xResult;
    uint32_t ulNotificationValue;
    DL_Timer_setLoadValue(PWM_0_INST, 32000-1);
    DL_Timer_setCaptureCompareValue(PWM_0_INST, 16000, GPIO_PWM_0_C0_IDX);
    vTaskDelay(pdMS_TO_TICKS(200));
    DL_Timer_setCaptureCompareValue(PWM_0_INST, 0, GPIO_PWM_0_C0_IDX);
    for(;;)
    {
        xResult = xTaskNotifyWait(0, 0, &ulNotificationValue, portMAX_DELAY);
        switch (ulNotificationValue)
        {
        case BUZZER_EVENT_NOTICE:
            // Short Beep
            DL_Timer_setLoadValue(PWM_0_INST, 32000-1);
            DL_Timer_setCaptureCompareValue(PWM_0_INST, 16000, GPIO_PWM_0_C0_IDX);
            vTaskDelay(pdMS_TO_TICKS(200));
            DL_Timer_setCaptureCompareValue(PWM_0_INST, 0, GPIO_PWM_0_C0_IDX);
            break;
        
        default:
            break;
        }
    }
}

void task_imu(void * argument)
{
    (void) argument;
    const TickType_t xFrequency = pdMS_TO_TICKS(10);
    const uint8_t steps = 50;
    TickType_t xLastWakeTime;
    // LSM6DSO_Axes_t acceleration;
    // LSM6DSO_Axes_t angular_velocity;
    uint8_t device_id;
    // lsm6dso_io.Init = NULL;
    // lsm6dso_io.DeInit = NULL;
    // lsm6dso_io.BusType = 1; // 4-wire SPI
    // lsm6dso_io.GetTick = lsm6dsx_custom_get_tick;
    // lsm6dso_io.ReadReg = lsm6dsx_spi_read_reg;
    // lsm6dso_io.WriteReg = lsm6dsx_spi_write_reg;
    // lsm6dso_io.Delay = lsm6dsx_custom_delay;
    // LSM6DSO_RegisterBusIO(&lsm6dso_obj, &lsm6dso_io);
    // LSM6DSO_Init(&lsm6dso_obj);
    // LSM6DSO_ReadID(&lsm6dso_obj, &device_id);
    // lsm6dsx_spi_read_reg(0, LSM6DSO_WHO_AM_I, &device_id, 1);
    uart1_printf("lsm6dso id:0x%X\r\n", device_id);
    // OLED_ShowHexNum(6*8, 8*2, device_id, 2, OLED_6X8);
    // if(device_id == LSM6DSO_ID){
    //     OLED_ShowString(6*8, 8*1, "IMU OK", OLED_6X8);
    //     OLED_Update();
    // }else{
    //     OLED_ShowString(6*8, 8*1, "IMU ERR", OLED_6X8);
    //     uart1_send("IMU ERROR\r\n", 11);
    // }
    // OLED_Update();
    for(;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        // LSM6DSO_ACC_GetAxes(&lsm6dso_obj, &acceleration);
        // LSM6DSO_GYRO_GetAxes(&lsm6dso_obj, &angular_velocity);
        // uart1_printf("Gx:%d Gy:%d Gz:%d Ax:%d Ay:%d Az:%d\r\n", angular_velocity.x, angular_velocity.y, angular_velocity.z, acceleration.x, acceleration.y, acceleration.z);
        // OLED_ShowNum(6*3, 5*8, angular_velocity.x, 5, OLED_6X8);
        // OLED_ShowNum(6*3, 6*8, angular_velocity.y, 5, OLED_6X8);
        // OLED_ShowNum(6*3, 7*8, angular_velocity.z, 5, OLED_6X8);
        // OLED_ShowNum(6*14, 5*8, acceleration.x, 5, OLED_6X8);
        // OLED_ShowNum(6*14, 6*8, acceleration.y, 5, OLED_6X8);
        // OLED_ShowNum(6*14, 7*8, acceleration.z, 5, OLED_6X8);
        // OLED_Update();
    }
}

int32_t lsm6dsx_spi_read_reg(void* handle, uint8_t reg, uint8_t* data, uint16_t len){
    DL_SPI_setChipSelect(handle, DL_SPI_CHIP_SELECT_0);
    reg |= 0x80;
    DL_SPI_transmitDataBlocking8(handle, reg);
    for(uint16_t i = 0; i < len; i++){
        data[i] = DL_SPI_receiveDataBlocking8(handle);
    }
    DL_SPI_setChipSelect(handle, DL_SPI_CHIP_SELECT_NONE);
    return 0;
}

int32_t lsm6dsx_spi_write_reg(void* handle, uint8_t reg, uint8_t* data, uint16_t len){
    DL_SPI_setChipSelect(handle, DL_SPI_CHIP_SELECT_0);
    DL_SPI_transmitDataBlocking8(handle, reg);
    for(uint16_t i = 0; i < len; i++){
        DL_SPI_transmitDataBlocking8(handle, data[i]);
    }
    DL_SPI_setChipSelect(handle, DL_SPI_CHIP_SELECT_NONE);
    return 0;
}

void lsm6dsx_custom_delay(uint32_t milliseconds){
    vTaskDelay(pdMS_TO_TICKS(milliseconds));
}

int32_t lsm6dsx_custom_get_tick(void){
    return (int32_t)xTaskGetTickCount();
}

void qmc5883l_i2c_write(uint8_t reg, uint8_t* data, uint16_t len){
    /*
     * Fill FIFO with data. This example will send a MAX of 8 bytes since it
     * doesn't handle the case where FIFO is full
     */
    DL_I2C_fillControllerTXFIFO(I2C_0_INST, data, len);

    /* Wait for I2C to be Idle */
    while (!(DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_IDLE));

    /* Send the packet to the controller.
     * This function will send Start + Stop automatically.
     */
    DL_I2C_startControllerTransfer(I2C_0_INST, 0x0D, DL_I2C_CONTROLLER_DIRECTION_TX, len);

    /* Poll until the Controller writes all bytes */
    while (DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);

    /* Trap if there was an error */
    if (DL_I2C_getControllerStatus(I2C_0_INST) &
        DL_I2C_CONTROLLER_STATUS_ERROR) {
        /* LED will remain high if there is an error */
        __BKPT(0);
    }
}

void qmc5883l_i2c_read(uint8_t reg, uint8_t* data, uint16_t len){
    DL_I2C_fillControllerTXFIFO(I2C_0_INST, &reg, 1);
    // while (!(DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_IDLE));
    DL_I2C_startControllerTransferAdvanced(I2C_0_INST, 0x0D, DL_I2C_CONTROLLER_DIRECTION_TX, 1,
        DL_I2C_CONTROLLER_START_ENABLE, DL_I2C_CONTROLLER_STOP_DISABLE, DL_I2C_CONTROLLER_ACK_DISABLE);
    // while (DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS);

    DL_I2C_startControllerTransfer(I2C_0_INST, 0x0D, DL_I2C_CONTROLLER_DIRECTION_RX, len);
    for(uint8_t i = 0; i < len; i++){
        // while (DL_I2C_isControllerRXFIFOEmpty(I2C_0_INST));
        data[i] = DL_I2C_receiveControllerData(I2C_0_INST);
    }
}
#if (configCHECK_FOR_STACK_OVERFLOW)
    /*
     *  ======== vApplicationStackOverflowHook ========
     *  When stack overflow checking is enabled the application must provide a
     *  stack overflow hook function. This default hook function is declared as
     *  weak, and will be used by default, unless the application specifically
     *  provides its own hook function.
     */
    #if defined(__IAR_SYSTEMS_ICC__)
__weak void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
    #elif (defined(__TI_COMPILER_VERSION__))
        #pragma WEAK(vApplicationStackOverflowHook)
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
    #elif (defined(__GNUC__) || defined(__ti_version__))
void __attribute__((weak)) vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
    #endif
{
    /* default to spin upon stack overflow */
    while (1) {}
}
#endif

#if (configSUPPORT_STATIC_ALLOCATION == 1)
/*
 *  ======== vApplicationGetIdleTaskMemory ========
 *  When static allocation is enabled, the app must provide this callback
 *  function for use by the Idle task.
 */
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   uint32_t *pulIdleTaskStackSize)
{
    static StaticTask_t xIdleTaskTCB;
    static StackType_t uxIdleTaskStack[configIDLE_TASK_STACK_DEPTH];

    *ppxIdleTaskTCBBuffer   = &xIdleTaskTCB;
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;
    *pulIdleTaskStackSize   = configIDLE_TASK_STACK_DEPTH;
}

#if (configUSE_TIMERS == 1)
/*
 *  ======== vApplicationGetTimerTaskMemory ========
 *  When static allocation is enabled, and timers are used, the app must provide
 *  this callback function for use by the Timer Service task.
 */
void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
                                    StackType_t **ppxTimerTaskStackBuffer,
                                    uint32_t *pulTimerTaskStackSize)
{
    static StaticTask_t xTimerTaskTCB;
    static StackType_t uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH];

    *ppxTimerTaskTCBBuffer   = &xTimerTaskTCB;
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;
    *pulTimerTaskStackSize   = configTIMER_TASK_STACK_DEPTH;
}
#endif
#endif