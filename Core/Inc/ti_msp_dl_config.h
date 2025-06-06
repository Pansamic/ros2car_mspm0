/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)



#define CPUCLK_FREQ                                                     32000000



/* Defines for PWM_MOTOR_12 */
#define PWM_MOTOR_12_INST                                                  TIMG7
#define PWM_MOTOR_12_INST_IRQHandler                            TIMG7_IRQHandler
#define PWM_MOTOR_12_INST_INT_IRQN                              (TIMG7_INT_IRQn)
#define PWM_MOTOR_12_INST_CLK_FREQ                                      32000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_MOTOR_12_C0_PORT                                          GPIOA
#define GPIO_PWM_MOTOR_12_C0_PIN                                  DL_GPIO_PIN_26
#define GPIO_PWM_MOTOR_12_C0_IOMUX                               (IOMUX_PINCM59)
#define GPIO_PWM_MOTOR_12_C0_IOMUX_FUNC              IOMUX_PINCM59_PF_TIMG7_CCP0
#define GPIO_PWM_MOTOR_12_C0_IDX                             DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_MOTOR_12_C1_PORT                                          GPIOA
#define GPIO_PWM_MOTOR_12_C1_PIN                                   DL_GPIO_PIN_4
#define GPIO_PWM_MOTOR_12_C1_IOMUX                                (IOMUX_PINCM9)
#define GPIO_PWM_MOTOR_12_C1_IOMUX_FUNC               IOMUX_PINCM9_PF_TIMG7_CCP1
#define GPIO_PWM_MOTOR_12_C1_IDX                             DL_TIMER_CC_1_INDEX

/* Defines for PWM_MOTOR_34 */
#define PWM_MOTOR_34_INST                                                  TIMG6
#define PWM_MOTOR_34_INST_IRQHandler                            TIMG6_IRQHandler
#define PWM_MOTOR_34_INST_INT_IRQN                              (TIMG6_INT_IRQn)
#define PWM_MOTOR_34_INST_CLK_FREQ                                      32000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_MOTOR_34_C0_PORT                                          GPIOA
#define GPIO_PWM_MOTOR_34_C0_PIN                                  DL_GPIO_PIN_29
#define GPIO_PWM_MOTOR_34_C0_IOMUX                                (IOMUX_PINCM4)
#define GPIO_PWM_MOTOR_34_C0_IOMUX_FUNC               IOMUX_PINCM4_PF_TIMG6_CCP0
#define GPIO_PWM_MOTOR_34_C0_IDX                             DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_MOTOR_34_C1_PORT                                          GPIOA
#define GPIO_PWM_MOTOR_34_C1_PIN                                  DL_GPIO_PIN_30
#define GPIO_PWM_MOTOR_34_C1_IOMUX                                (IOMUX_PINCM5)
#define GPIO_PWM_MOTOR_34_C1_IOMUX_FUNC               IOMUX_PINCM5_PF_TIMG6_CCP1
#define GPIO_PWM_MOTOR_34_C1_IDX                             DL_TIMER_CC_1_INDEX

/* Defines for PWM_0 */
#define PWM_0_INST                                                         TIMG0
#define PWM_0_INST_IRQHandler                                   TIMG0_IRQHandler
#define PWM_0_INST_INT_IRQN                                     (TIMG0_INT_IRQn)
#define PWM_0_INST_CLK_FREQ                                             32000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_0_C0_PORT                                                 GPIOA
#define GPIO_PWM_0_C0_PIN                                         DL_GPIO_PIN_23
#define GPIO_PWM_0_C0_IOMUX                                      (IOMUX_PINCM53)
#define GPIO_PWM_0_C0_IOMUX_FUNC                     IOMUX_PINCM53_PF_TIMG0_CCP0
#define GPIO_PWM_0_C0_IDX                                    DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_0_C1_PORT                                                 GPIOA
#define GPIO_PWM_0_C1_PIN                                         DL_GPIO_PIN_13
#define GPIO_PWM_0_C1_IOMUX                                      (IOMUX_PINCM35)
#define GPIO_PWM_0_C1_IOMUX_FUNC                     IOMUX_PINCM35_PF_TIMG0_CCP1
#define GPIO_PWM_0_C1_IDX                                    DL_TIMER_CC_1_INDEX

/* Defines for PWM_1 */
#define PWM_1_INST                                                         TIMA1
#define PWM_1_INST_IRQHandler                                   TIMA1_IRQHandler
#define PWM_1_INST_INT_IRQN                                     (TIMA1_INT_IRQn)
#define PWM_1_INST_CLK_FREQ                                             32000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_1_C0_PORT                                                 GPIOB
#define GPIO_PWM_1_C0_PIN                                          DL_GPIO_PIN_0
#define GPIO_PWM_1_C0_IOMUX                                      (IOMUX_PINCM12)
#define GPIO_PWM_1_C0_IOMUX_FUNC                     IOMUX_PINCM12_PF_TIMA1_CCP0
#define GPIO_PWM_1_C0_IDX                                    DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_1_C1_PORT                                                 GPIOA
#define GPIO_PWM_1_C1_PIN                                         DL_GPIO_PIN_24
#define GPIO_PWM_1_C1_IOMUX                                      (IOMUX_PINCM54)
#define GPIO_PWM_1_C1_IOMUX_FUNC                     IOMUX_PINCM54_PF_TIMA1_CCP1
#define GPIO_PWM_1_C1_IDX                                    DL_TIMER_CC_1_INDEX

/* Defines for PWM_SERVO */
#define PWM_SERVO_INST                                                     TIMA0
#define PWM_SERVO_INST_IRQHandler                               TIMA0_IRQHandler
#define PWM_SERVO_INST_INT_IRQN                                 (TIMA0_INT_IRQn)
#define PWM_SERVO_INST_CLK_FREQ                                         32000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_SERVO_C0_PORT                                             GPIOB
#define GPIO_PWM_SERVO_C0_PIN                                     DL_GPIO_PIN_14
#define GPIO_PWM_SERVO_C0_IOMUX                                  (IOMUX_PINCM31)
#define GPIO_PWM_SERVO_C0_IOMUX_FUNC                 IOMUX_PINCM31_PF_TIMA0_CCP0
#define GPIO_PWM_SERVO_C0_IDX                                DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_SERVO_C1_PORT                                             GPIOA
#define GPIO_PWM_SERVO_C1_PIN                                      DL_GPIO_PIN_7
#define GPIO_PWM_SERVO_C1_IOMUX                                  (IOMUX_PINCM14)
#define GPIO_PWM_SERVO_C1_IOMUX_FUNC                 IOMUX_PINCM14_PF_TIMA0_CCP1
#define GPIO_PWM_SERVO_C1_IDX                                DL_TIMER_CC_1_INDEX
/* GPIO defines for channel 2 */
#define GPIO_PWM_SERVO_C2_PORT                                             GPIOB
#define GPIO_PWM_SERVO_C2_PIN                                     DL_GPIO_PIN_12
#define GPIO_PWM_SERVO_C2_IOMUX                                  (IOMUX_PINCM29)
#define GPIO_PWM_SERVO_C2_IOMUX_FUNC                 IOMUX_PINCM29_PF_TIMA0_CCP2
#define GPIO_PWM_SERVO_C2_IDX                                DL_TIMER_CC_2_INDEX
/* GPIO defines for channel 3 */
#define GPIO_PWM_SERVO_C3_PORT                                             GPIOA
#define GPIO_PWM_SERVO_C3_PIN                                     DL_GPIO_PIN_12
#define GPIO_PWM_SERVO_C3_IOMUX                                  (IOMUX_PINCM34)
#define GPIO_PWM_SERVO_C3_IOMUX_FUNC                 IOMUX_PINCM34_PF_TIMA0_CCP3
#define GPIO_PWM_SERVO_C3_IDX                                DL_TIMER_CC_3_INDEX



/* Defines for TIMER_ENCODER */
#define TIMER_ENCODER_INST                                              (TIMG12)
#define TIMER_ENCODER_INST_IRQHandler                          TIMG12_IRQHandler
#define TIMER_ENCODER_INST_INT_IRQN                            (TIMG12_INT_IRQn)
#define TIMER_ENCODER_INST_LOAD_VALUE                                       (0U)




/* Defines for I2C_0 */
#define I2C_0_INST                                                          I2C0
#define I2C_0_INST_IRQHandler                                    I2C0_IRQHandler
#define I2C_0_INST_INT_IRQN                                        I2C0_INT_IRQn
#define I2C_0_BUS_SPEED_HZ                                                100000
#define GPIO_I2C_0_SDA_PORT                                                GPIOA
#define GPIO_I2C_0_SDA_PIN                                        DL_GPIO_PIN_28
#define GPIO_I2C_0_IOMUX_SDA                                      (IOMUX_PINCM3)
#define GPIO_I2C_0_IOMUX_SDA_FUNC                       IOMUX_PINCM3_PF_I2C0_SDA
#define GPIO_I2C_0_SCL_PORT                                                GPIOA
#define GPIO_I2C_0_SCL_PIN                                        DL_GPIO_PIN_31
#define GPIO_I2C_0_IOMUX_SCL                                      (IOMUX_PINCM6)
#define GPIO_I2C_0_IOMUX_SCL_FUNC                       IOMUX_PINCM6_PF_I2C0_SCL

/* Defines for I2C_1 */
#define I2C_1_INST                                                          I2C1
#define I2C_1_INST_IRQHandler                                    I2C1_IRQHandler
#define I2C_1_INST_INT_IRQN                                        I2C1_INT_IRQn
#define GPIO_I2C_1_SDA_PORT                                                GPIOB
#define GPIO_I2C_1_SDA_PIN                                         DL_GPIO_PIN_3
#define GPIO_I2C_1_IOMUX_SDA                                     (IOMUX_PINCM16)
#define GPIO_I2C_1_IOMUX_SDA_FUNC                      IOMUX_PINCM16_PF_I2C1_SDA
#define GPIO_I2C_1_SCL_PORT                                                GPIOB
#define GPIO_I2C_1_SCL_PIN                                         DL_GPIO_PIN_2
#define GPIO_I2C_1_IOMUX_SCL                                     (IOMUX_PINCM15)
#define GPIO_I2C_1_IOMUX_SCL_FUNC                      IOMUX_PINCM15_PF_I2C1_SCL


/* Defines for UART_2 */
#define UART_2_INST                                                        UART2
#define UART_2_INST_FREQUENCY                                           32000000
#define UART_2_INST_IRQHandler                                  UART2_IRQHandler
#define UART_2_INST_INT_IRQN                                      UART2_INT_IRQn
#define GPIO_UART_2_RX_PORT                                                GPIOA
#define GPIO_UART_2_TX_PORT                                                GPIOA
#define GPIO_UART_2_RX_PIN                                        DL_GPIO_PIN_22
#define GPIO_UART_2_TX_PIN                                        DL_GPIO_PIN_21
#define GPIO_UART_2_IOMUX_RX                                     (IOMUX_PINCM47)
#define GPIO_UART_2_IOMUX_TX                                     (IOMUX_PINCM46)
#define GPIO_UART_2_IOMUX_RX_FUNC                      IOMUX_PINCM47_PF_UART2_RX
#define GPIO_UART_2_IOMUX_TX_FUNC                      IOMUX_PINCM46_PF_UART2_TX
#define UART_2_BAUD_RATE                                                (921600)
#define UART_2_IBRD_32_MHZ_921600_BAUD                                       (2)
#define UART_2_FBRD_32_MHZ_921600_BAUD                                      (11)
/* Defines for UART_1 */
#define UART_1_INST                                                        UART1
#define UART_1_INST_FREQUENCY                                           32000000
#define UART_1_INST_IRQHandler                                  UART1_IRQHandler
#define UART_1_INST_INT_IRQN                                      UART1_INT_IRQn
#define GPIO_UART_1_RX_PORT                                                GPIOA
#define GPIO_UART_1_TX_PORT                                                GPIOA
#define GPIO_UART_1_RX_PIN                                        DL_GPIO_PIN_18
#define GPIO_UART_1_TX_PIN                                        DL_GPIO_PIN_17
#define GPIO_UART_1_IOMUX_RX                                     (IOMUX_PINCM40)
#define GPIO_UART_1_IOMUX_TX                                     (IOMUX_PINCM39)
#define GPIO_UART_1_IOMUX_RX_FUNC                      IOMUX_PINCM40_PF_UART1_RX
#define GPIO_UART_1_IOMUX_TX_FUNC                      IOMUX_PINCM39_PF_UART1_TX
#define UART_1_BAUD_RATE                                                (921600)
#define UART_1_IBRD_32_MHZ_921600_BAUD                                       (2)
#define UART_1_FBRD_32_MHZ_921600_BAUD                                      (11)
/* Defines for UART_0 */
#define UART_0_INST                                                        UART0
#define UART_0_INST_FREQUENCY                                           32000000
#define UART_0_INST_IRQHandler                                  UART0_IRQHandler
#define UART_0_INST_INT_IRQN                                      UART0_INT_IRQn
#define GPIO_UART_0_RX_PORT                                                GPIOA
#define GPIO_UART_0_TX_PORT                                                GPIOA
#define GPIO_UART_0_RX_PIN                                         DL_GPIO_PIN_1
#define GPIO_UART_0_TX_PIN                                         DL_GPIO_PIN_0
#define GPIO_UART_0_IOMUX_RX                                      (IOMUX_PINCM2)
#define GPIO_UART_0_IOMUX_TX                                      (IOMUX_PINCM1)
#define GPIO_UART_0_IOMUX_RX_FUNC                       IOMUX_PINCM2_PF_UART0_RX
#define GPIO_UART_0_IOMUX_TX_FUNC                       IOMUX_PINCM1_PF_UART0_TX
#define UART_0_BAUD_RATE                                                (921600)
#define UART_0_IBRD_32_MHZ_921600_BAUD                                       (2)
#define UART_0_FBRD_32_MHZ_921600_BAUD                                      (11)




/* Defines for SPI_0 */
#define SPI_0_INST                                                         SPI0
#define SPI_0_INST_IRQHandler                                   SPI0_IRQHandler
#define SPI_0_INST_INT_IRQN                                       SPI0_INT_IRQn
#define GPIO_SPI_0_PICO_PORT                                              GPIOA
#define GPIO_SPI_0_PICO_PIN                                      DL_GPIO_PIN_14
#define GPIO_SPI_0_IOMUX_PICO                                   (IOMUX_PINCM36)
#define GPIO_SPI_0_IOMUX_PICO_FUNC                   IOMUX_PINCM36_PF_SPI0_PICO
#define GPIO_SPI_0_POCI_PORT                                              GPIOA
#define GPIO_SPI_0_POCI_PIN                                      DL_GPIO_PIN_10
#define GPIO_SPI_0_IOMUX_POCI                                   (IOMUX_PINCM21)
#define GPIO_SPI_0_IOMUX_POCI_FUNC                   IOMUX_PINCM21_PF_SPI0_POCI
/* GPIO configuration for SPI_0 */
#define GPIO_SPI_0_SCLK_PORT                                              GPIOA
#define GPIO_SPI_0_SCLK_PIN                                      DL_GPIO_PIN_11
#define GPIO_SPI_0_IOMUX_SCLK                                   (IOMUX_PINCM22)
#define GPIO_SPI_0_IOMUX_SCLK_FUNC                   IOMUX_PINCM22_PF_SPI0_SCLK
#define GPIO_SPI_0_CS0_PORT                                               GPIOA
#define GPIO_SPI_0_CS0_PIN                                        DL_GPIO_PIN_8
#define GPIO_SPI_0_IOMUX_CS0                                    (IOMUX_PINCM19)
#define GPIO_SPI_0_IOMUX_CS0_FUNC                     IOMUX_PINCM19_PF_SPI0_CS0
#define GPIO_SPI_0_CS1_PORT                                               GPIOA
#define GPIO_SPI_0_CS1_PIN                                        DL_GPIO_PIN_3
#define GPIO_SPI_0_IOMUX_CS1                                     (IOMUX_PINCM8)
#define GPIO_SPI_0_IOMUX_CS1_FUNC                IOMUX_PINCM8_PF_SPI0_CS1_POCI1
/* Defines for SPI_1 */
#define SPI_1_INST                                                         SPI1
#define SPI_1_INST_IRQHandler                                   SPI1_IRQHandler
#define SPI_1_INST_INT_IRQN                                       SPI1_INT_IRQn
#define GPIO_SPI_1_PICO_PORT                                              GPIOB
#define GPIO_SPI_1_PICO_PIN                                      DL_GPIO_PIN_15
#define GPIO_SPI_1_IOMUX_PICO                                   (IOMUX_PINCM32)
#define GPIO_SPI_1_IOMUX_PICO_FUNC                   IOMUX_PINCM32_PF_SPI1_PICO
#define GPIO_SPI_1_POCI_PORT                                              GPIOA
#define GPIO_SPI_1_POCI_PIN                                      DL_GPIO_PIN_16
#define GPIO_SPI_1_IOMUX_POCI                                   (IOMUX_PINCM38)
#define GPIO_SPI_1_IOMUX_POCI_FUNC                   IOMUX_PINCM38_PF_SPI1_POCI
/* GPIO configuration for SPI_1 */
#define GPIO_SPI_1_SCLK_PORT                                              GPIOB
#define GPIO_SPI_1_SCLK_PIN                                      DL_GPIO_PIN_16
#define GPIO_SPI_1_IOMUX_SCLK                                   (IOMUX_PINCM33)
#define GPIO_SPI_1_IOMUX_SCLK_FUNC                   IOMUX_PINCM33_PF_SPI1_SCLK
#define GPIO_SPI_1_CS0_PORT                                               GPIOA
#define GPIO_SPI_1_CS0_PIN                                        DL_GPIO_PIN_2
#define GPIO_SPI_1_IOMUX_CS0                                     (IOMUX_PINCM7)
#define GPIO_SPI_1_IOMUX_CS0_FUNC                      IOMUX_PINCM7_PF_SPI1_CS0
#define GPIO_SPI_1_CS1_PORT                                               GPIOA
#define GPIO_SPI_1_CS1_PIN                                       DL_GPIO_PIN_27
#define GPIO_SPI_1_IOMUX_CS1                                    (IOMUX_PINCM60)
#define GPIO_SPI_1_IOMUX_CS1_FUNC               IOMUX_PINCM60_PF_SPI1_CS1_POCI1



/* Defines for ADC12_0 */
#define ADC12_0_INST                                                        ADC1
#define ADC12_0_INST_IRQHandler                                  ADC1_IRQHandler
#define ADC12_0_INST_INT_IRQN                                    (ADC1_INT_IRQn)
#define ADC12_0_ADCMEM_0                                      DL_ADC12_MEM_IDX_0
#define ADC12_0_ADCMEM_0_REF                     DL_ADC12_REFERENCE_VOLTAGE_VDDA
#define ADC12_0_ADCMEM_0_REF_VOLTAGE_V                                       3.3
#define GPIO_ADC12_0_C0_PORT                                               GPIOA
#define GPIO_ADC12_0_C0_PIN                                       DL_GPIO_PIN_15



/* Defines for DMA_UART2_RX */
#define DMA_UART2_RX_CHAN_ID                                                 (5)
#define UART_2_INST_DMA_TRIGGER_0                            (DMA_UART2_RX_TRIG)

/* Defines for DMA_UART2_TX */
#define DMA_UART2_TX_CHAN_ID                                                 (4)
#define UART_2_INST_DMA_TRIGGER_1                            (DMA_UART2_TX_TRIG)

/* Defines for DMA_UART1_RX */
#define DMA_UART1_RX_CHAN_ID                                                 (3)
#define UART_1_INST_DMA_TRIGGER_0                            (DMA_UART1_RX_TRIG)

/* Defines for DMA_UART1_TX */
#define DMA_UART1_TX_CHAN_ID                                                 (2)
#define UART_1_INST_DMA_TRIGGER_1                            (DMA_UART1_TX_TRIG)

/* Defines for DMA_UART0_RX */
#define DMA_UART0_RX_CHAN_ID                                                 (1)
#define UART_0_INST_DMA_TRIGGER_0                            (DMA_UART0_RX_TRIG)

/* Defines for DMA_UART0_TX */
#define DMA_UART0_TX_CHAN_ID                                                 (0)
#define UART_0_INST_DMA_TRIGGER_1                            (DMA_UART0_TX_TRIG)



/* Port definition for Pin Group GPIO_GRP_BUTTON */
#define GPIO_GRP_BUTTON_PORT                                             (GPIOB)

/* Defines for PIN_BUTTON1: GPIOB.5 with pinCMx 18 on package pin 53 */
// groups represented: ["GPIO_GRP_ENCODER","GPIO_GRP_IMU_INT","GPIO_GRP_BUTTON"]
// pins affected: ["PIN_MOTOR1_A","PIN_MOTOR1_B","PIN_MOTOR2_A","PIN_MOTOR2_B","PIN_MOTOR3_A","PIN_MOTOR3_B","PIN_MOTOR4_A","PIN_MOTOR4_B","PIN_INT1","PIN_INT2","PIN_BUTTON1"]
#define GPIO_MULTIPLE_GPIOB_INT_IRQN                            (GPIOB_INT_IRQn)
#define GPIO_MULTIPLE_GPIOB_INT_IIDX            (DL_INTERRUPT_GROUP1_IIDX_GPIOB)
#define GPIO_GRP_BUTTON_PIN_BUTTON1_IIDX                     (DL_GPIO_IIDX_DIO5)
#define GPIO_GRP_BUTTON_PIN_BUTTON1_PIN                          (DL_GPIO_PIN_5)
#define GPIO_GRP_BUTTON_PIN_BUTTON1_IOMUX                        (IOMUX_PINCM18)
/* Port definition for Pin Group GPIO_GRP_ENCODER */
#define GPIO_GRP_ENCODER_PORT                                            (GPIOB)

/* Defines for PIN_MOTOR1_A: GPIOB.25 with pinCMx 56 on package pin 27 */
#define GPIO_GRP_ENCODER_PIN_MOTOR1_A_IIDX                  (DL_GPIO_IIDX_DIO25)
#define GPIO_GRP_ENCODER_PIN_MOTOR1_A_PIN                       (DL_GPIO_PIN_25)
#define GPIO_GRP_ENCODER_PIN_MOTOR1_A_IOMUX                      (IOMUX_PINCM56)
/* Defines for PIN_MOTOR1_B: GPIOB.24 with pinCMx 52 on package pin 23 */
#define GPIO_GRP_ENCODER_PIN_MOTOR1_B_IIDX                  (DL_GPIO_IIDX_DIO24)
#define GPIO_GRP_ENCODER_PIN_MOTOR1_B_PIN                       (DL_GPIO_PIN_24)
#define GPIO_GRP_ENCODER_PIN_MOTOR1_B_IOMUX                      (IOMUX_PINCM52)
/* Defines for PIN_MOTOR2_A: GPIOB.23 with pinCMx 51 on package pin 22 */
#define GPIO_GRP_ENCODER_PIN_MOTOR2_A_IIDX                  (DL_GPIO_IIDX_DIO23)
#define GPIO_GRP_ENCODER_PIN_MOTOR2_A_PIN                       (DL_GPIO_PIN_23)
#define GPIO_GRP_ENCODER_PIN_MOTOR2_A_IOMUX                      (IOMUX_PINCM51)
/* Defines for PIN_MOTOR2_B: GPIOB.22 with pinCMx 50 on package pin 21 */
#define GPIO_GRP_ENCODER_PIN_MOTOR2_B_IIDX                  (DL_GPIO_IIDX_DIO22)
#define GPIO_GRP_ENCODER_PIN_MOTOR2_B_PIN                       (DL_GPIO_PIN_22)
#define GPIO_GRP_ENCODER_PIN_MOTOR2_B_IOMUX                      (IOMUX_PINCM50)
/* Defines for PIN_MOTOR3_A: GPIOB.21 with pinCMx 49 on package pin 20 */
#define GPIO_GRP_ENCODER_PIN_MOTOR3_A_IIDX                  (DL_GPIO_IIDX_DIO21)
#define GPIO_GRP_ENCODER_PIN_MOTOR3_A_PIN                       (DL_GPIO_PIN_21)
#define GPIO_GRP_ENCODER_PIN_MOTOR3_A_IOMUX                      (IOMUX_PINCM49)
/* Defines for PIN_MOTOR3_B: GPIOB.20 with pinCMx 48 on package pin 19 */
#define GPIO_GRP_ENCODER_PIN_MOTOR3_B_IIDX                  (DL_GPIO_IIDX_DIO20)
#define GPIO_GRP_ENCODER_PIN_MOTOR3_B_PIN                       (DL_GPIO_PIN_20)
#define GPIO_GRP_ENCODER_PIN_MOTOR3_B_IOMUX                      (IOMUX_PINCM48)
/* Defines for PIN_MOTOR4_A: GPIOB.19 with pinCMx 45 on package pin 16 */
#define GPIO_GRP_ENCODER_PIN_MOTOR4_A_IIDX                  (DL_GPIO_IIDX_DIO19)
#define GPIO_GRP_ENCODER_PIN_MOTOR4_A_PIN                       (DL_GPIO_PIN_19)
#define GPIO_GRP_ENCODER_PIN_MOTOR4_A_IOMUX                      (IOMUX_PINCM45)
/* Defines for PIN_MOTOR4_B: GPIOB.18 with pinCMx 44 on package pin 15 */
#define GPIO_GRP_ENCODER_PIN_MOTOR4_B_IIDX                  (DL_GPIO_IIDX_DIO18)
#define GPIO_GRP_ENCODER_PIN_MOTOR4_B_PIN                       (DL_GPIO_PIN_18)
#define GPIO_GRP_ENCODER_PIN_MOTOR4_B_IOMUX                      (IOMUX_PINCM44)
/* Defines for PIN_MOTOR1_1: GPIOB.13 with pinCMx 30 on package pin 1 */
#define GPIO_GRP_MCTRL_PIN_MOTOR1_1_PORT                                 (GPIOB)
#define GPIO_GRP_MCTRL_PIN_MOTOR1_1_PIN                         (DL_GPIO_PIN_13)
#define GPIO_GRP_MCTRL_PIN_MOTOR1_1_IOMUX                        (IOMUX_PINCM30)
/* Defines for PIN_MOTOR1_2: GPIOB.17 with pinCMx 43 on package pin 14 */
#define GPIO_GRP_MCTRL_PIN_MOTOR1_2_PORT                                 (GPIOB)
#define GPIO_GRP_MCTRL_PIN_MOTOR1_2_PIN                         (DL_GPIO_PIN_17)
#define GPIO_GRP_MCTRL_PIN_MOTOR1_2_IOMUX                        (IOMUX_PINCM43)
/* Defines for PIN_MOTOR2_1: GPIOA.25 with pinCMx 55 on package pin 26 */
#define GPIO_GRP_MCTRL_PIN_MOTOR2_1_PORT                                 (GPIOA)
#define GPIO_GRP_MCTRL_PIN_MOTOR2_1_PIN                         (DL_GPIO_PIN_25)
#define GPIO_GRP_MCTRL_PIN_MOTOR2_1_IOMUX                        (IOMUX_PINCM55)
/* Defines for PIN_MOTOR2_2: GPIOB.26 with pinCMx 57 on package pin 28 */
#define GPIO_GRP_MCTRL_PIN_MOTOR2_2_PORT                                 (GPIOB)
#define GPIO_GRP_MCTRL_PIN_MOTOR2_2_PIN                         (DL_GPIO_PIN_26)
#define GPIO_GRP_MCTRL_PIN_MOTOR2_2_IOMUX                        (IOMUX_PINCM57)
/* Defines for PIN_MOTOR3_1: GPIOB.27 with pinCMx 58 on package pin 29 */
#define GPIO_GRP_MCTRL_PIN_MOTOR3_1_PORT                                 (GPIOB)
#define GPIO_GRP_MCTRL_PIN_MOTOR3_1_PIN                         (DL_GPIO_PIN_27)
#define GPIO_GRP_MCTRL_PIN_MOTOR3_1_IOMUX                        (IOMUX_PINCM58)
/* Defines for PIN_MOTOR3_2: GPIOA.5 with pinCMx 10 on package pin 45 */
#define GPIO_GRP_MCTRL_PIN_MOTOR3_2_PORT                                 (GPIOA)
#define GPIO_GRP_MCTRL_PIN_MOTOR3_2_PIN                          (DL_GPIO_PIN_5)
#define GPIO_GRP_MCTRL_PIN_MOTOR3_2_IOMUX                        (IOMUX_PINCM10)
/* Defines for PIN_MOTOR4_1: GPIOA.6 with pinCMx 11 on package pin 46 */
#define GPIO_GRP_MCTRL_PIN_MOTOR4_1_PORT                                 (GPIOA)
#define GPIO_GRP_MCTRL_PIN_MOTOR4_1_PIN                          (DL_GPIO_PIN_6)
#define GPIO_GRP_MCTRL_PIN_MOTOR4_1_IOMUX                        (IOMUX_PINCM11)
/* Defines for PIN_MOTOR4_2: GPIOB.1 with pinCMx 13 on package pin 48 */
#define GPIO_GRP_MCTRL_PIN_MOTOR4_2_PORT                                 (GPIOB)
#define GPIO_GRP_MCTRL_PIN_MOTOR4_2_PIN                          (DL_GPIO_PIN_1)
#define GPIO_GRP_MCTRL_PIN_MOTOR4_2_IOMUX                        (IOMUX_PINCM13)
/* Port definition for Pin Group GPIO_GRP_IMU_INT */
#define GPIO_GRP_IMU_INT_PORT                                            (GPIOB)

/* Defines for PIN_INT1: GPIOB.7 with pinCMx 24 on package pin 59 */
#define GPIO_GRP_IMU_INT_PIN_INT1_IIDX                       (DL_GPIO_IIDX_DIO7)
#define GPIO_GRP_IMU_INT_PIN_INT1_PIN                            (DL_GPIO_PIN_7)
#define GPIO_GRP_IMU_INT_PIN_INT1_IOMUX                          (IOMUX_PINCM24)
/* Defines for PIN_INT2: GPIOB.8 with pinCMx 25 on package pin 60 */
#define GPIO_GRP_IMU_INT_PIN_INT2_IIDX                       (DL_GPIO_IIDX_DIO8)
#define GPIO_GRP_IMU_INT_PIN_INT2_PIN                            (DL_GPIO_PIN_8)
#define GPIO_GRP_IMU_INT_PIN_INT2_IOMUX                          (IOMUX_PINCM25)
/* Port definition for Pin Group GPIO_GRP_OLED */
#define GPIO_GRP_OLED_PORT                                               (GPIOB)

/* Defines for PIN_RESET: GPIOB.9 with pinCMx 26 on package pin 61 */
#define GPIO_GRP_OLED_PIN_RESET_PIN                              (DL_GPIO_PIN_9)
#define GPIO_GRP_OLED_PIN_RESET_IOMUX                            (IOMUX_PINCM26)
/* Defines for PIN_DC: GPIOB.10 with pinCMx 27 on package pin 62 */
#define GPIO_GRP_OLED_PIN_DC_PIN                                (DL_GPIO_PIN_10)
#define GPIO_GRP_OLED_PIN_DC_IOMUX                               (IOMUX_PINCM27)






/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_PWM_MOTOR_12_init(void);
void SYSCFG_DL_PWM_MOTOR_34_init(void);
void SYSCFG_DL_PWM_0_init(void);
void SYSCFG_DL_PWM_1_init(void);
void SYSCFG_DL_PWM_SERVO_init(void);
void SYSCFG_DL_TIMER_ENCODER_init(void);
void SYSCFG_DL_I2C_0_init(void);
void SYSCFG_DL_I2C_1_init(void);
void SYSCFG_DL_UART_2_init(void);
void SYSCFG_DL_UART_1_init(void);
void SYSCFG_DL_UART_0_init(void);
void SYSCFG_DL_SPI_0_init(void);
void SYSCFG_DL_SPI_1_init(void);
void SYSCFG_DL_ADC12_0_init(void);
void SYSCFG_DL_DMA_init(void);

void SYSCFG_DL_RTC_init(void);
void SYSCFG_DL_SYSTICK_init(void);

bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
