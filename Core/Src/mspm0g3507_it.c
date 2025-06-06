#include "ti_msp_dl_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "uart.h"
#include "tb6612.h"

extern TaskHandle_t button_handle;
extern uart_cb_t uart_cb[3];
extern tb6612_cb_t motor_cb[4];

void HardFault_Handler(void){
    while(1){
        __WFI();
    }
}


void GROUP1_IRQHandler(void){
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t pin_a = 0;
    uint32_t pin_b = 0;
    uint32_t gpioa_int = GPIOA->CPU_INT.MIS;
    uint32_t gpiob_int = GPIOB->CPU_INT.MIS;

    if ((gpiob_int & GPIO_GRP_BUTTON_PIN_BUTTON1_PIN) == GPIO_GRP_BUTTON_PIN_BUTTON1_PIN) {
        DL_GPIO_clearInterruptStatus(GPIO_GRP_BUTTON_PORT, GPIO_GRP_BUTTON_PIN_BUTTON1_PIN);
        if(button_handle != NULL){
            xTaskNotifyFromISR(button_handle, 0, eNoAction, &xHigherPriorityTaskWoken);
        }
    }
    if((gpiob_int & GPIO_GRP_ENCODER_PIN_MOTOR1_A_PIN) == GPIO_GRP_ENCODER_PIN_MOTOR1_A_PIN){
        DL_GPIO_clearInterruptStatus(GPIO_GRP_ENCODER_PORT, GPIO_GRP_ENCODER_PIN_MOTOR1_A_PIN);
        pin_a = DL_GPIO_readPins(GPIO_GRP_ENCODER_PORT, GPIO_GRP_ENCODER_PIN_MOTOR1_A_PIN);
        pin_b = DL_GPIO_readPins(GPIO_GRP_ENCODER_PORT, GPIO_GRP_ENCODER_PIN_MOTOR1_B_PIN);
        pin_a >>= 1;
        if(pin_a ^ pin_b){
            motor_cb[0].encoder_count--;
        } else {
            motor_cb[0].encoder_count++;
        }
    }else if((gpiob_int & GPIO_GRP_ENCODER_PIN_MOTOR1_B_PIN) == GPIO_GRP_ENCODER_PIN_MOTOR1_B_PIN){
        DL_GPIO_clearInterruptStatus(GPIO_GRP_ENCODER_PORT, GPIO_GRP_ENCODER_PIN_MOTOR1_B_PIN);
        pin_a = DL_GPIO_readPins(GPIO_GRP_ENCODER_PORT, GPIO_GRP_ENCODER_PIN_MOTOR1_A_PIN);
        pin_b = DL_GPIO_readPins(GPIO_GRP_ENCODER_PORT, GPIO_GRP_ENCODER_PIN_MOTOR1_B_PIN);
        pin_a >>= 1;
        if(pin_a ^ pin_b){
            motor_cb[0].encoder_count++;
        } else {
            motor_cb[0].encoder_count--;
        }
    }
    if((gpiob_int & GPIO_GRP_ENCODER_PIN_MOTOR2_A_PIN) == GPIO_GRP_ENCODER_PIN_MOTOR2_A_PIN){
        DL_GPIO_clearInterruptStatus(GPIO_GRP_ENCODER_PORT, GPIO_GRP_ENCODER_PIN_MOTOR2_A_PIN);
        pin_a = DL_GPIO_readPins(GPIO_GRP_ENCODER_PORT, GPIO_GRP_ENCODER_PIN_MOTOR2_A_PIN);
        pin_b = DL_GPIO_readPins(GPIO_GRP_ENCODER_PORT, GPIO_GRP_ENCODER_PIN_MOTOR2_B_PIN);
        pin_a >>= 1;
        if(pin_a ^ pin_b){
            motor_cb[1].encoder_count++;
        } else {
            motor_cb[1].encoder_count--;
        }
    }else if((gpiob_int & GPIO_GRP_ENCODER_PIN_MOTOR2_B_PIN) == GPIO_GRP_ENCODER_PIN_MOTOR2_B_PIN){
        DL_GPIO_clearInterruptStatus(GPIO_GRP_ENCODER_PORT, GPIO_GRP_ENCODER_PIN_MOTOR2_B_PIN);
        pin_a = DL_GPIO_readPins(GPIO_GRP_ENCODER_PORT, GPIO_GRP_ENCODER_PIN_MOTOR2_A_PIN);
        pin_b = DL_GPIO_readPins(GPIO_GRP_ENCODER_PORT, GPIO_GRP_ENCODER_PIN_MOTOR2_B_PIN);
        pin_a >>= 1;
        if(pin_a ^ pin_b){
            motor_cb[1].encoder_count--;
        } else {
            motor_cb[1].encoder_count++;
        }
    }
    if((gpiob_int & GPIO_GRP_ENCODER_PIN_MOTOR3_A_PIN) == GPIO_GRP_ENCODER_PIN_MOTOR3_A_PIN){
        DL_GPIO_clearInterruptStatus(GPIO_GRP_ENCODER_PORT, GPIO_GRP_ENCODER_PIN_MOTOR3_A_PIN);
        pin_a = DL_GPIO_readPins(GPIO_GRP_ENCODER_PORT, GPIO_GRP_ENCODER_PIN_MOTOR3_A_PIN);
        pin_b = DL_GPIO_readPins(GPIO_GRP_ENCODER_PORT, GPIO_GRP_ENCODER_PIN_MOTOR3_B_PIN);
        pin_a >>= 1;
        if(pin_a ^ pin_b){
            motor_cb[2].encoder_count--;
        } else {
            motor_cb[2].encoder_count++;
        }
    }else if((gpiob_int & GPIO_GRP_ENCODER_PIN_MOTOR3_B_PIN) == GPIO_GRP_ENCODER_PIN_MOTOR3_B_PIN){
        DL_GPIO_clearInterruptStatus(GPIO_GRP_ENCODER_PORT, GPIO_GRP_ENCODER_PIN_MOTOR3_B_PIN);
        pin_a = DL_GPIO_readPins(GPIO_GRP_ENCODER_PORT, GPIO_GRP_ENCODER_PIN_MOTOR3_A_PIN);
        pin_b = DL_GPIO_readPins(GPIO_GRP_ENCODER_PORT, GPIO_GRP_ENCODER_PIN_MOTOR3_B_PIN);
        pin_a >>= 1;
        if(pin_a ^ pin_b){
            motor_cb[2].encoder_count++;
        } else {
            motor_cb[2].encoder_count--;
        }
    }
    if((gpiob_int & GPIO_GRP_ENCODER_PIN_MOTOR4_A_PIN) == GPIO_GRP_ENCODER_PIN_MOTOR4_A_PIN){
        DL_GPIO_clearInterruptStatus(GPIO_GRP_ENCODER_PORT, GPIO_GRP_ENCODER_PIN_MOTOR4_A_PIN);
        pin_a = DL_GPIO_readPins(GPIO_GRP_ENCODER_PORT, GPIO_GRP_ENCODER_PIN_MOTOR4_A_PIN);
        pin_b = DL_GPIO_readPins(GPIO_GRP_ENCODER_PORT, GPIO_GRP_ENCODER_PIN_MOTOR4_B_PIN);
        pin_a >>= 1;
        if(pin_a ^ pin_b){
            motor_cb[3].encoder_count++;
        } else {
            motor_cb[3].encoder_count--;
        }
    }else if((gpiob_int & GPIO_GRP_ENCODER_PIN_MOTOR4_B_PIN) == GPIO_GRP_ENCODER_PIN_MOTOR4_B_PIN){
        DL_GPIO_clearInterruptStatus(GPIO_GRP_ENCODER_PORT, GPIO_GRP_ENCODER_PIN_MOTOR4_B_PIN);
        pin_a = DL_GPIO_readPins(GPIO_GRP_ENCODER_PORT, GPIO_GRP_ENCODER_PIN_MOTOR4_A_PIN);
        pin_b = DL_GPIO_readPins(GPIO_GRP_ENCODER_PORT, GPIO_GRP_ENCODER_PIN_MOTOR4_B_PIN);
        pin_a >>= 1;
        if(pin_a ^ pin_b){
            motor_cb[3].encoder_count--;
        } else {
            motor_cb[3].encoder_count++;
        }
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void UART0_IRQHandler(void){
    switch (DL_UART_getPendingInterrupt(UART_0_INST)) {
        case DL_UART_IIDX_EOT_DONE:
            /* UART TXFIFO Empty */
            
            break;
        case DL_UART_IIDX_DMA_DONE_TX:
            /* DMA is done transferring data from source to TXFIFO */

            break;
        case DL_UART_MAIN_IIDX_DMA_DONE_RX:
            /* DMA is done transferring data from RXFIFO to receive buffer*/

            break;
        default:
            break;
    }
}

void UART1_IRQHandler(void){

}

void UART2_IRQHandler(void){

}

void SPI0_IRQHandler(void){
    switch (DL_SPI_getPendingInterrupt(SPI_0_INST)) {
        case DL_SPI_IIDX_DMA_DONE_TX:
            /* DMA is done transferring data from source to TXFIFO */
            
            break;
        case DL_SPI_IIDX_TX_EMPTY:
            /* SPI is done transmitting data and TXFIFO is empty */
            
            break;
        case DL_SPI_IIDX_DMA_DONE_RX:
            /* DMA is done transferring data from RXFIFO to receive buffer*/
            
        default:
            break;
    }
}

void SPI1_IRQHandler(void){

}

void DMA_IRQHandler(void){

}

void TIMG12_IRQHandler(void){
    
}