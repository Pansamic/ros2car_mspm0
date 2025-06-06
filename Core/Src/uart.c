#include <stdarg.h>
#include "lwprintf.h"
#include "uart.h"

uart_cb_t uart_cb[3];

uint8_t uart_tx_buffer[3][128];
uint8_t uart_rx_buffer[3][128];

uint8_t uart_printf_buffer[128];

static void clear_uart_printf_buffer(void){
    for(uint32_t* p=(uint32_t*)uart_printf_buffer; p<(uint32_t*)(uart_printf_buffer+sizeof(uart_printf_buffer)); p++) *p = 0;
}

void uart_init(void){
    uart_cb[0].huart = UART_0_INST;
    uart_cb[0].dma_rx_channel = DMA_UART0_RX_CHAN_ID;
    uart_cb[0].dma_tx_channel = DMA_UART0_TX_CHAN_ID;
    uart_cb[1].huart = UART_1_INST;
    uart_cb[1].dma_rx_channel = DMA_UART1_RX_CHAN_ID;
    uart_cb[1].dma_tx_channel = DMA_UART1_TX_CHAN_ID;
    uart_cb[2].huart = UART_2_INST;
    uart_cb[2].dma_rx_channel = DMA_UART2_RX_CHAN_ID;
    uart_cb[2].dma_tx_channel = DMA_UART2_TX_CHAN_ID;
    for (int i=0; i<3; i++){
        uart_cb[i].expect_dma_rx_len = 0;
        uart_cb[i].expect_dma_tx_len = 0;
        ringbuf_init(&uart_cb[i].rx_ringbuf, uart_rx_buffer[i], sizeof(uart_rx_buffer[i]), RINGBUF_RULE_OVERWRITE);
        ringbuf_init(&uart_cb[i].tx_ringbuf, uart_tx_buffer[i], sizeof(uart_tx_buffer[i]), RINGBUF_RULE_OVERWRITE);
        uart_cb[i].tx_cplt_flag = 0;
    }
}

void uart_send(uart_cb_t* pcb, uint8_t* data, size_t len){

    // /* Configure DMA source, destination and size */
    // void* pdata;
    // uint16_t dma_rest_size;
    // size_t transfer_length;
    // DL_DMA_disableChannel(DMA, pcb->dma_tx_channel);
    // dma_rest_size = DL_DMA_getTransferSize(DMA, pcb->dma_tx_channel);

    // (void)ringbuf_get_stuffed_continuous_block(&pcb->tx_ringbuf, &pdata, &transfer_length);
    // DL_DMA_setSrcAddr(DMA, pcb->dma_tx_channel, (uint32_t)pdata);
    // DL_DMA_setDestAddr(DMA, pcb->dma_tx_channel, (uint32_t)(&pcb->huart->TXDATA));
    // DL_DMA_setTransferSize(DMA, pcb->dma_tx_channel, transfer_length);
    // /*
    //  * The UART DMA TX interrupt is set indicating the UART is ready to
    //  * transmit data, so enabling the DMA will start the transfer
    //  */
    // DL_DMA_enableChannel(DMA, pcb->dma_tx_channel);
    for(size_t i = 0; i < len; i++){
        while(DL_UART_isBusy(pcb->huart));
        DL_UART_transmitData(pcb->huart, data[i]);
    }
}

void uart_read(uart_cb_t* pcb, uint8_t* data, size_t len){

}

void uart0_send(uint8_t* data, size_t len){
    uart_send(&uart_cb[0], data, len);
}

void uart0_read(uint8_t* data, size_t len){
    uart_read(&uart_cb[0], data, len);
}

void uart1_send(uint8_t* data, size_t len){
    uart_send(&uart_cb[1], data, len);
}

void uart1_read(uint8_t* data, size_t len){
    uart_read(&uart_cb[1], data, len);
}

void uart2_send(uint8_t* data, size_t len){
    uart_send(&uart_cb[2], data, len);
}

void uart2_read(uint8_t* data, size_t len){
    uart_read(&uart_cb[2], data, len);
}

void uart0_printf(const char* fmt, ...){
    va_list args;
    clear_uart_printf_buffer();
    va_start(args, fmt);
    lwvsnprintf((char*)uart_printf_buffer, sizeof(uart_printf_buffer), fmt, args);
    va_end(args);
    uart0_send(uart_printf_buffer, strlen((char*)uart_printf_buffer));
}

void uart1_printf(const char* fmt, ...){
    va_list args;
    clear_uart_printf_buffer();
    va_start(args, fmt);
    lwvsnprintf((char*)uart_printf_buffer, sizeof(uart_printf_buffer), fmt, args);
    va_end(args);
    uart1_send(uart_printf_buffer, strlen((char*)uart_printf_buffer));
}

void uart2_printf(const char* fmt, ...){
    va_list args;
    clear_uart_printf_buffer();
    va_start(args, fmt);
    lwvsnprintf((char*)uart_printf_buffer, sizeof(uart_printf_buffer), fmt, args);
    va_end(args);
    uart2_send(uart_printf_buffer, strlen((char*)uart_printf_buffer));
}

