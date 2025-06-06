#ifndef __UART_H__
#define __UART_H__

#include <stdint.h>
#include "cRingbuf.h"
#include "ti_msp_dl_config.h"

typedef struct {
    UART_Regs* huart;
    uint32_t dma_rx_channel;
    uint32_t dma_tx_channel;
    size_t expect_dma_rx_len;
    size_t expect_dma_tx_len;
    ringbuf_t rx_ringbuf;
    ringbuf_t tx_ringbuf;
    volatile uint8_t tx_cplt_flag;
} uart_cb_t;

void uart_init(void);
void uart0_send(uint8_t* data, size_t len);
void uart0_read(uint8_t* data, size_t len);
void uart1_send(uint8_t* data, size_t len);
void uart1_read(uint8_t* data, size_t len);
void uart2_send(uint8_t* data, size_t len);
void uart2_read(uint8_t* data, size_t len);
void uart0_printf(const char* fmt, ...);
void uart1_printf(const char* fmt, ...);
void uart2_printf(const char* fmt, ...);
#endif