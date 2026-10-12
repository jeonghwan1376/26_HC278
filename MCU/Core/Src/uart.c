#include "uart.h"

#include <string.h>

#define UART_TX_QUEUE_SIZE   16U
#define UART_TX_MSG_SIZE     96U
#define UART_RX_RING_SIZE    256U

static UART_HandleTypeDef *uart_handle = NULL;

static volatile uint8_t rx_ch = 0U;
static volatile uint8_t rx_ring[UART_RX_RING_SIZE];
static volatile uint16_t rx_ring_wr = 0U;
static volatile uint16_t rx_ring_rd = 0U;
static volatile uint8_t rx_ring_overflow = 0U;

static volatile uint8_t uart_tx_busy = 0U;
static volatile uint8_t uart_tx_wr = 0U;
static volatile uint8_t uart_tx_rd = 0U;
static volatile uint8_t uart_tx_count = 0U;
static volatile uint8_t uart_tx_overflow = 0U;

static char uart_tx_queue[UART_TX_QUEUE_SIZE][UART_TX_MSG_SIZE];
static uint16_t uart_tx_len[UART_TX_QUEUE_SIZE];

static char uart_line_buffer[UART_RX_LINE_SIZE];
static size_t uart_line_length = 0U;
static uint8_t uart_line_discarding = 0U;

static void UART_TxKick(void);
static uint8_t UART_RingPop(uint8_t *out);

HAL_StatusTypeDef UART_Comm_Init(UART_HandleTypeDef *huart)
{
    if (huart == NULL)
    {
        return HAL_ERROR;
    }

    uart_handle = huart;

    rx_ring_wr = 0U;
    rx_ring_rd = 0U;
    rx_ring_overflow = 0U;

    uart_tx_busy = 0U;
    uart_tx_wr = 0U;
    uart_tx_rd = 0U;
    uart_tx_count = 0U;
    uart_tx_overflow = 0U;

    uart_line_length = 0U;
    uart_line_discarding = 0U;
    memset(uart_line_buffer, 0, sizeof(uart_line_buffer));

    return HAL_UART_Receive_IT(uart_handle, (uint8_t *)&rx_ch, 1U);
}

HAL_StatusTypeDef UART_Send_IT(const char *msg)
{
    uint16_t len;
    uint32_t primask;

    if ((uart_handle == NULL) || (msg == NULL))
    {
        return HAL_ERROR;
    }

    len = (uint16_t)strlen(msg);
    if (len == 0U)
    {
        return HAL_ERROR;
    }

    if (len >= UART_TX_MSG_SIZE)
    {
        len = UART_TX_MSG_SIZE - 1U;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    if (uart_tx_count >= UART_TX_QUEUE_SIZE)
    {
        uart_tx_overflow = 1U;
        __set_PRIMASK(primask);
        return HAL_BUSY;
    }

    memcpy(uart_tx_queue[uart_tx_wr], msg, len);
    uart_tx_queue[uart_tx_wr][len] = '\0';
    uart_tx_len[uart_tx_wr] = len;

    uart_tx_wr++;
    if (uart_tx_wr >= UART_TX_QUEUE_SIZE)
    {
        uart_tx_wr = 0U;
    }
    uart_tx_count++;

    __set_PRIMASK(primask);

    UART_TxKick();
    return HAL_OK;
}

void UART_SendMessage(const char *msg)
{
    (void)UART_Send_IT(msg);
}

uint8_t UART_GetLine(char *out_line, size_t out_size)
{
    uint8_t ch;

    if ((out_line == NULL) || (out_size < 2U))
    {
        return 0U;
    }

    while (UART_RingPop(&ch) != 0U)
    {
        if ((ch == '\r') || (ch == '\n'))
        {
            if (uart_line_discarding != 0U)
            {
                uart_line_discarding = 0U;
                uart_line_length = 0U;
                continue;
            }

            if (uart_line_length == 0U)
            {
                /* CRLF의 두 번째 종료 문자처럼 빈 줄은 무시 */
                continue;
            }

            if (uart_line_length >= out_size)
            {
                uart_line_length = 0U;
                return 0U;
            }

            memcpy(out_line, uart_line_buffer, uart_line_length);
            out_line[uart_line_length] = '\0';
            uart_line_length = 0U;
            return 1U;
        }

        if (uart_line_discarding != 0U)
        {
            continue;
        }

        if (uart_line_length < (sizeof(uart_line_buffer) - 1U))
        {
            uart_line_buffer[uart_line_length++] = (char)ch;
        }
        else
        {
            /* 너무 긴 한 줄은 종료 문자가 올 때까지 폐기 */
            uart_line_discarding = 1U;
            uart_line_length = 0U;
            rx_ring_overflow = 1U;
        }
    }

    return 0U;
}

void UART_Process(void)
{
    UART_TxKick();
}

uint8_t UART_GetRxOverflow(void)
{
    return rx_ring_overflow;
}

void UART_ClearRxOverflow(void)
{
    rx_ring_overflow = 0U;
}

uint8_t UART_GetTxOverflow(void)
{
    return uart_tx_overflow;
}

void UART_ClearTxOverflow(void)
{
    uart_tx_overflow = 0U;
}

static uint8_t UART_RingPop(uint8_t *out)
{
    uint8_t result = 0U;
    uint32_t primask;

    if (out == NULL)
    {
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    if (rx_ring_rd != rx_ring_wr)
    {
        *out = rx_ring[rx_ring_rd];
        rx_ring_rd++;
        if (rx_ring_rd >= UART_RX_RING_SIZE)
        {
            rx_ring_rd = 0U;
        }
        result = 1U;
    }

    __set_PRIMASK(primask);
    return result;
}

static void UART_TxKick(void)
{
    char *msg = NULL;
    uint16_t len = 0U;
    uint32_t primask;

    if (uart_handle == NULL)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    if ((uart_tx_busy == 0U) && (uart_tx_count > 0U))
    {
        uart_tx_busy = 1U;
        msg = uart_tx_queue[uart_tx_rd];
        len = uart_tx_len[uart_tx_rd];
    }

    __set_PRIMASK(primask);

    if (msg != NULL)
    {
        if (HAL_UART_Transmit_IT(uart_handle, (uint8_t *)msg, len) != HAL_OK)
        {
            primask = __get_PRIMASK();
            __disable_irq();
            uart_tx_busy = 0U;
            __set_PRIMASK(primask);
        }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((uart_handle != NULL) && (huart == uart_handle))
    {
        uint16_t next = rx_ring_wr + 1U;

        if (next >= UART_RX_RING_SIZE)
        {
            next = 0U;
        }

        if (next != rx_ring_rd)
        {
            rx_ring[rx_ring_wr] = rx_ch;
            rx_ring_wr = next;
        }
        else
        {
            rx_ring_overflow = 1U;
        }

        (void)HAL_UART_Receive_IT(uart_handle, (uint8_t *)&rx_ch, 1U);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((uart_handle != NULL) && (huart == uart_handle))
    {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();

        if (uart_tx_count > 0U)
        {
            uart_tx_rd++;
            if (uart_tx_rd >= UART_TX_QUEUE_SIZE)
            {
                uart_tx_rd = 0U;
            }
            uart_tx_count--;
        }

        uart_tx_busy = 0U;
        __set_PRIMASK(primask);

        UART_TxKick();
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if ((uart_handle != NULL) && (huart == uart_handle))
    {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        uart_tx_busy = 0U;
        __set_PRIMASK(primask);

        (void)HAL_UART_Receive_IT(uart_handle, (uint8_t *)&rx_ch, 1U);
    }
}
