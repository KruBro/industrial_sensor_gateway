#include "main.h"
#include "lcdi2c.h"
#include "modbus.h"
#include <stdio.h>
#include <string.h>

#define DEBOUNCE_MS            200U
#define LOOP_DELAY_MS          20U
#define RX_BUF_SIZE            256U
#define RX_DEBUG               0

#define MB_REQ_LEN             8U
#define MB_FRAME_TIMEOUT_MS    1000U

ADC_HandleTypeDef hadc1;
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart1;

volatile uint16_t adc_values[4];
volatile uint8_t valve_state = 0;

uint8_t rx_byte;
volatile uint8_t rx_buf[RX_BUF_SIZE];
volatile uint16_t rx_len = 0;
volatile uint32_t last_rx_tick = 0;
volatile uint8_t rx_overflow = 0;
volatile uint8_t rx_error = 0;

volatile uint16_t dbg_err = 0;
volatile uint8_t  dbg_code = 0;
volatile uint8_t  dbg_sr = 0;
volatile uint16_t dbg_drop = 0;
volatile uint8_t  dbg_snap[5];
volatile uint16_t dbg_snap_len = 0;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);

static uint16_t mbport_get_frame(uint8_t *dst, uint16_t max);

static uint16_t ADC_Read_Channel(uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;

    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);

    uint16_t value = HAL_ADC_GetValue(&hadc1);

    HAL_ADC_Stop(&hadc1);

    return value;
}

static void LCD_UpdateLine(uint8_t row, const char *text, char *shown)
{
    char line[17];

    snprintf(line, sizeof(line), "%-16s", text);

    if (strcmp(line, shown) != 0)
    {
        LCD_SetCursor(row, 0);
        LCD_Print(line);
        strcpy(shown, line);
    }
}

#if RX_DEBUG
static void hex5(char *out, const volatile uint8_t *b, uint16_t len)
{
    static const char digits[] = "0123456789ABCDEF";

    for (uint8_t i = 0; i < 5; i++)
    {
        if (i < len)
        {
            out[2 * i]     = digits[(b[i] >> 4) & 0x0F];
            out[2 * i + 1] = digits[b[i] & 0x0F];
        }
        else
        {
            out[2 * i]     = '-';
            out[2 * i + 1] = '-';
        }
    }
    out[10] = '\0';
}
#endif

int main(void)
{
    char text[32];
    char shown0[17] = "";
    char shown1[17] = "";
    uint8_t frame[MB_REQ_LEN];
    uint8_t reply[32];

    HAL_Init();

    SystemClock_Config();

    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_USART1_UART_Init();
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    MX_I2C1_Init();

    LCD_Init(&hi2c1);
    LCD_Clear();

    while (1)
    {
        adc_values[0] = ADC_Read_Channel(ADC_CHANNEL_0);
        adc_values[1] = ADC_Read_Channel(ADC_CHANNEL_1);
        adc_values[2] = ADC_Read_Channel(ADC_CHANNEL_2);

        uint16_t pressure    = (uint16_t)((adc_values[0] * 4000UL)  / 4095);
        uint16_t flow        = (uint16_t)((adc_values[1] * 10000UL) / 4095);
        uint16_t temperature = (uint16_t)((adc_values[2] * 1500UL)  / 4095);

        regs[0] = pressure;
        regs[1] = flow;
        regs[2] = temperature;
        regs[3] = valve_state;

#if RX_DEBUG
        char hx[11];
        uint16_t len = rx_len;
        uint16_t slen = dbg_snap_len;

        hex5(hx, dbg_snap, slen);
        snprintf(text, sizeof(text), "X%u %s", (unsigned)slen, hx);
        LCD_UpdateLine(0, text, shown0);

        hex5(hx, rx_buf, len);
        snprintf(text, sizeof(text), "N%u %s", (unsigned)len, hx);
        LCD_UpdateLine(1, text, shown1);
#else
        snprintf(text, sizeof(text), "P:%u.%u F:%u.%u",
                 (unsigned)(pressure / 10), (unsigned)(pressure % 10),
                 (unsigned)(flow / 10),     (unsigned)(flow % 10));
        LCD_UpdateLine(0, text, shown0);

        snprintf(text, sizeof(text), "T:%u.%u V:%s",
                 (unsigned)(temperature / 10), (unsigned)(temperature % 10),
                 valve_state ? "OPEN" : "CLOSED");
        LCD_UpdateLine(1, text, shown1);
#endif

        uint16_t n = mbport_get_frame(frame, sizeof(frame));
        if (n != 0)
        {
            uint16_t r = mb_handle_request(frame, n, reply);
            if (r != 0)
            {
                HAL_UART_Transmit(&huart1, reply, r, 100);
            }
        }

        HAL_Delay(LOOP_DELAY_MS);
    }
}

static void rx_drop(uint16_t count)
{
    __disable_irq();

    uint16_t rest = rx_len - count;
    for (uint16_t i = 0; i < rest; i++)
    {
        rx_buf[i] = rx_buf[count + i];
    }
    rx_len = rest;

    __enable_irq();
}

static uint16_t mbport_get_frame(uint8_t *dst, uint16_t max)
{
    if (max < MB_REQ_LEN)
    {
        return 0;
    }

    for (;;)
    {
        uint16_t len;

        __disable_irq();

        if (rx_overflow)
        {
            rx_overflow = 0;
            rx_error = 0;
            rx_len = 0;
            __enable_irq();
            return 0;
        }

        rx_error = 0;
        len = rx_len;

        if (len >= MB_REQ_LEN)
        {
            for (uint16_t i = 0; i < MB_REQ_LEN; i++)
            {
                dst[i] = rx_buf[i];
            }
        }
        else if (len > 0 && (HAL_GetTick() - last_rx_tick) > MB_FRAME_TIMEOUT_MS)
        {
            for (uint8_t i = 0; i < 5; i++)
            {
                dbg_snap[i] = (i < len) ? rx_buf[i] : 0;
            }
            dbg_snap_len = len;
            rx_len = 0;
        }

        __enable_irq();

        if (len < MB_REQ_LEN)
        {
            return 0;
        }

        if (modbus_validate_frame(dst, MB_REQ_LEN) == MB_OK)
        {
            rx_drop(MB_REQ_LEN);
            return MB_REQ_LEN;
        }

        for (uint8_t i = 0; i < 5; i++)
        {
            dbg_snap[i] = dst[i];
        }
        dbg_snap_len = len;
        dbg_drop++;
        rx_drop(1);
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    static uint32_t last_tick = 0;

    if (GPIO_Pin == GPIO_PIN_0)
    {
        uint32_t now = HAL_GetTick();

        if ((now - last_tick) > DEBOUNCE_MS)
        {
            valve_state = !valve_state;
            last_tick = now;
        }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        if (rx_len < RX_BUF_SIZE)
        {
            rx_buf[rx_len] = rx_byte;
            rx_len++;
        }
        else
        {
            rx_overflow = 1;
        }

        last_rx_tick = HAL_GetTick();

        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        rx_error = 1;
        dbg_err++;
        dbg_code |= (uint8_t)huart->ErrorCode;
        dbg_sr = (uint8_t)huart->Instance->SR;
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK |
                                   RCC_CLOCKTYPE_SYSCLK |
                                   RCC_CLOCKTYPE_PCLK1 |
                                   RCC_CLOCKTYPE_PCLK2;

    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;

    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }
}

static void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    hadc1.Instance = ADC1;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;

    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }

    sConfig.Channel = ADC_CHANNEL_0;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

static void MX_I2C1_Init(void)
{
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        Error_Handler();
    }
}

static void MX_USART1_UART_Init(void)
{
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 9600;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

void Error_Handler(void)
{
    __disable_irq();

    while (1)
    {
    }
}
