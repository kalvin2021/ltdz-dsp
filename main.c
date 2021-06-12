/*******************************************************************************
 * File Name            : main.c
 * Author               : Kalvin
 * Original Author      : zn007
 * Date First Issued    : 09/30/2018
 * Modified             : 2021/05/29
 * Description          : Main program body  8mhz cy
 ********************************************************************************/

/* Includes ------------------------------------------------------------------*/

#include "stm32f10x.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>

/* -------------------------------------------------------------------------- */

/** Default serial port baud rate */
#define CONFIG_SERIAL_BAUD              (57600)

/**
 * Enable/disable AD8307 LOG/RMS-detector implementation.
 *
 * Set to 1 for original hardware implementation using AD8307 LOG-detector.
 * Set to 0 for new ADC+DSP implementation.
 */
#define CONFIG_AD8307_ENABLE            0   /*< 0 | 1 */

/** ADC sample buffer length 2**N */
#define CONFIG_RX_ADC_BUFFER_SIZE       (4096)

/** Enable/disable ADF4351 low spur mode */
#define CONFIG_ADF4351_LOW_SPUR_ENABLE  0   /*< 0: Low noise | 1: Low spur */

/** Tracking generator default output power level (for good linearity/dynamic range) */
#define CONFIG_TG_ADF4351_OUTPUT_LEVEL  (ADF4351_OUTPUT_LEVEL_MIN)

/** RX mixer LO driving power level */
#define CONFIG_RX_ADF4351_OUTPUT_LEVEL  (ADF4351_OUTPUT_LEVEL_MIN)

/** ADF4351 PLL lock wait time in microseconds typ. 500us - 1000us */
#define CONFIG_ADF4351_PLL_LOCK_TIME_us (750)

/** Firmware extra version info */
#define FIRMWARE_EXTRAVER               10

/* -------------------------------------------------------------------------- */

#if CONFIG_AD8307_ENABLE
/** Default RX LO frequency offset in Hz in NA mode when using AD8307 */
#define RX_DEFAULT_NA_MODE_FREQ_OFFSET_Hz   (120000L)
/** Wait time after PLL lock */
#define CONFIG_SWEEP_WAITTIME_us            (50)
#else
/** Default RX LO frequency offset in Hz in NA mode when using new ADC dB mode */
#define RX_DEFAULT_NA_MODE_FREQ_OFFSET_Hz   (10*8000L)
/** Wait time after PLL lock */
#define CONFIG_SWEEP_WAITTIME_us            (0)
#endif

/** Default RX LO frequency offset / 10 in network analyzer (NA) mode */
#define RX_DEFAULT_NA_FREQ_OFFSET_DIV_10    (RX_DEFAULT_NA_MODE_FREQ_OFFSET_Hz / 10)

/* -------------------------------------------------------------------------. */

/**
 * Busy RED LED
 */
#define BUSY_LED_PORT   (GPIOB)
#define BUSY_LED_PIN    (GPIO_Pin_1)

#define BUSY_LED_DISABLE() \
    do { BUSY_LED_PORT->BRR = BUSY_LED_PIN; } while (0)

#define BUSY_LED_ENABLE() \
    do { BUSY_LED_PORT->BSRR = BUSY_LED_PIN; } while (0)

#define BUSY_LED_TOGGLE() \
    do { BUSY_LED_PORT->ODR ^= BUSY_LED_PIN; } while (0)

/**
 * Tracking generator status BLUE LED
 */
#define TG_LED_PORT   (GPIOA)
#define TG_LED_PIN    (GPIO_Pin_11)

#define TG_LED_DISABLE() \
    do { TG_LED_PORT->BRR = TG_LED_PIN; } while (0)

#define TG_LED_ENABLE() \
    do { TG_LED_PORT->BSRR = TG_LED_PIN; } while (0)

/**
 * Tracking generator enable/disable push-button GPIO pin
 */
#define TG_BUTTON_PORT      (GPIOA)
#define TG_BUTTON_PIN       (GPIO_Pin_12)

#define TG_BUTTON_STATE()   ((TG_BUTTON_PORT->IDR & TG_BUTTON_PIN) ? 1 : 0)

/**
 * ADF4351 TG shift register load enable (loads on 0->1 transition)
 */
#define ADF4351_TG_LE_PORT  (GPIOB)
#define ADF4351_TG_LE_PIN   (GPIO_Pin_12)

#define ADF4351_TG_LE_CLR() \
    do { ADF4351_TG_LE_PORT->BRR = ADF4351_TG_LE_PIN; } while (0)

#define ADF4351_TG_LE_SET() \
    do { ADF4351_TG_LE_PORT->BSRR = ADF4351_TG_LE_PIN; } while (0)

/**
 * ADF4351 RX shift register load enable (loads on 0->1 transition)
 */
#define ADF4351_RX_LE_PORT  (GPIOA)
#define ADF4351_RX_LE_PIN   (GPIO_Pin_4)

#define ADF4351_RX_LE_CLR() \
    do { ADF4351_RX_LE_PORT->BRR = ADF4351_RX_LE_PIN; } while (0)

#define ADF4351_RX_LE_SET() \
    do { ADF4351_RX_LE_PORT->BSRR = ADF4351_RX_LE_PIN; } while (0)

/**
 * RBW-filter ADC input channel
 */
#define ADC_RBW_PORT            (GPIOA)
#define ADC_RBW_PIN             (GPIO_Pin_2)
#define ADC_RBW_ADC_CHANNEL     (2)

/**
 * AD8307 LOG/RMS-detector ADC input channel
 */
#define ADC_AD8307_PORT         (GPIOA)
#define ADC_AD8307_PIN          (GPIO_Pin_3)
#define ADC_AD8307_ADC_CHANNEL  (3)

/* -------------------------------------------------------------------------. */

/**
 * Serial port command identifiers
 */
enum
{
    /* Start of the command sequence */
    COMMAND_PREFIX = 0x8f,

    /* Firmware version query */
    COMMAND_VERSION = 'v',

    /* Firmware extra version number */
    COMMAND_EXTRAVER  = 's',

    /* Read the signal level in dB */
    COMMAND_READ_LEVEL  = 'm',

    /* Set the TG output frequency */
    COMMAND_SET_TG_FREQ = 'f',

    /* Single sweep */
    COMMAND_SINGLE_SWEEP = 'x',

    /* Continuous sweep */
    COMMAND_CONTINUOUS  = 'a',

    /* Single sweep with ADC sample buffer output */
    COMMAND_SINGLE_SWEEP_ADC_BUFFER  = 'b',
};

/** Serial port command buffer size */
#define COMMAND_BUFFER_SIZE (26)

/** Serial port command buffer */
volatile uint8_t command_buffer[COMMAND_BUFFER_SIZE];

/** Serial port command  length */
volatile uint8_t command_length = 0;

/** Set true when COMMAND_VERSION received */
volatile uint8_t command_flag_version = 0;

/** Set true when COMMAND_EXTRAVER received */
volatile uint8_t command_flag_extraver = 0;

/* Set true when COMMAND_READ_LEVEL received */
volatile uint8_t command_flag_read_level = 0;

/* -------------------------------------------------------------------------- */

/**
 * Read an unsigned numeric ASCII value of the given length from the command buffer.
 */
static uint32_t command_read_unsigned(const volatile uint8_t* buf, unsigned length)
{
    uint32_t value = 0;
    for (unsigned n = 0; n < length; n++)
    {
        value = value * 10;
        value += buf[n] - '0';
    }
    return value;
}

/**
 * Read a frequency value from the given location of the command buffer.
 */
static uint32_t command_read_frequency(const volatile uint8_t* buf)
{
    return command_read_unsigned(buf, 9);
}

/**
 * Read a frequency step size from the given location of the command buffer.
 */
static uint32_t command_read_step_size(const volatile uint8_t* buf)
{
    return command_read_unsigned(buf, 6);
}

/**
 * Read a sweep step count from the given location of the command buffer.
 */
static unsigned command_read_step_count(const volatile uint8_t* buf)
{
    return command_read_unsigned(buf, 4);
}

/**
 * Read a sweep step pause value from the given location of the command buffer.
 */
static unsigned command_read_step_pause_ms(const volatile uint8_t* buf)
{
    return command_read_unsigned(buf, 3);
}

/**
 * Reset the command buffer
 */
static void command_buffer_clear(void)
{
    command_length = 0;
    memset((uint8_t*)command_buffer, 0, sizeof(command_buffer));
}

/* -------------------------------------------------------------------------- */

/**
 * Delay for N microseconds
 */
static void delay_us(unsigned N)
{
    const uint32_t delay_cycle_count = (SystemCoreClock / 1000000L) * N;
    DWT->CYCCNT = 0;
    __ASM volatile ("NOP");
    __ASM volatile ("NOP");
    __ASM volatile ("NOP");
    while (DWT->CYCCNT < delay_cycle_count)
    {
        /* wait */
    }
}

/**
 * Delay for N milliseconds
 */
static void delay_ms(unsigned N)
{
    const uint32_t delay_cycle_count = (SystemCoreClock / 1000L) * N;
    DWT->CYCCNT = 0;
    __ASM volatile ("NOP");
    __ASM volatile ("NOP");
    __ASM volatile ("NOP");
    while (DWT->CYCCNT < delay_cycle_count)
    {
        /* wait */
    }
}

/**
 * Init the DWT delay utility
 */
void delay_init(void)
{
    CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk;
    CoreDebug->DEMCR |=  CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;
    DWT->CTRL |=  DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0;
}

/* -------------------------------------------------------------------------- */

/** ADF4351 minimum valid frequency / 10 */
#define ADF4351_MIN_FREQ_DIV_10     (3200000UL)

/** ADF4351 maximum valid frequency / 10 */
#define ADF4351_MAX_FREQ_DIV_10     (440000000UL)

/** ADF4351 reference oscillator frequency */
#define ADF4351_REF_Hz              (25000000UL)
#define ADF4351_REF_DIV_10          ((ADF4351_REF_Hz) / 10)

/** ADF4351 frequency ranges and step sizes */
#define ADF4351_RANGE_1_MIN_DIV_10  (ADF4351_MIN_FREQ_DIV_10)
#define ADF4351_RANGE_1_STEP_Hz     (125)
#define ADF4351_RANGE_2_MIN_DIV_10  (6875000UL)
#define ADF4351_RANGE_2_STEP_Hz     (250)
#define ADF4351_RANGE_3_MIN_DIV_10  (13750000UL)
#define ADF4351_RANGE_3_STEP_Hz     (500)
#define ADF4351_RANGE_4_MIN_DIV_10  (27500000UL)
#define ADF4351_RANGE_4_STEP_Hz     (1000)
#define ADF4351_RANGE_5_MIN_DIV_10  (55000000UL)
#define ADF4351_RANGE_5_STEP_Hz     (2000)
#define ADF4351_RANGE_6_MIN_DIV_10  (110000000UL)
#define ADF4351_RANGE_6_STEP_Hz     (4000)
#define ADF4351_RANGE_7_MIN_DIV_10  (220000000UL)
#define ADF4351_RANGE_7_STEP_Hz     (8000)
#define ADF4351_RANGE_7_MAX_DIV_10  (ADF4351_MAX_FREQ_DIV_10)

/** ADF4351 PLL locktime in microseconds */
#define ADF4351_PLL_LOCK_TIME_us    (CONFIG_ADF4351_PLL_LOCK_TIME_us)

/** Typedef for ADF4351 frequencies */
typedef uint32_t adf4351_freq_div_10_t;

/** Typedef for ADF4351 offset frequencies */
typedef int32_t adf4351_freq_offset_div_10_t;

/** Returns true if the given frequency is a valid ADF4351 frequency */
static bool adf4351_is_valid_freq(adf4351_freq_div_10_t freq_div_10)
{
    return (ADF4351_MIN_FREQ_DIV_10 <= freq_div_10)
           && (freq_div_10 <= ADF4351_MAX_FREQ_DIV_10);
}

/** Returns true if the given frequency is not a valid ADF4351 frequency */
static bool adf4351_is_unlocked(adf4351_freq_div_10_t freq_div_10)
{
    return !adf4351_is_valid_freq(freq_div_10);
}

/**
 * Returns ADF4351 register configuration for the given frequency.
 */

/** ADF4351 register configuration */
typedef struct adf4351_reg_config_t
{
    /** Set true when ADF4351 register configuration is valid */
    bool valid;

    /** ADF4351 register R0 value */
    uint32_t r0;

    /** ADF4351 register R4 value */
    uint32_t r4;
}
adf4351_reg_config_t;

/**
 * Returns the ADF4351 register configuration for the given frequency.
 *
 * Flag valid is set to true when the given frequency is a valid frequency.
 * Flag valid is set to false if the given frequency is not a valid frequency.
 */
static adf4351_reg_config_t adf4351_reg_config(adf4351_freq_div_10_t freq_div_10)
{
    adf4351_reg_config_t config = { .valid = true };

    unsigned rf_divider = 0;

    if ((ADF4351_RANGE_1_MIN_DIV_10 <= freq_div_10) && (freq_div_10 < ADF4351_RANGE_2_MIN_DIV_10))
    {
        /* 125Hz step size */
        rf_divider = 6;
    }
    else if ((ADF4351_RANGE_2_MIN_DIV_10 <= freq_div_10) && (freq_div_10 < ADF4351_RANGE_3_MIN_DIV_10))
    {
        /* 250Hz step size */
        rf_divider = 5;
    }
    else if ((ADF4351_RANGE_3_MIN_DIV_10 <= freq_div_10) && (freq_div_10 < ADF4351_RANGE_4_MIN_DIV_10))
    {
        /* 500Hz step size */
        rf_divider = 4;
    }
    else if ((ADF4351_RANGE_4_MIN_DIV_10 <= freq_div_10) && (freq_div_10 < ADF4351_RANGE_5_MIN_DIV_10))
    {
        /* 1kHz step size */
        rf_divider = 3;
    }
    else if ((ADF4351_RANGE_5_MIN_DIV_10 <= freq_div_10) && (freq_div_10 < ADF4351_RANGE_6_MIN_DIV_10))
    {
        /* 2kHz step size */
        rf_divider = 2;
    }
    else if ((ADF4351_RANGE_6_MIN_DIV_10 <= freq_div_10) && (freq_div_10 < ADF4351_RANGE_7_MIN_DIV_10))
    {
        /* 4kHz step size */
        rf_divider = 1;
    }
    else if ((ADF4351_RANGE_7_MIN_DIV_10 <= freq_div_10) && (freq_div_10 < ADF4351_RANGE_7_MAX_DIV_10))
    {
        /* 8kHz step size */
        rf_divider = 0;
    }
    else
    {
        /* Invalid frequency */
        config.valid = false;
    }

    if (config.valid)
    {
        const uint32_t f_int = (freq_div_10 << rf_divider) / ADF4351_REF_DIV_10;
        const uint32_t f_frac = (freq_div_10 << rf_divider) / 800 % 3125;
        config.r0 = (f_int << 15) + (f_frac << 3);
        config.r4 = 0x008C803C | (rf_divider << 20);
    }

    return config;
}

#define ADF4351_R0_FRAC_VALUE_SHIFT     3
#define ADF4351_R0_INT_VALUE_SHIFT      15

#define ADF4351_R2_LOW_NOISE_MODE       0x00000000
#define ADF4351_R2_LOW_SPUR_MODE        0x60000000

#define ADF4351_OUTPUT_LEVEL_MIN        0
#define ADF4351_OUTPUT_LEVEL_MAX        3
#define ADF4351_R4_OUTPUT_LEVEL_SHIFT   3
#define ADF4351_R4_OUTPUT_LEVEL_MASK    0x00000003

#define ADF4351_R4_OUTPUT_ENABLE        1
#define ADF4351_R4_OUTPUT_DISABLE       0
#define ADF4351_R4_OUTPUT_ENABLE_SHIFT  5
#define ADF4351_R4_OUTPUT_ENABLE_MASK   0x00000001

/** ADF4351 noise mode */
#if CONFIG_ADF4351_LOW_SPUR_ENABLE
#define ADF4351_NOISE_MODE  (ADF4351_R2_LOW_SPUR_MODE)
#else
#define ADF4351_NOISE_MODE  (ADF4351_R2_LOW_NOISE_MODE)
#endif

/** Write tracking generator ADF4351 configuration register */
static void adf4351_tg_write_reg(uint32_t reg_value)
{
    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET)
    {
        /* wait until tx buffer is empty */
    }
    SPI_I2S_SendData(SPI2, ((uint8_t) (reg_value >> 24)));
    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET)
    {
        /* wait until tx buffer is empty */
    }
    SPI_I2S_SendData(SPI2, ((uint8_t) (reg_value >> 16)));
    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET)
    {
        /* wait until tx buffer is empty */
    }
    SPI_I2S_SendData(SPI2, ((uint8_t) (reg_value >> 8)));
    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET)
    {
        /* wait until tx buffer is empty */
    }
    SPI_I2S_SendData(SPI2, ((uint8_t) reg_value));
    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET)
    {
        /* wait until tx buffer is empty */
    }
    delay_us(1);
    ADF4351_TG_LE_SET();
    delay_us(1);
    ADF4351_TG_LE_CLR();
}

/** Set tracking generator ADF4351 configuration registers */
static void adf4351_tg_config(uint32_t wr0, uint32_t wr1, uint32_t wr2, uint32_t wr4)
{
    adf4351_tg_write_reg(0x00580005);
    adf4351_tg_write_reg(wr4);
    adf4351_tg_write_reg(0x000004B3);
    adf4351_tg_write_reg(wr2);
    adf4351_tg_write_reg(wr1);
    adf4351_tg_write_reg(wr0);
}

/** Write RX local oscillator ADF4351 configuration register */
static void adf4351_rx_write_reg(uint32_t reg_value)
{
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)
    {
        /* wait until tx buffer is empty */
    }
    SPI_I2S_SendData(SPI1, ((uint8_t) (reg_value >> 24)));
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)
    {
        /* wait until tx buffer is empty */
    }
    SPI_I2S_SendData(SPI1, ((uint8_t) (reg_value >> 16)));
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)
    {
        /* wait until tx buffer is empty */
    }
    SPI_I2S_SendData(SPI1, ((uint8_t) (reg_value >> 8)));
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)
    {
        /* wait until tx buffer is empty */
    }
    SPI_I2S_SendData(SPI1, ((uint8_t) reg_value));
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)
    {
        /* wait until tx buffer is empty */
    }
    delay_us(1);
    ADF4351_RX_LE_SET();
    delay_us(1);
    ADF4351_RX_LE_CLR();
}

/** Set RX local oscillator ADF4351 configuration registers */
static void adf4351_rx_config(uint32_t wr0, uint32_t wr1, uint32_t wr2, uint32_t wr4)
{
    adf4351_rx_write_reg(0x00580005);
    adf4351_rx_write_reg(wr4);
    adf4351_rx_write_reg(0x000004B3);
    adf4351_rx_write_reg(wr2);
    adf4351_rx_write_reg(wr1);
    adf4351_rx_write_reg(wr0);
}

/* -------------------------------------------------------------------------- */

static_assert(CONFIG_TG_ADF4351_OUTPUT_LEVEL >= ADF4351_OUTPUT_LEVEL_MIN,
              "ADF4351 output level outside valid range");

static_assert(CONFIG_TG_ADF4351_OUTPUT_LEVEL <= ADF4351_OUTPUT_LEVEL_MAX,
              "ADF4351 output level outside valid range");

static unsigned tg_output_level = CONFIG_TG_ADF4351_OUTPUT_LEVEL;

/** Disable tracking generator output */
static void tg_output_disable(void)
{
    adf4351_tg_config(0, 0x08008011, 0x00004E62, 0x00EC803C);
}

/** Set tracking generator output frequency */
static void tg_frequency_set(adf4351_freq_div_10_t freq_div_10)
{
    const adf4351_reg_config_t config = adf4351_reg_config(freq_div_10);
    if (config.valid)
    {
        uint32_t r4 = config.r4;
        r4 &= ~(ADF4351_R4_OUTPUT_LEVEL_MASK << ADF4351_R4_OUTPUT_LEVEL_SHIFT);
        r4 |= (tg_output_level & ADF4351_R4_OUTPUT_LEVEL_MASK) << ADF4351_R4_OUTPUT_LEVEL_SHIFT;
        adf4351_tg_config(config.r0, 0x0800E1A9, 0x00004E42 | ADF4351_NOISE_MODE, r4);
    }
    else
    {
        tg_output_disable();
    }
}

/* -------------------------------------------------------------------------- */

static_assert(CONFIG_RX_ADF4351_OUTPUT_LEVEL >= ADF4351_OUTPUT_LEVEL_MIN,
              "ADF4351 output level outside valid range");

static_assert(CONFIG_RX_ADF4351_OUTPUT_LEVEL <= ADF4351_OUTPUT_LEVEL_MAX,
              "ADF4351 output level outside valid range");

/** RX LO default output frequency offset setting in NA mode */
static adf4351_freq_offset_div_10_t rx_na_mode_freq_offset_div_10 = RX_DEFAULT_NA_FREQ_OFFSET_DIV_10;

/** Disable the RX LO */
static void rx_output_disable(void)
{
    adf4351_rx_config(0, 0x08008011, 0x00004E62, 0x00EC803C);
}

/** Set the RX frequency */
static void rx_frequency_set(adf4351_freq_div_10_t freq_div_10)
{
    const adf4351_reg_config_t config = adf4351_reg_config(freq_div_10);
    if (config.valid)
    {
        uint32_t r4 = config.r4;
        r4 &= ~(ADF4351_R4_OUTPUT_LEVEL_MASK << ADF4351_R4_OUTPUT_LEVEL_SHIFT);
        r4 |= (CONFIG_RX_ADF4351_OUTPUT_LEVEL & ADF4351_R4_OUTPUT_LEVEL_MASK) << ADF4351_R4_OUTPUT_LEVEL_SHIFT;
        adf4351_rx_config(config.r0, 0x0800E1A9, 0x00004E42 | ADF4351_NOISE_MODE, config.r4);
    }
    else
    {
        rx_output_disable();
    }
}

/* --------------------------------------------------------------------------*/

/** Write uint8_t value to Uart port */
static void uart_write_u8(uint8_t val)
{
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
    {
        /* wait */
    }
    USART1->DR = (uint8_t) val;
}

/** Write uint16_t value to Uart */
static void uart_write_u16(uint16_t val)
{
    uart_write_u8(val);
    uart_write_u8(val >> 8);
}

/** Write uint32_t value to Uart */
static void uart_write_u32(uint32_t val)
{
    uart_write_u16(val);
    uart_write_u16(val >> 16);
}

/** Uart serial port interrupt handler */
void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        const uint8_t ch = USART_ReceiveData(USART1);
        if (command_length < sizeof(command_buffer))
        {
            command_buffer[command_length++] = ch;
            if (command_buffer[0] == COMMAND_PREFIX)
            {
                switch (command_buffer[1])
                {
                case COMMAND_VERSION:
                {
                    command_flag_version++;
                    command_buffer_clear();
                    break;
                }
                case COMMAND_EXTRAVER:
                {
                    command_flag_extraver++;
                    command_buffer_clear();
                    break;
                }
                case COMMAND_READ_LEVEL:
                {
                    command_flag_read_level++;
                    command_buffer_clear();
                    break;
                }
                case COMMAND_SET_TG_FREQ:
                {
                    break;
                }
                case COMMAND_SINGLE_SWEEP:
                {
                    break;
                }
                case COMMAND_CONTINUOUS:
                {
                    break;
                }
                case COMMAND_SINGLE_SWEEP_ADC_BUFFER:
                {
                    break;
                }
                }
            }
            else
            {
                command_length = 0;
                command_buffer[0] = 0;
                command_buffer[1] = 0;
            }
        }
    }
    USART_ClearITPendingBit(USART1, USART_IT_RXNE);
}

/* --------------------------------------------------------------------------*/

/** AD8307 LOG/RMS read average count */
#define RX_LEVEL_AVERAGE_COUNT    (1)

/** Returns the AD8307 LOG/RMS value */
static int16_t rx_ad8307_level(unsigned average_count)
{
    uint32_t ad8307_level_sum = 0;

    for (unsigned n = 0; n < average_count; n++)
    {
        ADC_SoftwareStartConvCmd(ADC1, ENABLE);

        while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC))
        {
            /* wait */
        }

        ad8307_level_sum += ADC_GetConversionValue(ADC1);
    }

    const uint16_t ad8307_level = ad8307_level_sum / average_count;
    const int16_t dB = (ad8307_level >> 2) - 80;
    return (dB < 0) ? 0 : dB;
}

/**
 * Look-up table for Log2(1+N)*100.
 */
static const int log2_table[256] =
{
#include "ltdz_log2_table.inc"
};

/** Returns 20*LOG10(x)*10 */
static int16_t log_dB(uint64_t x)
{
    int exp = 8;

    /* Make sure that log argument won't be zero */
    x |= 1;

    /* Normalize upwards */
    while (x < 256)
    {
        exp--;
        x <<= 1;
    }

    /* Normalize downwards */
    while (x >= 512)
    {
        exp++;
        x >>= 1;
    }

    int man = log2_table[x & 0xff];
    int dB_x10 = ((602 * exp) + (602 * man / 100)) / 10;
    return dB_x10;
}

/** RX mixer output sample buffer */
static uint16_t rx_adc_buffer[CONFIG_RX_ADC_BUFFER_SIZE];

/** Set true when the sample buffer is full */
volatile bool rx_adc_buffer_ready;

/** Start ADC sample buffer capture using DMA */
static void rx_adc_buffer_capture_start(void)
{
    /** DMA channel configuration */
    static DMA_InitTypeDef DMA_InitStructure =
    {
        .DMA_BufferSize = CONFIG_RX_ADC_BUFFER_SIZE,
        .DMA_DIR = DMA_DIR_PeripheralSRC,
        .DMA_M2M = DMA_M2M_Disable,
        .DMA_MemoryBaseAddr = (uint32_t) rx_adc_buffer,
        .DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord,
        .DMA_MemoryInc = DMA_MemoryInc_Enable,
        .DMA_PeripheralBaseAddr = (uint32_t) (&(ADC1->DR)),
        .DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord,
        .DMA_PeripheralInc = DMA_PeripheralInc_Disable,
        .DMA_Mode = DMA_Mode_Normal,
        .DMA_Priority = DMA_Priority_High,
    };

    /* DMA buffer acquisition in progress */
    rx_adc_buffer_ready = false;

    /* Configure the ADC DMA */
    DMA_DeInit(DMA1_Channel1);
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);
    DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);

    /* Start ADC buffer capture */
    ADC_DMACmd(ADC1, ENABLE);
    DMA_Cmd(DMA1_Channel1, ENABLE);
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

/** Wait until ADC buffer capture is ready */
static void rx_adc_buffer_capture_wait(void)
{
    /* Wait until DMA buffer acquisition is completed */
    while (!rx_adc_buffer_ready)
    {
        /* wait */
    }
}

/** Stop the ADC buffer capture */
static void rx_adc_buffer_capture_stop(void)
{
    /* Stop ADC buffer capture */
    ADC_SoftwareStartConvCmd(ADC1, DISABLE);
    DMA_Cmd(DMA1_Channel1, DISABLE);
    ADC_DMACmd(ADC1, DISABLE);
}

/** Remove the DC offset from the ADC samples */
static void rx_adc_buffer_dc_offset_remove(void)
{
    uint32_t buffer_mean_sum = 0;

    for (uint16_t n = 0; n < CONFIG_RX_ADC_BUFFER_SIZE; n++)
    {
        buffer_mean_sum += rx_adc_buffer[n];
    }

    const unsigned buffer_mean = buffer_mean_sum / CONFIG_RX_ADC_BUFFER_SIZE;

    for (unsigned n = 0; n < CONFIG_RX_ADC_BUFFER_SIZE; n++)
    {
        rx_adc_buffer[n] = (rx_adc_buffer[n] - buffer_mean);
    }
}

/** Fill the RX mixer output sample buffer using DMA */
static void rx_adc_buffer_fill(void)
{
    rx_adc_buffer_capture_start();

    rx_adc_buffer_capture_wait();

    rx_adc_buffer_capture_stop();

    rx_adc_buffer_dc_offset_remove();
}

/** DMA callback interrupt when RX ADC buffer is full */
void DMA1_Channel1_IRQHandler(void)
{
    rx_adc_buffer_ready = true;
    DMA_ClearITPendingBit(DMA1_IT_TC1);
}

/** Reads the RX LO mixer output into ADC buffer and returns signal level in dB */
static int16_t rx_adc_buffer_dB(void)
{
    rx_adc_buffer_fill();

    uint64_t squared_sum = 0;

    for (unsigned n = 0; n < CONFIG_RX_ADC_BUFFER_SIZE; n++)
    {
        const int16_t x = (int16_t) rx_adc_buffer[n];
        squared_sum += ((int32_t)x * x);
    }

    return log_dB(squared_sum / CONFIG_RX_ADC_BUFFER_SIZE);
}

/** Returns the current RX LO signal level in dB */
static uint16_t rx_level_dB(unsigned average_count)
{
    int16_t dB = 0;

    if (CONFIG_AD8307_ENABLE)
    {
        dB = rx_ad8307_level(average_count);
    }
    else
    {
        dB = (rx_adc_buffer_dB() + 660) / 4;
    }

    return (dB > 0) ? dB : 0;
}

/* --------------------------------------------------------------------------*/

/** Set true when tracking generator is disabled */
static bool tg_is_disabled = 1;

/**
 * Sweep configuration
 */
typedef struct sweep_config_t
{
    /** Start frequency / 10 Hz */
    adf4351_freq_div_10_t start_freq_div_10;

    /** Step size / 10 Hz */
    int32_t step_size_div_10;

    /** Number of steps to perform in one sweep */
    unsigned step_count;

    /** Pause in milliseconds after step if > 0 */
    unsigned pause_ms;

    /** Set true when in NA operating mode (ie. TG is enabled) */
    bool mode_na_enable;
}
sweep_config_t;

/**
 * Sweep output process function.
 *
 * @param frequency_div_10 Current frequency / 10.
 * @param rx_offset_div_10 Current RX LO offset (0 if TG is off).
 */
typedef void (*sweep_output_process_t)(adf4351_freq_div_10_t frequency_div_10,
                                       adf4351_freq_offset_div_10_t rx_offset_div_10,
                                       int32_t step_size_div_10);

/**
 * Perform one sweep.
 */
static void sweep(const sweep_config_t* const config,
                  const sweep_output_process_t sweep_output_process)
{
    BUSY_LED_ENABLE();

    for (unsigned n = 0; n < config->step_count; n++)
    {
        const adf4351_freq_div_10_t tg_freq =
            config->start_freq_div_10 + config->step_size_div_10 * n;

        const adf4351_freq_offset_div_10_t rx_offset =
            config->mode_na_enable ? rx_na_mode_freq_offset_div_10 : 0;

        const adf4351_freq_div_10_t rx_freq = tg_freq + rx_offset;

        if (config->mode_na_enable)
        {
            if (adf4351_is_unlocked(tg_freq) || adf4351_is_unlocked(rx_freq))
            {
                tg_output_disable();
                rx_output_disable();
            }
            else
            {
                tg_frequency_set(tg_freq);
                rx_frequency_set(rx_freq);
            }
        }
        else
        {
            tg_output_disable();
            if (adf4351_is_unlocked(rx_freq))
            {
                rx_output_disable();
            }
            else
            {
                rx_frequency_set(rx_freq);
            }
        }

        delay_us(ADF4351_PLL_LOCK_TIME_us + CONFIG_SWEEP_WAITTIME_us);

        sweep_output_process(tg_freq, rx_offset, config->step_size_div_10);

        if (config->pause_ms > 10)
        {
            delay_ms(config->pause_ms / 10);
        }
    }

    BUSY_LED_DISABLE();
}

static void sweep_output_dB(adf4351_freq_div_10_t frequency_div_10,
                            adf4351_freq_offset_div_10_t rx_offset_div_10,
                            int32_t step_size_div_10)
{
    (void) frequency_div_10;
    (void) rx_offset_div_10;
    (void) step_size_div_10;
    uint16_t level = rx_level_dB(RX_LEVEL_AVERAGE_COUNT);
    uart_write_u16(level);
    uart_write_u16(level);
}

static void sweep_output_adc_buffer(adf4351_freq_div_10_t frequency_div_10,
                                    adf4351_freq_offset_div_10_t rx_offset_div_10,
                                    int32_t step_size_div_10)
{
    rx_adc_buffer_fill();

    /* Output current frequency */
    uart_write_u32(frequency_div_10);

    /* Output rx lo offset (set to 0 in spectrum analyzer mode) */
    uart_write_u32(rx_offset_div_10);

    /* Output sweep step size */
    uart_write_u32((uint32_t) step_size_div_10);

    /* Output ADC buffer length */
    uart_write_u16(CONFIG_RX_ADC_BUFFER_SIZE);

    /* Output the ADC buffer samples */
    for (unsigned n = 0; n < CONFIG_RX_ADC_BUFFER_SIZE; n++)
    {
        uart_write_u16(rx_adc_buffer[n]);
    }
}

/* --------------------------------------------------------------------------*/

/** Returns true if the tracking generator button is released */
static bool tg_button_is_released(void)
{
    return TG_BUTTON_STATE();
}

/** Update tracking generator status LED state */
static void tg_status_led_update(bool enabled)
{
    if (enabled)
    {
        TG_LED_ENABLE();
    }
    else
    {
        TG_LED_DISABLE();
    };
}

/* --------------------------------------------------------------------------*/

void RCC_Configuration(void);
void GPIO_Configuration(void);
void NVIC_Configuration(void);
void USART1_Configuration(void);
void Adc_Init(void);
void SPI2_Conf(void);
void SPI1_Conf(void);

/** Sweep parameters */
static sweep_config_t sweep_config = {
    .start_freq_div_10 = ADF4351_MIN_FREQ_DIV_10,
    .step_size_div_10 = 0,
    .step_count = 0,
    .pause_ms = 0,
    .mode_na_enable = false,
};

int main(void)
{
    unsigned long count1 = 0;
    unsigned long cc = 0;

    bool tg_enabled = false;
    bool tg_button_state = 1;

    delay_init();

    RCC_Configuration();
    NVIC_Configuration();
    GPIO_Configuration();
    USART1_Configuration();
    Adc_Init();
    SPI2_Conf();
    SPI1_Conf();

    tg_output_disable();
    rx_output_disable();

    while (1)
    {
        if ((!tg_button_is_released()) && (tg_button_state)) // 判断PINA12端口是否为低电平
        {
            if (cc < 10000)
            {
                delay_us(1);
                cc++;
            } //防抖
            else
            {
                cc = 0;
                tg_enabled = !tg_enabled;
                tg_button_state = 0;
            }
        }

        if (tg_button_is_released())
        {
            tg_button_state = 1;
        }

        tg_status_led_update(tg_enabled);

        if (command_flag_version > 0)
        {
            uart_write_u8(FIRMWARE_VERSION);
            command_flag_version--;
        }
        else if (command_flag_extraver > 0)
        {
            uart_write_u32(FIRMWARE_EXTRAVER);
            command_flag_extraver--;
        }
        else if (command_flag_read_level > 0)
        {
            const uint16_t level = rx_level_dB(RX_LEVEL_AVERAGE_COUNT);
            uart_write_u16(level);
            uart_write_u16(level);
            command_flag_read_level--;
        }
        else if ((command_buffer[0] == COMMAND_PREFIX)
                 && (command_buffer[1] == COMMAND_SET_TG_FREQ)
                 && (command_length == 10))
        {
            const uint32_t tg_freq_div_10 = command_read_frequency(&command_buffer[2]);
            command_buffer_clear();

            tg_is_disabled = adf4351_is_unlocked(tg_freq_div_10);
            if (tg_is_disabled)
            {
                tg_output_disable();
                rx_output_disable();
            }
            else
            {
                tg_frequency_set(tg_freq_div_10);
            }

            BUSY_LED_ENABLE();
        }
        else if ((command_buffer[0] == COMMAND_PREFIX)
                 && ((command_buffer[1] == COMMAND_SINGLE_SWEEP)
                     || (command_buffer[1] == COMMAND_SINGLE_SWEEP_ADC_BUFFER))
                 && (command_length == 23))
        {
            const bool legacy_dB_output = (command_buffer[1] == COMMAND_SINGLE_SWEEP);

            sweep_config.start_freq_div_10 = command_read_frequency(&command_buffer[2]);
            sweep_config.step_size_div_10 = command_read_step_size(&command_buffer[13]);
            sweep_config.step_count = command_read_step_count(&command_buffer[19]);
            sweep_config.pause_ms = 0;
            sweep_config.mode_na_enable = tg_enabled;
            command_buffer_clear();

            sweep(&sweep_config,
                  legacy_dB_output ? sweep_output_dB : sweep_output_adc_buffer);
        }
        else if ((command_buffer[0] == COMMAND_PREFIX)
                 && (command_buffer[1] == COMMAND_CONTINUOUS)
                 && (command_length == 26))
        {
            sweep_config.start_freq_div_10 = command_read_frequency(&command_buffer[2]);
            sweep_config.step_size_div_10 = command_read_step_size(&command_buffer[13]);
            sweep_config.step_count = command_read_step_count(&command_buffer[19]);
            sweep_config.pause_ms = command_read_step_pause_ms(&command_buffer[23]);
            sweep_config.mode_na_enable = tg_enabled;
            command_buffer_clear();

            sweep(&sweep_config, sweep_output_dB);
        }

        if (tg_is_disabled)
        {
            if (count1 < 120000)
            {
                count1++;
            }
            else
            {
                count1 = 0;
                BUSY_LED_TOGGLE();
            }
        }
    }
}

/* --------------------------------------------------------------------------*/

void RCC_Configuration(void)
{
    ErrorStatus HSEStartUpStatus;

    RCC_DeInit(); //时钟管理重置 （复位成缺省值）
    RCC_HSEConfig(RCC_HSE_ON); //打开外部晶振
    HSEStartUpStatus = RCC_WaitForHSEStartUp(); //等待外部晶振就绪
    if (HSEStartUpStatus == SUCCESS)
    {
        RCC_HCLKConfig(RCC_SYSCLK_Div1); // AHB使用系统时钟    72mhz
        RCC_PCLK2Config(RCC_HCLK_Div1); // 72mhz
        RCC_PCLK1Config(RCC_HCLK_Div2); // 36mhz
        FLASH_SetLatency(FLASH_Latency_2);
        FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
        RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9); // HD MD
        RCC_PLLCmd(ENABLE);
        while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET) //等待PLL启动
        {
            /* wait */
        }
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK); //将PLL设置成系统时钟源
        while (RCC_GetSYSCLKSource() != 0x08) //检查是否将HSE_9倍频后作为系统时钟  0x08
            //PLL作为系统时钟，0X04 HSE作为系统时钟 0X00 HSI作为系统时钟
        {
            /* wait */
        }
    }

    /* Configure peripheral clocks */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE); //使能AB时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_ADCCLKConfig(RCC_PCLK2_Div6); // 12MHZ
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
}

/* --------------------------------------------------------------------------*/

void GPIO_Configuration(void) ///初始化要用的端口
{

    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE);

    /* USART1 TX pin */
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* USART1 RX pin */
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* Busy LED pin */
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = BUSY_LED_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(BUSY_LED_PORT, &GPIO_InitStructure);

    /* TG state LED pin */
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = TG_LED_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(TG_LED_PORT, &GPIO_InitStructure);

    /* TG button pin */
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = TG_BUTTON_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(TG_BUTTON_PORT, &GPIO_InitStructure);

    /* PA.3 AD8307 LOG-detector output as analog input */
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = ADC_AD8307_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(ADC_AD8307_PORT, &GPIO_InitStructure);

    /* PA.2 LO mixer RBW-filter output as analog input */
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = ADC_RBW_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(ADC_RBW_PORT, &GPIO_InitStructure);

    /* SPI1 ie. RX LO ADF4351 pins */
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* RX ADF4351 LE pin */
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = ADF4351_RX_LE_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(ADF4351_RX_LE_PORT, &GPIO_InitStructure);
    ADF4351_RX_LE_CLR();

    /* SPI2 ie TG ADF4351 pins */
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* TG ADF4351 LE pin */
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = ADF4351_TG_LE_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(ADF4351_TG_LE_PORT, &GPIO_InitStructure);
    ADF4351_TG_LE_CLR();
}

/* --------------------------------------------------------------------------*/

void NVIC_Configuration(void)
{
#ifdef VECT_TAB_RAM
    /* Set the Vector Table base location at 0x20000000 */
    NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
#else /* VECT_TAB_FLASH  */
    /* Set the Vector Table base location at 0x08000000 */
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
#endif

    /* Config UART1 interrupt handler */
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Config DMA1 channel 1 interrupt handler */
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/* --------------------------------------------------------------------------*/

void USART1_Configuration(void)
{
    USART_ClockInitTypeDef  USART_ClockInitStructure;
    USART_ClockStructInit(&USART_ClockInitStructure);
    USART_ClockInit(USART1, &USART_ClockInitStructure);

    USART_InitTypeDef USART_InitStructure;
    USART_StructInit(&USART_InitStructure);
    USART_InitStructure.USART_BaudRate = CONFIG_SERIAL_BAUD;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    USART_Cmd(USART1, ENABLE);
}

/* --------------------------------------------------------------------------*/

void Adc_Init(void)
{
    ADC_DeInit(ADC1);

    ADC_InitTypeDef ADC_InitStructure;

    ADC_StructInit(&ADC_InitStructure);

#if CONFIG_AD8307_ENABLE
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);
    ADC_RegularChannelConfig(ADC1, ADC_AD8307_ADC_CHANNEL, 1, ADC_SampleTime_239Cycles5);
#else
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Left;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);
    ADC_RegularChannelConfig(ADC1, ADC_RBW_ADC_CHANNEL, 1, ADC_SampleTime_1Cycles5);
#endif

    /* Enable ADC1 */
    ADC_Cmd(ADC1, ENABLE);

    /* Reset ADC1 calibration */
    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1))
    {
        /* wait */
    }

    /* Run ADC1 calibration */
    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1))
    {
        /* wait */
    }
}

/* --------------------------------------------------------------------------*/

void SPI1_Conf(void)
{
    SPI_Cmd(SPI1, DISABLE);

    SPI_InitTypeDef SPI_InitStructure;

    SPI_StructInit(&SPI_InitStructure);
    SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx; //单线TX
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master; //主模式
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b; // 8位宽
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low; //空闲低电平
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge; //奇数边沿
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft; //外部脚管理
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2; //波特率预分频
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB; // MSB
    SPI_InitStructure.SPI_CRCPolynomial = 7; //校验值7忽略校验
    SPI_Init(SPI1, &SPI_InitStructure); /*Enable SPI1.NSS as a GPIO*/

    SPI_Cmd(SPI1, ENABLE); //开启外设1
}

/* --------------------------------------------------------------------------*/

void SPI2_Conf(void)
{
    SPI_Cmd(SPI2, DISABLE);

    SPI_InitTypeDef SPI_InitStructure;
    SPI_StructInit(&SPI_InitStructure);
    SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx; //单线TX
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master; //主模式
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b; // 8位宽
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low; //空闲低电平
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge; //奇数边沿
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft; //外部脚管理
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2; //波特率预分频
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB; // MSB
    SPI_InitStructure.SPI_CRCPolynomial = 7; //校验值7
    SPI_Init(SPI2, &SPI_InitStructure); //

    SPI_Cmd(SPI2, ENABLE); //开启外设2
}

/******************* (C) COPYRIGHT 2007 STMicroelectronics *****END OF FILE****/
