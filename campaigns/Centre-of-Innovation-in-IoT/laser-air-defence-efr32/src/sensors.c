/* ================================================================
   sensors.c  â€“  Sensor drivers + fusion
   MLX90640 (I2C), HC-SR04 (GPIO timer), HB100 radar (UART/ADC)
   ================================================================ */
#include "sensors.h"
#include "config.h"
#include "em_gpio.h"
#include "em_i2c.h"
#include "em_usart.h"
#include "em_adc.h"
#include "em_cmu.h"
#include "em_timer.h"
#include <string.h>
#include <math.h>
 
#define MLX_ADDR            0x33
#define MLX_REG_RAM         0x0400
#define MLX_FRAME_WORDS     834
#define MLX_ROWS            24
#define MLX_COLS            32

/* Raw frame buffer (2 bytes Ã— 834 words) */
static uint16_t s_mlx_frame[MLX_FRAME_WORDS];
 
#define US_SOUND_CM_US      0.0343f   /* cm per microsecond       */
#define US_TIMEOUT_US       30000u    /* 300 cm max               */
 
static void     i2c_init_sensor(void);
static void     uart_radar_init(void);
static void     adc_radar_init(void);
static uint32_t us_pulse_us(GPIO_Port_TypeDef trig_port, unsigned trig_pin,
                             GPIO_Port_TypeDef echo_port, unsigned echo_pin);

/* ----------------------------------------------------------------
   sensors_init â€“ configure all peripherals
   ---------------------------------------------------------------- */
void sensors_init(void)
{
    CMU_ClockEnable(cmuClock_GPIO,  true);
    CMU_ClockEnable(cmuClock_I2C0,  true);
    CMU_ClockEnable(cmuClock_USART0, true);
    CMU_ClockEnable(cmuClock_ADC0,  true);

    /* Ultrasonic TRIG pins â€“ output */
    GPIO_PinModeSet(US1_TRIG_PORT, US1_TRIG_PIN, gpioModePushPull, 0);
    GPIO_PinModeSet(US2_TRIG_PORT, US2_TRIG_PIN, gpioModePushPull, 0);

    /* Ultrasonic ECHO pins â€“ input, filtered */
    GPIO_PinModeSet(US1_ECHO_PORT, US1_ECHO_PIN, gpioModeInputPullFilter, 0);
    GPIO_PinModeSet(US2_ECHO_PORT, US2_ECHO_PIN, gpioModeInputPullFilter, 0);

    i2c_init_sensor();
    uart_radar_init();
    adc_radar_init();
}
 
static void i2c_init_sensor(void)
{
    GPIO_PinModeSet(IR_I2C_PORT, IR_SDA_PIN, gpioModeWiredAndPullUpFilter, 1);
    GPIO_PinModeSet(IR_I2C_PORT, IR_SCL_PIN, gpioModeWiredAndPullUpFilter, 1);

    I2C_Init_TypeDef i2cInit = I2C_INIT_DEFAULT;
    i2cInit.freq = I2C_FREQ_FAST_MAX;  /* 400 kHz */
    I2C_Init(I2C0, &i2cInit);

    /* Route SDA/SCL to PA0/PA1 via ROUTE registers (EFR32 series 2) */
    GPIO->I2CROUTE[0].SDAROUTE = (IR_I2C_PORT << _GPIO_I2C_SDAROUTE_PORT_SHIFT)
                                | (IR_SDA_PIN  << _GPIO_I2C_SDAROUTE_PIN_SHIFT);
    GPIO->I2CROUTE[0].SCLROUTE = (IR_I2C_PORT << _GPIO_I2C_SCLROUTE_PORT_SHIFT)
                                | (IR_SCL_PIN  << _GPIO_I2C_SCLROUTE_PIN_SHIFT);
    GPIO->I2CROUTE[0].ROUTEEN  = GPIO_I2C_ROUTEEN_SDAPEN | GPIO_I2C_ROUTEEN_SCLPEN;
}
 
static bool mlx_read_frame(void)
{
    uint8_t reg_hi = (uint8_t)(MLX_REG_RAM >> 8);
    uint8_t reg_lo = (uint8_t)(MLX_REG_RAM & 0xFF);

    I2C_TransferSeq_TypeDef seq;
    I2C_TransferReturn_TypeDef ret;
    uint8_t cmd[2] = { reg_hi, reg_lo };

    seq.addr        = MLX_ADDR << 1;
    seq.flags       = I2C_FLAG_WRITE_READ;
    seq.buf[0].data = cmd;
    seq.buf[0].len  = 2;
    seq.buf[1].data = (uint8_t *)s_mlx_frame;
    seq.buf[1].len  = MLX_FRAME_WORDS * 2;

    ret = I2CSPM_Transfer(I2C0, &seq);
    return (ret == i2cTransferDone);
}
 
IRResult ir_read(void)
{
    IRResult r = { 0 };

    if (!mlx_read_frame()) {
        r.target_found = false;
        return r;
    }

    float peak   = -300.0f;
    float sum_t  = 0.0f;
    float sum_ax = 0.0f;
    float sum_ay = 0.0f;
    int   hot_n  = 0;

    for (int row = 0; row < MLX_ROWS; row++) {
        for (int col = 0; col < MLX_COLS; col++) {
            /* MLX90640 rawâ†’Â°C: simplified (real driver uses calibration EEPROM) */
            uint16_t raw = s_mlx_frame[row * MLX_COLS + col];
            float temp_c = ((float)raw / 100.0f) - 273.15f;

            if (temp_c > peak) peak = temp_c;

            if (temp_c > IR_THRESHOLD_C) {
                /* normalised position â€“1..+1 */
                float ax = ((float)col / (MLX_COLS - 1)) * 2.0f - 1.0f;
                float ay = ((float)row / (MLX_ROWS - 1)) * 2.0f - 1.0f;
                sum_t  += temp_c;
                sum_ax += ax * temp_c;
                sum_ay += ay * temp_c;
                hot_n++;
            }
        }
    }

    r.peak_temp_c = peak;
    if (hot_n > 0 && sum_t > 0.0f) {
        r.centroid_az   = sum_ax / sum_t;
        r.centroid_el   = sum_ay / sum_t;
        r.target_found  = true;
    } else {
        r.target_found  = false;
    }
    return r;
}
 
static uint32_t us_pulse_us(GPIO_Port_TypeDef tp, unsigned tpin,
                             GPIO_Port_TypeDef ep, unsigned epin)
{
    /* 10 Âµs trigger pulse */
    GPIO_PinOutSet(tp, tpin);
    for (volatile int i = 0; i < 240; i++);  /* ~10 Âµs @ 24 MHz */
    GPIO_PinOutClear(tp, tpin);

    /* Wait for ECHO high */
    uint32_t timeout = US_TIMEOUT_US;
    while (!GPIO_PinInGet(ep, epin) && timeout--);
    if (!timeout) return 0;

    /* Measure pulse width in Âµs (simple polling; TIMER capture preferred) */
    uint32_t count = 0;
    while (GPIO_PinInGet(ep, epin) && count < US_TIMEOUT_US) {
        for (volatile int i = 0; i < 24; i++);  /* ~1 Âµs */
        count++;
    }
    return count;
}

USResult ultrasonic_read(void)
{
    USResult r;
    uint32_t t1 = us_pulse_us(US1_TRIG_PORT, US1_TRIG_PIN,
                               US1_ECHO_PORT, US1_ECHO_PIN);
    uint32_t t2 = us_pulse_us(US2_TRIG_PORT, US2_TRIG_PIN,
                               US2_ECHO_PORT, US2_ECHO_PIN);

    r.dist1_cm = (float)t1 * US_SOUND_CM_US / 2.0f;
    r.dist2_cm = (float)t2 * US_SOUND_CM_US / 2.0f;
    r.in_range = (r.dist1_cm < US_MAX_CM && r.dist1_cm > 2.0f) ||
                 (r.dist2_cm < US_MAX_CM && r.dist2_cm > 2.0f);
    return r;
}
 
static void uart_radar_init(void)
{
    USART_InitAsync_TypeDef init = USART_INITASYNC_DEFAULT;
    init.baudrate = 9600;
    USART_InitAsync(RADAR_UART, &init);

    GPIO->USARTROUTE[0].TXROUTE = (RADAR_TX_PORT << _GPIO_USART_TXROUTE_PORT_SHIFT)
                                 | (RADAR_TX_PIN  << _GPIO_USART_TXROUTE_PIN_SHIFT);
    GPIO->USARTROUTE[0].RXROUTE = (RADAR_RX_PORT << _GPIO_USART_RXROUTE_PORT_SHIFT)
                                 | (RADAR_RX_PIN  << _GPIO_USART_RXROUTE_PIN_SHIFT);
    GPIO->USARTROUTE[0].ROUTEEN = GPIO_USART_ROUTEEN_TXPEN | GPIO_USART_ROUTEEN_RXPEN;
}

static void adc_radar_init(void)
{
    ADC_Init_TypeDef     adcInit  = ADC_INIT_DEFAULT;
    ADC_InitSingle_TypeDef single = ADC_INITSINGLE_DEFAULT;

    adcInit.timebase = ADC_TimebaseCalc(0);
    adcInit.prescale = ADC_PrescaleCalc(1000000, 0);
    ADC_Init(ADC0, &adcInit);

    single.reference = adcRefVDD;
    single.posSel    = RADAR_ADC_CH;
    single.resolution = adcRes12Bit;
    ADC_InitSingle(ADC0, &single);
}

RadarResult radar_read(void)
{
    RadarResult r = { 0 };

    /* Read ADC IF amplitude */
    ADC_Start(ADC0, adcStartSingle);
    while (ADC0->STATUS & ADC_STATUS_SINGLEACT);
    uint32_t raw = ADC_DataSingleGet(ADC0);

    /* Convert ADC counts to velocity via HB100 sensitivity (~19.5 Hz/km/h) */
    float voltage  = ((float)raw / 4095.0f) * 3.3f;
    r.speed_kmh    = voltage * 10.0f;       /* simplified calibration */
    r.approaching  = (r.speed_kmh > RADAR_SPEED_KMH_MIN);
    return r;
}
 
static uint8_t s_confirm_count = 0;

Target sensors_read_fused(void)
{
    Target t = { 0 };

    IRResult    ir  = ir_read();
    USResult    us  = ultrasonic_read();
    RadarResult rad = radar_read();

    /* Require at least IR + one corroborating sensor */
    bool corroborated = (us.in_range || rad.approaching);

    if (ir.target_found && corroborated) {
        /* Convert IR centroid (â€“1..+1) to degrees */
        t.az_deg   = ir.centroid_az * 90.0f;
        t.el_deg   = ir.centroid_el * 45.0f;

        /* Best distance: prefer ultrasonic if in range */
        t.dist_cm  = us.in_range
                   ? ((us.dist1_cm + us.dist2_cm) / 2.0f)
                   : 999.0f;

        t.speed_kmh = rad.speed_kmh;

        if (s_confirm_count < THREAT_CONFIRM_N)
            s_confirm_count++;

        t.confidence = s_confirm_count;
        t.valid      = (s_confirm_count >= THREAT_CONFIRM_N);
    } else {
        /* Decay confirmation counter */
        if (s_confirm_count > 0) s_confirm_count--;
        t.valid = false;
    }
    return t;
}
