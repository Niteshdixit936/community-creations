/* ================================================================
   actuator.c  â€“  Servo PWM + laser burst governor
   TIMER0 CC0/CC1 â†’ 50 Hz servo PWM
   PD2 GPIO â†’ IRLZ44N MOSFET gate (laser)
   ================================================================ */
#include "actuator.h"
#include "config.h"
#include "em_timer.h"
#include "em_gpio.h"
#include "em_cmu.h"
#include "em_emu.h"
#include <stdint.h>
#include <stdbool.h> 
#define TIMER_PRESCALER     timerPrescale64
#define TIMER_CLOCK_HZ      300000u
#define TIMER_TOP           (TIMER_CLOCK_HZ / SERVO_FREQ_HZ)   /* 6000 */
 
#define US_TO_TICKS(us)     ((uint32_t)((us) * TIMER_CLOCK_HZ / 1000000u))
 
static volatile uint32_t s_laser_on_ms    = 0;
static volatile uint32_t s_laser_off_ms   = 0;
static volatile bool     s_laser_active   = false;
static volatile bool     s_cooldown       = false;
 
static float mcu_temp_c(void)
{
    /* Production calibration lookup â€“ simplified placeholder.
       In real firmware use DEVINFO->CAL + EMU_GetTemperature()  */
    return 40.0f;   /* replace with EMU_GetTemperature() in SDK */
}

/* ----------------------------------------------------------------
   actuator_init
   ---------------------------------------------------------------- */
void actuator_init(void)
{
    CMU_ClockEnable(cmuClock_TIMER0, true);
    CMU_ClockEnable(cmuClock_GPIO,   true);

    /* Laser GPIO */
    GPIO_PinModeSet(LASER_PORT, LASER_PIN, gpioModePushPull, 0);

    /* Servo GPIO â€“ TIMER0 output */
    GPIO_PinModeSet(PAN_PWM_PORT,  PAN_PWM_PIN,  gpioModePushPull, 0);
    GPIO_PinModeSet(TILT_PWM_PORT, TILT_PWM_PIN, gpioModePushPull, 0);

    /* Route TIMER0 CC0â†’PD0, CC1â†’PD1 */
    GPIO->TIMERROUTE[0].ROUTEEN  = GPIO_TIMER_ROUTEEN_CC0PEN
                                 | GPIO_TIMER_ROUTEEN_CC1PEN;
    GPIO->TIMERROUTE[0].CC0ROUTE = (PAN_PWM_PORT  << _GPIO_TIMER_CC0ROUTE_PORT_SHIFT)
                                 | (PAN_PWM_PIN   << _GPIO_TIMER_CC0ROUTE_PIN_SHIFT);
    GPIO->TIMERROUTE[0].CC1ROUTE = (TILT_PWM_PORT << _GPIO_TIMER_CC1ROUTE_PORT_SHIFT)
                                 | (TILT_PWM_PIN  << _GPIO_TIMER_CC1ROUTE_PIN_SHIFT);

    /* Timer init */
    TIMER_Init_TypeDef timerInit = TIMER_INIT_DEFAULT;
    timerInit.prescale = TIMER_PRESCALER;
    TIMER_Init(TIMER0, &timerInit);
    TIMER_TopSet(TIMER0, TIMER_TOP - 1);

    /* CC channels â€“ PWM mode */
    TIMER_InitCC_TypeDef ccInit = TIMER_INITCC_DEFAULT;
    ccInit.mode    = timerCCModePWM;
    ccInit.cmoa    = timerOutputActionToggle;
    ccInit.outInvert = false;
    TIMER_InitCC(TIMER0, PAN_CC_CH,  &ccInit);
    TIMER_InitCC(TIMER0, TILT_CC_CH, &ccInit);

    /* Default: centre position */
    servo_set_pan_deg(0.0f);
    servo_set_tilt_deg(0.0f);

    TIMER_Enable(TIMER0, true);
}
 
static uint32_t deg_to_ticks(float deg, float min_deg, float max_deg)
{
    if (deg < min_deg) deg = min_deg;
    if (deg > max_deg) deg = max_deg;
 
    float t   = (deg - min_deg) / (max_deg - min_deg);
    float us  = SERVO_MIN_US + t * (float)(SERVO_MAX_US - SERVO_MIN_US);
    return US_TO_TICKS((uint32_t)us);
}

void servo_set_pan_deg(float deg)
{
    uint32_t ticks = deg_to_ticks(deg, PAN_MIN_DEG, PAN_MAX_DEG);
    TIMER_CompareSet(TIMER0, PAN_CC_CH, ticks);
}

void servo_set_tilt_deg(float deg)
{
    uint32_t ticks = deg_to_ticks(deg, TILT_MIN_DEG, TILT_MAX_DEG);
    TIMER_CompareSet(TIMER0, TILT_CC_CH, ticks);
}
 
bool laser_is_inhibited(void)
{
    return s_cooldown || (mcu_temp_c() >= THERMAL_LIMIT_C);
}

bool laser_fire(bool enable)
{
    if (enable) {
        if (laser_is_inhibited()) return false;
        if (!s_laser_active) {
            s_laser_on_ms  = 0;
            s_laser_active = true;
            GPIO_PinOutSet(LASER_PORT, LASER_PIN);
        }
        return true;
    } else {
        if (s_laser_active) {
            s_laser_active = false;
            s_laser_off_ms = 0;
            s_cooldown     = true;
            GPIO_PinOutClear(LASER_PORT, LASER_PIN);
        }
        return false;
    }
}
 
void actuator_tick_ms(void)
{
    if (s_laser_active) {
        s_laser_on_ms++;
        if (s_laser_on_ms >= LASER_MAX_ON_MS) {
            /* Auto-cutoff burst */
            laser_fire(false);
        }
    } else if (s_cooldown) {
        s_laser_off_ms++;
        if (s_laser_off_ms >= LASER_COOLDOWN_MS) {
            s_cooldown = false;
        }
    }
}
