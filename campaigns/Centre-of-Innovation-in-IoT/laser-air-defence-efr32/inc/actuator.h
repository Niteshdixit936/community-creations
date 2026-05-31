#ifndef ACTUATOR_H
#define ACTUATOR_H

/* ================================================================
   actuator.h  â€“  Pan-tilt servo + laser burst control
   ================================================================ */
#include <stdbool.h>

/* Initialise TIMER0 for 50 Hz PWM on PD0 (pan) and PD1 (tilt) */
void actuator_init(void);

/* Set servo angles: pan â€“90..+90Â°, tilt â€“45..+45Â° */
void servo_set_pan_deg(float deg);
void servo_set_tilt_deg(float deg);

/* Laser on/off with built-in thermal + burst-time guard */
bool laser_fire(bool enable);

/* Returns true if thermal limit or cooldown is active */
bool laser_is_inhibited(void);

/* Must be called every ms from SysTick to update laser timers */
void actuator_tick_ms(void);

#endif /* ACTUATOR_H */
