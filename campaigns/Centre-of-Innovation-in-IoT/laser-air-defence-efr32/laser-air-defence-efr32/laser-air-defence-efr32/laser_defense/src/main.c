/* ================================================================
   main.c  â€“  Laser Air Defence System  â€“  main control loop
   Silicon Labs EFR32xG28 / EFM32PG28
   Simplicity Studio 5 + Gecko SDK 4.x

   Control loop (50 Hz / 20 ms):
     1. Read & fuse sensors
     2. Kalman predict + update both axes
     3. Compute lead angle
     4. Drive servos
     5. Fire laser if threat confirmed + no inhibit
     6. Accept mesh targets from peer nodes
     7. Send BLE telemetry every 500 ms
   ================================================================ */

#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "sl_sleeptimer.h"   

#include "config.h"
#include "sensors.h"
#include "actuator.h"
#include "kalman.h"
#include "telemetry.h"

#include <stdbool.h>
#include <stdint.h>
 
static volatile uint32_t g_tick_ms = 0;

void SysTick_Handler(void)
{
    g_tick_ms++;
    actuator_tick_ms();   
}

static uint32_t tick_now(void) { return g_tick_ms; }

static void clock_init(void)
{
    /* Use HFRCO at 19.2 MHz (default on EFR32) */
    CMU_HFRCODPLLBandSet(cmuHFRCODPLLFreq_19M2Hz);
    CMU_ClockSelectSet(cmuClock_SYSCLK, cmuSelect_HFRCODPLL);
    SysTick_Config(CMU_ClockFreqGet(cmuClock_CORE) / 1000u);
}
 
static void wait_until(uint32_t deadline_ms)
{
    while ((int32_t)(deadline_ms - tick_now()) > 0)
        __WFI();   
}
 
static float mcu_temp(void)
{
    return EMU_GetTemperature();
}

int main(void)
{
    CHIP_Init();
    clock_init();

    sensors_init();
    actuator_init();
    telemetry_init();

    KalmanState kf_az, kf_el;
    kalman_init(&kf_az, 0.0f, KF_Q, KF_R);
    kalman_init(&kf_el, 0.0f, KF_Q, KF_R);

    uint32_t last_telem_ms  = 0;
    uint32_t last_loop_ms   = 0;
    bool     laser_on       = false;
    float    dt             = (float)CONTROL_LOOP_MS / 1000.0f;  

    servo_set_pan_deg(0.0f);
    servo_set_tilt_deg(0.0f);
 
    while (1) {
        uint32_t loop_start = tick_now();
        uint32_t deadline   = loop_start + CONTROL_LOOP_MS;

        Target local = sensors_read_fused();
 
        Target mesh_t = { 0 };
        bool   got_mesh = mesh_receive_target(&mesh_t);
 
        Target *best = &local;
        if (!local.valid && got_mesh && mesh_t.valid)
            best = &mesh_t;
 
        kalman_predict(&kf_az, dt);
        kalman_predict(&kf_el, dt);
 
        float cmd_az = 0.0f;
        float cmd_el = 0.0f;

        if (best->valid) {
            float az_est = kalman_update(&kf_az, best->az_deg);
            float el_est = kalman_update(&kf_el, best->el_deg);

          
            cmd_az = kalman_lead_angle(&kf_az, 0.05f);
            cmd_el = kalman_lead_angle(&kf_el, 0.05f);

            (void)az_est; (void)el_est;  
 
            servo_set_pan_deg(cmd_az);
            servo_set_tilt_deg(cmd_el);
 
            laser_on = laser_fire(true);
 
            mesh_broadcast_target(best);

        } else {
            
            laser_fire(false);
            laser_on = false;

          
            float cur_az = kf_az.x;
            float cur_el = kf_el.x;
            if (cur_az >  1.0f) cur_az -= 1.0f;
            else if (cur_az < -1.0f) cur_az += 1.0f;
            else cur_az = 0.0f;

            if (cur_el >  1.0f) cur_el -= 1.0f;
            else if (cur_el < -1.0f) cur_el += 1.0f;
            else cur_el = 0.0f;

            servo_set_pan_deg(cur_az);
            servo_set_tilt_deg(cur_el);

           
            kf_az.x = cur_az;
            kf_el.x = cur_el;
        }
        if ((tick_now() - last_telem_ms) >= TELEMETRY_MS) {
            last_telem_ms = tick_now();
            telemetry_send(best, laser_on, mcu_temp());
        }
        last_loop_ms = loop_start;
        wait_until(deadline);
    }
}
