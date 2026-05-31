#ifndef CONFIG_H
#define CONFIG_H

/* ================================================================
   config.h  â€“  Laser Air Defence System
   Silicon Labs EFR32xG28 / EFM32PG28
   Simplicity Studio 5 + Gecko SDK 4.x
   ================================================================ */
 
#define IR_I2C_PORT         gpioPortA
#define IR_SDA_PIN          0
#define IR_SCL_PIN          1

/* UART0  â€“  HB100 Doppler radar */
#define RADAR_UART          USART0
#define RADAR_TX_PORT       gpioPortB
#define RADAR_TX_PIN        0
#define RADAR_RX_PORT       gpioPortB
#define RADAR_RX_PIN        1

/* GPIO  â€“  HC-SR04 ultrasonic (pair 1) */
#define US1_TRIG_PORT       gpioPortC
#define US1_TRIG_PIN        0
#define US1_ECHO_PORT       gpioPortC
#define US1_ECHO_PIN        1

/* GPIO  â€“  HC-SR04 ultrasonic (pair 2) */
#define US2_TRIG_PORT       gpioPortC
#define US2_TRIG_PIN        2
#define US2_ECHO_PORT       gpioPortC
#define US2_ECHO_PIN        3

/* ADC  â€“  radar IF analog signal */
#define RADAR_ADC_PORT      gpioPortA
#define RADAR_ADC_PIN       4
#define RADAR_ADC_CH        adcPosSelAPORT1XCH4

/* PWM  â€“  pan servo (TIMER0 CC0) */
#define PAN_TIMER           TIMER0
#define PAN_CC_CH           0
#define PAN_PWM_PORT        gpioPortD
#define PAN_PWM_PIN         0

/* PWM  â€“  tilt servo (TIMER0 CC1) */
#define TILT_CC_CH          1
#define TILT_PWM_PORT       gpioPortD
#define TILT_PWM_PIN        1

/* GPIO  â€“  laser MOSFET gate */
#define LASER_PORT          gpioPortD
#define LASER_PIN           2

/* UART1  â€“  BLE / Zigbee telemetry */
#define BLE_UART            USART1
#define BLE_TX_PORT         gpioPortE
#define BLE_TX_PIN          0
#define BLE_RX_PORT         gpioPortE
#define BLE_RX_PIN          1 

#define SERVO_FREQ_HZ       50u          /* 50 Hz â†’ 20 ms period  */
#define SERVO_MIN_US        600u         /* ~0Â°                   */
#define SERVO_MID_US        1500u        /* ~90Â°                  */
#define SERVO_MAX_US        2400u        /* ~180Â°                 */
#define PAN_MIN_DEG         (-90)
#define PAN_MAX_DEG         90
#define TILT_MIN_DEG        (-45)
#define TILT_MAX_DEG        45
 
#define LASER_MAX_ON_MS     500u         /* burst cap             */
#define LASER_COOLDOWN_MS   300u         /* min off time          */
#define THERMAL_LIMIT_C     65           /* MCU die temp cutoff   */

#define IR_THRESHOLD_C      35.0f        /* heat sig min (Â°C)     */
#define US_MAX_CM           400u         /* ultrasonic max range  */
#define RADAR_SPEED_KMH_MIN 5.0f         /* ignore slow objects   */
#define THREAT_CONFIRM_N    3u           /* frames to confirm     */

#define KF_Q                0.1f         /* process noise         */
#define KF_R                1.0f         /* measurement noise     */
 
#define CONTROL_LOOP_MS     20u          /* 50 Hz control tick    */
#define TELEMETRY_MS        500u         /* BLE report interval   */
 
#define NODE_ID             0x01u        /* unique per unit       */
#define MESH_CHANNEL        15u

#endif /* CONFIG_H */
