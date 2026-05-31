#ifndef SENSORS_H
#define SENSORS_H

/* ================================================================
   sensors.h  â€“  IR camera, ultrasonic, Doppler radar drivers
   ================================================================ */
#include <stdint.h>
#include <stdbool.h> 
typedef struct {
    float  peak_temp_c;    /* hottest pixel temperature           */
    float  centroid_az;    /* azimuth  of heat centroid  (â€“1..1)  */
    float  centroid_el;    /* elevation of heat centroid (â€“1..1)  */
    bool   target_found;
} IRResult;
 
typedef struct {
    float  dist1_cm;       /* sensor pair 1 distance              */
    float  dist2_cm;       /* sensor pair 2 distance              */
    bool   in_range;       /* true if any return < US_MAX_CM      */
} USResult; 
typedef struct {
    float  speed_kmh;      /* Doppler speed estimate              */
    bool   approaching;    /* true = incoming threat              */
} RadarResult; 
typedef struct {
    float    az_deg;       /* azimuth  â€“90..+90                   */
    float    el_deg;       /* elevation â€“45..+45                  */
    float    dist_cm;      /* best distance estimate              */
    float    speed_kmh;
    uint8_t  confidence;   /* 0-3, increments each confirm frame  */
    bool     valid;
} Target;

/* Initialise all sensors (I2C, UART, GPIO) */
void sensors_init(void);

/* Read each sensor and return fused target.
   Call once per control loop tick.                               */
Target sensors_read_fused(void);

/* Individual reads (used internally, exposed for debug) */
IRResult    ir_read(void);
USResult    ultrasonic_read(void);
RadarResult radar_read(void);

#endif /* SENSORS_H */
