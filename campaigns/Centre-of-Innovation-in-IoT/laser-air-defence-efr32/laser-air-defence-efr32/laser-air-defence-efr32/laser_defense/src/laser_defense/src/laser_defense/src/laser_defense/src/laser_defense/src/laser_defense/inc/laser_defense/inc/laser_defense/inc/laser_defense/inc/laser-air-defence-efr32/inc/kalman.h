#ifndef KALMAN_H
#define KALMAN_H

/* ================================================================
   kalman.h  â€“  1-D Kalman filter for target position / velocity
   Used for both azimuth and elevation axes independently.
   ================================================================ */

typedef struct {
    float x;       /* estimated position  (degrees)       */
    float v;       /* estimated velocity  (degrees/sec)   */
    float p[2][2]; /* error covariance 2Ã—2 matrix         */
    float q;       /* process noise variance              */
    float r;       /* measurement noise variance          */
} KalmanState;

/* Initialise filter at a known position with zero velocity */
void  kalman_init(KalmanState *k, float init_pos, float q, float r);

/* Predict next state given elapsed time dt (seconds) */
void  kalman_predict(KalmanState *k, float dt);

/* Update with a new position measurement; returns corrected pos */
float kalman_update(KalmanState *k, float measured_pos);

/* Compute lead-angle offset for moving target:
   look_ahead_s = servo latency in seconds (~0.05)
   Returns predicted position after look_ahead_s               */
float kalman_lead_angle(const KalmanState *k, float look_ahead_s);

#endif /* KALMAN_H */
