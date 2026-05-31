/* ================================================================
   kalman.c  â€“  1-D Kalman filter implementation
   State vector: [position, velocity]
   ================================================================ */
#include "kalman.h"

void kalman_init(KalmanState *k, float init_pos, float q, float r)
{
    k->x    = init_pos;
    k->v    = 0.0f;
    k->q    = q;
    k->r    = r;
    /* initial covariance â€“ high uncertainty */
    k->p[0][0] = 1.0f;  k->p[0][1] = 0.0f;
    k->p[1][0] = 0.0f;  k->p[1][1] = 1.0f;
}

void kalman_predict(KalmanState *k, float dt)
{
    /* State transition: x_new = x + v*dt,  v_new = v */
    k->x = k->x + k->v * dt;

    /* Covariance prediction:  P = F*P*F' + Q
       F = [[1, dt],[0, 1]]  â†’ only upper-left changes significantly */
    float p00 = k->p[0][0] + dt * (k->p[1][0] + k->p[0][1]) + dt * dt * k->p[1][1] + k->q;
    float p01 = k->p[0][1] + dt * k->p[1][1];
    float p10 = k->p[1][0] + dt * k->p[1][1];
    float p11 = k->p[1][1] + k->q * 0.1f;

    k->p[0][0] = p00;
    k->p[0][1] = p01;
    k->p[1][0] = p10;
    k->p[1][1] = p11;
}

float kalman_update(KalmanState *k, float measured_pos)
{
    /* Innovation */
    float y = measured_pos - k->x;

    /* Innovation covariance S = H*P*H' + R  (H = [1,0]) */
    float s = k->p[0][0] + k->r;
    if (s < 1e-6f) s = 1e-6f;   /* guard divide-by-zero */

    /* Kalman gain K = P*H'/S */
    float k0 = k->p[0][0] / s;
    float k1 = k->p[1][0] / s;

    /* State update */
    k->x = k->x + k0 * y;
    k->v = k->v + k1 * y;

    /* Covariance update  P = (I - K*H)*P */
    float p00 = (1.0f - k0) * k->p[0][0];
    float p01 = (1.0f - k0) * k->p[0][1];
    float p10 = k->p[1][0] - k1 * k->p[0][0];
    float p11 = k->p[1][1] - k1 * k->p[0][1];

    k->p[0][0] = p00;
    k->p[0][1] = p01;
    k->p[1][0] = p10;
    k->p[1][1] = p11;

    return k->x;
}

float kalman_lead_angle(const KalmanState *k, float look_ahead_s)
{
    return k->x + k->v * look_ahead_s;
}
