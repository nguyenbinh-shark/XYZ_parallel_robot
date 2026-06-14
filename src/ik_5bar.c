#include "ik_5bar.h"
#include <math.h>

#define RAD2DEG 57.29577951f

static uint8_t solve_branch(float c, float d, float e, float sign, float *t_rad)
{
    float s = d * d + e * e - c * c;
    if (s < 0.0f) return 0;

    float root = sqrtf(s);
    float num  = e + sign * root;
    float den  = d + c;

    if (num == 0.0f && den == 0.0f) return 0;

    *t_rad = 2.0f * atan2f(num, den);
    return 1;
}

uint8_t ik_5bar(float x, float y, float *theta1_deg, float *theta2_deg)
{
    float t1, t2;

    float c1 = x * x + y * y + IK_L1 * IK_L1 - IK_L2 * IK_L2;
    float d1 = 2.0f * IK_L1 * x;
    float e1 = 2.0f * IK_L1 * y;
    if (!solve_branch(c1, d1, e1, +1.0f, &t1)) return 0;

    float dx = x - IK_L0;
    float c2 = dx * dx + y * y + IK_L4 * IK_L4 - IK_L3 * IK_L3;
    float d2 = 2.0f * IK_L4 * dx;
    float e2 = 2.0f * IK_L4 * y;
    if (!solve_branch(c2, d2, e2, -1.0f, &t2)) return 0;

    if (t1 < 0.0f) t1 += 2.0f * (float)M_PI;
    if (t2 < 0.0f) t2 += 2.0f * (float)M_PI;

    *theta1_deg = t1 * RAD2DEG;
    *theta2_deg = t2 * RAD2DEG;
    return 1;
}
