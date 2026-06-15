#include "ik_5bar.h"
#include <math.h>

#define RAD2DEG 57.29577951f

/*
 * Giai phuong trinh dang: A*cos(t) + B*sin(t) = C
 * Hai nghiem:
 *   t1 = 2*atan((B + s)/(A + C))
 *   t2 = 2*atan((B - s)/(A + C))   voi s = sqrt(A^2 + B^2 - C^2)
 * Tra ve 0 neu disc < 0 (ngoai workspace).
 */
static uint8_t angle(float A, float B, float C, float *t1, float *t2)
{
    float disc = A * A + B * B - C * C;
    if (disc < 0.0f) return 0;

    float s     = sqrtf(disc);
    float denom = A + C;
    *t1 = 2.0f * atanf((B + s) / denom);
    *t2 = 2.0f * atanf((B - s) / denom);
    return 1;
}

uint8_t ik_5bar(float x, float y, float *theta1_deg, float *theta2_deg)
{
    float theta1_t1, theta1_t2;
    float theta2_t1, theta2_t2;

    /* ====== Nhanh trai (O-A-B) ====== */
    float a = 2.0f * x * IK_L1;
    float b = 2.0f * y * IK_L1;
    float c = x * x + y * y + IK_L1 * IK_L1 - IK_L2 * IK_L2;
    if (!angle(a, b, c, &theta1_t1, &theta1_t2)) return 0;

    /* ====== Nhanh phai (D-C-B) ====== */
    float d = 2.0f * (x - IK_L0) * IK_L4;
    float e = 2.0f * y * IK_L4;
    float f = (IK_L0 - x) * (IK_L0 - x) + y * y + IK_L4 * IK_L4 - IK_L3 * IK_L3;
    if (!angle(d, e, f, &theta2_t1, &theta2_t2)) return 0;

    /* ====== Chon nghiem theo y ====== */
    float theta1, theta2;
    if (y >= 0.0f) {
        theta1 = theta1_t1;
        theta2 = theta2_t2;
    } else {
        theta1 = theta1_t2;
        theta2 = theta2_t1;
    }

    /* Tra ve goc toan hoc thuan: nguoc chieu kim dong ho (CCW) so voi
     * truc +X, dung y nhu MATLAB. Viec dao sang chieu dong co (CW)
     * duoc xu ly o lop anh xa servo (map_servo1/2). */
    *theta1_deg = theta1 * RAD2DEG;
    *theta2_deg = theta2 * RAD2DEG;
    return 1;
}
