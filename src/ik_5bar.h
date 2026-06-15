#ifndef IK_5BAR_H
#define IK_5BAR_H

#include <stdint.h>

/*
 * Dong hoc nguoc co cau 5 khau song song (five-bar linkage)
 *
 *        E (x, y)  <- diem cuoi
 *       /  \
 *     L2    L3


 **  (0,0)    (L0,0)    |        |
 *    A--------C
 *    |        |
 *   L1        L4 
 *    B        D
 *    \       /
*
 *  - Dong co 1 dat tai goc A(0,0), goc quay theta1 (do, so voi truc +X)
 *  - Dong co 2 dat tai C(L0,0), goc quay theta2 (do, so voi truc +X)
 *  - Quy uoc goc: duong = thuan chieu kim dong ho.
 *  - Working mode: khuyu huong ra ngoai
 */

/* Kich thuoc khau (mm) */
#define IK_L0  55.0f
#define IK_L1  70.0f
#define IK_L2  130.0f
#define IK_L3  130.0f
#define IK_L4  70.0f

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Tinh dong hoc nguoc.
 *   x, y      : toa do diem cuoi (mm)
 *   theta1_deg: [out] goc khau L1 (do, duong = NGUOC chieu kim dong ho / CCW)
 *   theta2_deg: [out] goc khau L4 (do, duong = NGUOC chieu kim dong ho / CCW)
 *   (Dao sang chieu dong co CW xu ly o lop anh xa servo)
 * Tra ve 1 neu giai duoc, 0 neu ngoai workspace.
 */
uint8_t ik_5bar(float x, float y, float *theta1_deg, float *theta2_deg);

#ifdef __cplusplus
}
#endif

#endif /* IK_5BAR_H */
