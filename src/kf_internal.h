#ifndef KF_INTERNAL_H
#define KF_INTERNAL_H

/* Shared by the KF and EKF; not part of the public API. */

#include "kalman/kf_types.h"

/* Given a predicted state and transition Jacobian F, set P = F P F^T + Q.
 * Validates, then commits x and P to *kf only on success. */
int kf_core_predict(kf_state *kf, const kf_real *x_pred, const kf_real *F);

/* Given an innovation y and measurement Jacobian H, run the Joseph-form update.
 * Validates, then commits x, P and nis to *kf only on success. */
int kf_core_update(kf_state *kf, const kf_real *y, const kf_real *H);

#endif
