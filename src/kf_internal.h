#ifndef KF_INTERNAL_H
#define KF_INTERNAL_H

/* Shared by the KF and EKF; not part of the public API. */

#include "kalman/kf_types.h"

/* kf->n and kf->m are plain fields the caller can overwrite after kf_init, and
 * every scratch buffer is sized by KF_MAX_STATE / KF_MAX_MEAS, so each entry
 * point re-checks them before indexing anything. */
static inline int kf_dims_ok(const kf_state *kf) {
    return (kf->n >= 1) && (kf->n <= KF_MAX_STATE) && (kf->m >= 1) && (kf->m <= KF_MAX_MEAS);
}

/* Given a predicted state and transition Jacobian F, set P = F P F^T + Q.
 * Validates, then commits x and P to *kf only on success. */
int kf_core_predict(kf_state *kf, const kf_real *x_pred, const kf_real *F);

/* Given an innovation y and measurement Jacobian H, run the Joseph-form update.
 * Validates, then commits x, P and nis to *kf only on success. */
int kf_core_update(kf_state *kf, const kf_real *y, const kf_real *H);

#endif
