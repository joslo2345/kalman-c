#ifndef KF_TYPES_H
#define KF_TYPES_H

#include "kalman/kf_config.h"

/* Every API function returns one of these. On error, the filter state is left unchanged. */
typedef enum {
    KF_OK = 0,
    KF_ERR_INVALID_INPUT = -1,         /* NULL pointer, bad dimension, NaN/Inf input */
    KF_ERR_NOT_POSITIVE_DEFINITE = -2, /* e.g. a singular innovation covariance */
    KF_ERR_NOT_IMPLEMENTED = -3,
    KF_ERR_MODEL_FAILED = -4 /* a user-supplied model callback returned non-zero */
} kf_status;

/* Matrices are flat, row-major arrays sized for the maximum dimensions. */
typedef struct {
    int n;                                  /* state size */
    int m;                                  /* measurement size */
    kf_real x[KF_MAX_STATE];                /* state estimate */
    kf_real P[KF_MAX_STATE * KF_MAX_STATE]; /* covariance */
    kf_real Q[KF_MAX_STATE * KF_MAX_STATE]; /* process noise */
    kf_real R[KF_MAX_MEAS * KF_MAX_MEAS];   /* measurement noise */
    kf_real nis;                            /* last normalized innovation squared */
} kf_state;

#endif
