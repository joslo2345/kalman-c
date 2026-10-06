#ifndef KF_CONFIG_H
#define KF_CONFIG_H

#ifdef KF_USE_DOUBLE
typedef double kf_real;
#define KF_PRECISION_NAME "float64"
#else
typedef float kf_real;
#define KF_PRECISION_NAME "float32"
#endif

#ifndef KF_MAX_STATE
#define KF_MAX_STATE 12
#endif

#ifndef KF_MAX_MEAS
#define KF_MAX_MEAS 6
#endif

/* Largest square matrix the library ever handles: P (n x n) or S (m x m). */
#if KF_MAX_STATE > KF_MAX_MEAS
#define KF_MAX_DIM KF_MAX_STATE
#else
#define KF_MAX_DIM KF_MAX_MEAS
#endif

#endif
