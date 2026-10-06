#ifndef KF_CONFIG_H
#define KF_CONFIG_H

/**
 * @file kf_config.h
 * @brief Compile-time configuration: precision, maximum sizes, specializations.
 *
 * Every option is a preprocessor definition, set on the compiler command line
 * (or as the CMake cache variables of the same name):
 *
 * - `KF_USE_DOUBLE`: use `double` instead of `float` for ::kf_real.
 * - `KF_MAX_STATE`, `KF_MAX_MEAS`: the largest sizes, which set the size of
 *   ::kf_state and of the stack scratch space.
 * - `KF_SPECIALIZE`: optional compile-time specializations of the KF/EKF
 *   predict and update for the sizes a firmware actually uses, listed as
 *   `KF_SIZE(n, m)` entries, for example `-DKF_SPECIALIZE=KF_SIZE(4,2)KF_SIZE(15,6)`.
 *   Each listed size gets its own copy of the core with constant dimensions,
 *   which the compiler can unroll and vectorize; other sizes use the general
 *   code. Results agree to rounding but are not bit-identical, because the
 *   compiler may vectorize and fuse multiply-adds differently once sizes are
 *   constant. Every entry costs code size. With CMake, set the `KF_SPECIALIZE`
 *   cache variable rather than `CMAKE_C_FLAGS`, whose parentheses the shell mangles.
 */

/** Library version as "major.minor.patch". CMake reads its project version from here. */
#define KALMAN_C_VERSION "0.1.0"

#ifndef KF_USE_DOUBLE
/** Scalar type for every filter quantity: `float`, or `double` with KF_USE_DOUBLE. */
typedef float kf_real;
/** Precision name used in benchmark output: "float32", or "float64" with KF_USE_DOUBLE. */
#define KF_PRECISION_NAME "float32"
#else
typedef double kf_real;
#define KF_PRECISION_NAME "float64"
#endif

#ifndef KF_MAX_STATE
/** Largest state size n. Sizes every ::kf_state and scratch buffer; default 12. */
#define KF_MAX_STATE 12
#endif

#ifndef KF_MAX_MEAS
/** Largest measurement size m; default 6. */
#define KF_MAX_MEAS 6
#endif

/** C99 `restrict`, spelled so the public headers also compile as C++. */
#ifdef __cplusplus
#define KF_RESTRICT __restrict
#else
#define KF_RESTRICT restrict
#endif

/** Largest square matrix the library handles: P (n x n) or S (m x m). */
#if KF_MAX_STATE > KF_MAX_MEAS
#define KF_MAX_DIM KF_MAX_STATE
#else
#define KF_MAX_DIM KF_MAX_MEAS
#endif

#endif
