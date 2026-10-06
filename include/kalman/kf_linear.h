#ifndef KF_LINEAR_H
#define KF_LINEAR_H

#include "kalman/kf_types.h"

int kf_init(kf_state *kf, int n, int m);
int kf_predict(kf_state *kf, const kf_real *F);
int kf_update(kf_state *kf, const kf_real *z, const kf_real *H);

#endif
