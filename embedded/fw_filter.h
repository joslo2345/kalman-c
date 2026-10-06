#ifndef FW_FILTER_H
#define FW_FILTER_H

/* Each firmware image links exactly one implementation of this interface. */

void filter_init(void);
int filter_step(const float *z); /* predict + update; 0 on success */
const float *filter_x(void);     /* current state estimate (4 values) */

#endif
