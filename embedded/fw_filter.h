#ifndef FW_FILTER_H
#define FW_FILTER_H

/* Each firmware image links exactly one implementation of this interface. */

void filter_init(void);
int filter_step(int k);     /* predict + update with measurement row k; 0 on success */
const void *filter_x(void); /* state estimate: 4 words, float or Q-format int32 */

#endif
