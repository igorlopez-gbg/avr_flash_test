#ifndef _COMMON_H_
#define _COMMON_H_

#include <stdint.h>


#define TRK_HEADER_DATA_BYTES 6
#define TRK_BYTES 256

#define NELEMS(x)  (sizeof(x) / sizeof((x)[0]))

typedef __attribute__((__packed__)) enum { FALSE = 0, TRUE = !FALSE } boolean;

void my_nop(void);

#endif
