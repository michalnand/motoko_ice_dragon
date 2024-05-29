#ifndef _GONIO_H_
#define _GONIO_H_

#include <stdint.h>

#define SINE_TABLE_SIZE ((uint32_t)1024)
#define SINE_TABLE_MAX  ((int32_t)512)


//angle in range 0..SINE_TABLE_SIZE-1, unsigned
//SINE_TABLE_MAX corresponds to 1 value
//-SINE_TABLE_MAX corresponds to -1 value

int32_t sin_tab(uint32_t angle);
int32_t cos_tab(uint32_t angle);
 
#endif