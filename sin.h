
#include "arm_common_tables.h"


float sin_f32(float x){
 
	float sinVal, fract;                   /* Temporary input, output variables */
  uint16_t index;                                /* Index variable */
  float32_t a, b;                                /* Two nearest output values */
  float32_t findex;
	
	unsigned int n = (unsigned int)x;    // int value

  /* Map input value to [0 1] */
  x -= (float)n;

  /* Calculation of index of the table */
  findex = (float32_t)FAST_MATH_TABLE_SIZE * x;
  index = (uint16_t)findex;
	
  /* fractional value calculation */
  fract = findex - (float32_t) index;

  /* Read two nearest values of input value from the sin table */
  a = sinTable_f32[index];
  b = sinTable_f32[index+1];

  /* Linear interpolation process */
  sinVal = (1.0f - fract) * a + fract * b;

  /* Return output value */
  return (sinVal);
}

