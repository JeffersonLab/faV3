#pragma once
#include <stdlib.h>
#include <stdio.h>
#include <byteswap.h>

// example usage
//
//   unsigned int input=0x1234;
//   print_bit( (struct b_field *)&input);
//   print_hex(input);
//


struct b_field {
#ifdef VXWORKS
  unsigned b_31: 1;       unsigned b_30: 1;
  unsigned b_29: 1;       unsigned b_28: 1;
  unsigned b_27: 1;       unsigned b_26: 1;
  unsigned b_25: 1;       unsigned b_24: 1;
  unsigned b_23: 1;       unsigned b_22: 1;
  unsigned b_21: 1;       unsigned b_20: 1;
  unsigned b_19: 1;       unsigned b_18: 1;
  unsigned b_17: 1;       unsigned b_16: 1;
  unsigned b_15: 1;       unsigned b_14: 1;
  unsigned b_13: 1;       unsigned b_12: 1;
  unsigned b_11: 1;       unsigned b_10: 1;
  unsigned b_9: 1;        unsigned b_8: 1;
  unsigned b_7: 1;        unsigned b_6: 1;
  unsigned b_5: 1;        unsigned b_4: 1;
  unsigned b_3: 1;        unsigned b_2: 1;
  unsigned b_1: 1;        unsigned b_0: 1;
#else
  unsigned b_0 : 1;  unsigned b_1 : 1;  unsigned b_2 : 1;  unsigned b_3 : 1;
  unsigned b_4 : 1;  unsigned b_5 : 1;  unsigned b_6 : 1;  unsigned b_7 : 1;
  unsigned b_8 : 1;  unsigned b_9 : 1;  unsigned b_10: 1;  unsigned b_11: 1;
  unsigned b_12: 1;  unsigned b_13: 1;  unsigned b_14: 1;  unsigned b_15: 1;
  unsigned b_16: 1;  unsigned b_17: 1;  unsigned b_18: 1;  unsigned b_19: 1;
  unsigned b_20: 1;  unsigned b_21: 1;  unsigned b_22: 1;  unsigned b_23: 1;
  unsigned b_24: 1;  unsigned b_25: 1;  unsigned b_26: 1;  unsigned b_27: 1;
  unsigned b_28: 1;  unsigned b_29: 1;  unsigned b_30: 1;  unsigned b_31: 1;
#endif
};

void print_bit( struct b_field *bit );
void print_hex(unsigned int ihex);

void
print_bit( struct b_field *bit )
{

  printf("\n              28   24   20   16    12    8    4    0\n");
  printf("0x%08X: ", *(unsigned int *)bit);
  printf("%d%d%d%d %d%d%d%d %d%d%d%d %d%d%d%d  %d%d%d%d %d%d%d%d %d%d%d%d %d%d%d%d\n",
	 bit->b_31,bit->b_30,bit->b_29,bit->b_28,
	 bit->b_27,bit->b_26,bit->b_25,bit->b_24,
	 bit->b_23,bit->b_22,bit->b_21,bit->b_20,
	 bit->b_19,bit->b_18,bit->b_17,bit->b_16,
	 bit->b_15,bit->b_14,bit->b_13,bit->b_12,
	 bit->b_11,bit->b_10,bit->b_9,bit->b_8,
	 bit->b_7,bit->b_6,bit->b_5,bit->b_4,
	 bit->b_3,bit->b_2,bit->b_1,bit->b_0 );
}

void
print_hex(unsigned int ihex)
{
  printf("             0x%X  0x%X  0x%X  0x%X   0x%X  0x%X  0x%X  0x%X",
	 ((ihex)&0xF0000000)>>28,
	 ((ihex)&0x0F000000)>>24,
	 ((ihex)&0x00F00000)>>20,
	 ((ihex)&0x000F0000)>>16,

	 ((ihex)&0x0000F000)>>12,
	 ((ihex)&0x00000F00)>>8,
	 ((ihex)&0x000000F0)>>4,
	 ((ihex)&0x0000000F)>>0);
  printf("\n\n");

}
