/**
 * The copyrights, all other intellectual and industrial 
 * property rights are retained by XMOS and/or its licensors. 
 * Terms and conditions covering the use of this code can
 * be found in the Xmos End User License Agreement.
 *
 * Copyright XMOS Ltd 2011
 *
 * In the case where this code is a modification of existing code
 * under a separate license, the separate license terms are shown
 * below. The modifications to the code are still covered by the 
 * copyright notice above.
 *
 **/

#define SIN_TAB_SIZ 64

#if (SIN_TAB_SIZ != NUM_SIN_ANGS_IN_QUAD)
	#error Wrong Sine Table Size
#endif

short sine_table[SIN_TAB_SIZ+1] = {
      0 , 
    402 , 
    804 , 
   1205 , 
   1606 , 
   2006 , 
   2404 , 
   2801 , 
   3196 , 
   3590 , 
   3981 , 
   4370 , 
   4756 , 
   5139 , 
   5520 , 
   5897 , 
   6270 , 
   6639 , 
   7005 , 
   7366 , 
   7723 , 
   8076 , 
   8423 , 
   8765 , 
   9102 , 
   9434 , 
   9760 , 
  10080 , 
  10394 , 
  10702 , 
  11003 , 
  11297 , 
  11585 , 
  11866 , 
  12140 , 
  12406 , 
  12665 , 
  12916 , 
  13160 , 
  13395 , 
  13623 , 
  13842 , 
  14053 , 
  14256 , 
  14449 , 
  14635 , 
  14811 , 
  14978 , 
  15137 , 
  15286 , 
  15426 , 
  15557 , 
  15679 , 
  15791 , 
  15893 , 
  15986 , 
  16069 , 
  16143 , 
  16207 , 
  16261 , 
  16305 , 
  16340 , 
  16364 , 
  16379,
  16384
};

