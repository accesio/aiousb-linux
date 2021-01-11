#include <stdio.h>
#include "aiousb.h"

int main (int argc, char **argv)
{
  int status = 0;
  status = AiousbInit();

  printf("status = %d\n", status);

  status = ADC_SetScanLimits(0, 0, 15);

  printf("status = %d\n", status);

  ADC_SetOversample(0, 5);

  double data[16] = { 5 };
  status = ADC_GetScanV(0, data);

  printf("status = %d\n", status);

  int i = 0;

  for ( i = 0 ; i < 16 ; i++)
  {
    printf("%f\n", data[i]);
  }


  return status;

}
