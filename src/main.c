/*
******************************************************************************
@Adi          :    Mustafa
@Soyadi       :    Ergül
@Tarih        :    09.07.2023
@Ciktilar     :
				10 ardisik carpim sonucu = 493807104 elemanlar baslangic 315'dan 324. elemana kadar 926 denemede buldu.
				13 ardisik carpim sonucu = 2091059712 elemanlar baslangic 438'dan 450. elemana kadar 923 denemede buldu.
				14 ardisik carpim sonucu = 2123366400 elemanlar baslangic 657'dan 670. elemana kadar 922 denemede buldu.
				15 ardisik carpim sonucu = 2099380224 elemanlar baslangic 651'dan 665. elemana kadar 921 denemede buldu.

******************************************************************************
*/

/*Serial Wire Viewer (SWV) output is activated for STM32F4. Commands such as printf and scanf have been output on the ITM Data Console.*/

#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>


int i; //Array scan variable
volatile int32_t ITM_RxBuffer = ITM_RXBUFFER_EMPTY; // Initialize as EMPTY
/* Private function prototypes */
void Maxs(const uint8_t* buffer, int length, int k, int* maxProduct, int* start, int* end); // Function Prototype
int _write(int file, char *ptr, int len); // ITM Data Console Function Prototype
int _read(int file, char *ptr, int len);  // ITM Data Console Function Prototype


int main(void)
{
	uint8_t buffer[1000] ={
	    6,4,0,7,7,1,7,6,5,3,1,3,3,0,6,2,4,9,1,9,2,2,5,1,1,9,6,7,4,4,2,6,5,7,4,7,4,2,3,5,5,3,4,9,1,9,4,9,3,4,
	    9,6,9,8,3,5,2,0,3,1,2,7,7,4,5,0,6,3,2,6,2,3,9,5,7,8,3,1,8,0,1,6,9,8,4,8,0,1,8,6,9,4,7,8,8,5,1,8,4,3,
	    8,5,8,6,1,5,6,0,7,8,9,1,1,2,9,4,9,4,9,5,4,5,9,5,0,1,7,3,7,9,5,8,3,3,1,9,5,2,8,5,3,2,0,8,8,0,5,5,1,1,
	    1,2,5,4,0,6,9,8,7,4,7,1,5,8,5,2,3,8,6,3,0,5,0,7,1,5,6,9,3,2,9,0,9,6,3,2,9,5,2,2,7,4,4,3,0,4,3,5,5,7,
	    6,6,8,9,6,6,4,8,9,5,0,4,4,5,2,4,4,5,2,3,1,6,1,7,3,1,8,5,6,4,0,3,0,9,8,7,1,1,1,2,1,7,2,2,3,8,3,1,1,3,
	    6,2,2,2,3,0,3,5,8,9,0,7,2,9,6,2,9,0,4,9,1,5,6,0,4,4,0,7,7,2,3,9,0,7,1,3,8,1,0,5,1,5,8,5,9,3,0,7,9,6,
	    0,8,6,6,7,0,1,7,2,4,2,7,1,2,1,8,8,3,9,9,8,7,9,7,9,0,8,7,9,2,2,7,4,9,2,6,5,7,2,7,3,3,3,0,0,1,0,5,3,3,
	    6,7,8,8,1,2,2,0,2,3,5,4,2,1,8,0,9,7,5,1,2,5,4,5,4,0,5,9,4,7,5,2,2,4,3,5,2,5,8,4,9,0,7,7,1,1,6,7,0,5,
	    5,6,0,1,3,6,0,4,8,3,9,5,8,6,4,4,6,7,0,6,3,2,4,4,1,5,7,2,2,1,5,5,3,9,7,5,3,6,9,7,8,1,7,9,7,7,8,4,6,1,
	    7,4,0,6,4,9,5,5,1,4,9,2,9,0,8,6,2,5,6,9,3,2,1,9,7,8,4,6,8,6,2,2,4,8,2,8,3,9,7,2,2,4,1,3,7,5,6,5,7,0,
	    5,6,0,5,7,4,9,0,2,6,1,4,0,7,9,7,2,9,6,8,6,5,2,4,1,4,5,3,5,1,0,0,4,7,4,8,2,1,6,6,3,7,0,4,8,4,4,0,3,1,
	    3,9,8,9,0,0,0,8,8,9,5,2,4,3,4,5,0,6,5,8,5,4,1,2,2,7,5,8,8,6,6,6,8,8,1,1,6,4,2,7,1,7,1,4,7,9,9,2,4,4,
	    4,2,9,2,8,2,3,0,8,6,3,4,6,5,6,7,4,8,1,3,9,1,9,1,2,3,1,6,2,8,2,4,5,8,6,1,7,8,6,6,4,5,8,3,5,9,1,2,4,5,
	    6,6,5,2,9,4,7,6,5,4,5,6,8,2,8,4,8,9,1,2,8,8,3,1,4,2,6,0,7,6,9,0,0,4,2,2,4,2,1,9,0,2,2,6,7,1,0,5,5,6,
	    2,6,3,2,1,1,1,1,1,0,9,3,7,0,5,4,4,2,1,7,5,0,6,9,4,1,6,5,8,9,6,0,4,0,8,0,7,1,9,8,4,0,3,8,5,0,9,6,2,4,
	    5,5,4,4,4,3,6,2,9,8,1,2,3,0,9,8,7,8,7,9,9,2,7,2,4,4,2,8,4,9,0,9,1,8,8,8,4,5,8,0,1,5,6,1,6,6,0,9,7,9,
	    1,9,1,3,3,8,7,5,4,9,9,2,0,0,5,2,4,0,6,3,6,8,9,9,1,2,5,6,0,7,1,7,6,0,6,0,5,8,8,6,1,1,6,4,6,7,1,0,9,4,
	    0,5,0,7,7,5,4,1,0,0,2,2,5,6,9,8,3,1,5,5,2,0,0,0,5,5,9,3,5,7,2,9,7,2,5,7,1,6,3,6,2,6,9,5,6,1,8,8,2,6,
	    7,0,4,2,8,2,5,2,4,8,3,6,0,0,8,2,3,2,5,7,5,3,0,4,2,0,7,5,2,9,6,3,4,5,0,
	    };

	  int k = 15 ; // The length of the consecutive product

	  if(k > sizeof(buffer)){
	  	  printf("HATA: Carpim uzunlugu dizi uzunlugundan daha buyuk olamaz");
	  }
	  else{
	  	  int maxProduct, start, end;
	  	  /*The first parameter of the Maxs function refers to the array buffer.
	  	   * The second parameter is the length of this array.
	  	   * The third parameter represents the length of the consecutive parameter that receives input from the user.
	  	   * The fourth parameter is the maximum size of consecutive numbers.
	  	   * Parameters 5 and 6 represent the starting and ending points of consecutive numbers, respectively.*/
	  	  Maxs(buffer, sizeof(buffer), k, &maxProduct, &start, &end);
	  	  printf("%d ardisik carpim sonucu = %d elemanlar baslangic %d'dan %d. elemana kadar %d denemede buldu.\n", k,maxProduct,start,end,i);
	  }

	  return 0;
}

void Maxs(const uint8_t* buffer, int length, int k, int* maxProduct, int* start, int* end) {
    *maxProduct = 0;
    *start = 0;
    *end = 0;

    for (i = 0; i <= length - k; i++) { //Since the length of the given array is equal to the variable length, here it is 1000
        int product = 1; //ineffective multiplier
        for (int j = i; j < i + k; j++) { // The for loop multiplies the number of consecutive numbers entered.
            product *= buffer[j];
        }
        if (product > *maxProduct) { //If the new value is greater than the maxProduct value, the product value is assigned to the variable maxProduct.
            *maxProduct = product;
            *start = i;
            *end = i + k - 1;
        }
    }
}
int _write(int file, char *ptr, int len)
{
	int DataIdx;

	for (DataIdx = 0; DataIdx < len; DataIdx++)
	{
		ITM_SendChar(*ptr++);
	}
	return len;
}
int _read(int file, char *ptr, int len)
{
	int DataIdx;

	for (DataIdx = 0; DataIdx < len; DataIdx++)
	{
		*ptr++ = ITM_ReceiveChar();
	}

return len;
}


void EVAL_AUDIO_TransferComplete_CallBack(uint32_t pBuffer, uint32_t Size){
  /* TODO, implement your code here */
  return;
}

/*
 * Callback used by stm324xg_eval_audio_codec.c.
 * Refer to stm324xg_eval_audio_codec.h for more info.
 */
uint16_t EVAL_AUDIO_GetSampleCallBack(void){
  /* TODO, implement your code here */
  return -1;
}
