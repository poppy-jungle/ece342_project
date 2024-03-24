#include "config.h"
#include "main.h"
#include <string.h>

extern UART_HandleTypeDef huart3;

void print_msg(char * msg) {
  HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), 100);
}

int twosCompToInt(uint8_t value) {
	if (value & 0x80) {
		return -((~value + 1) & 0xFF);
   } else {
		return value;
   }
}

int twosCompToInt_10bit(uint16_t value) {
	if (value & 0x200) {
		return -((~value + 1) & 0xFF);
   } else {
		return value;
   }
}

int twosCompToInt_16bit(uint16_t value) {
	if (value & 0x8000) {
		return -((~value + 1) & 0xFF);
   } else {
		return value;
   }
}