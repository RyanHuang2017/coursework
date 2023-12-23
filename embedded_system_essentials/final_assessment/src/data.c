/**
 * @file data.c
 * @brief this file contains functions to realize interchange between ascii char
 * and standard integer type
 *
 *
 * @author Sichuan Huang
 * @date 05-30-2023
 *
 */
#include "../include/common/data.h"

uint8_t my_itoa(int32_t data, uint8_t * ptr, uint32_t base){
  uint8_t length = 0;
  int sign;

  if (data < 0){
    sign = data;
    data = -data;
  }
  // conversion
  do {
      *ptr = (data % base) + '0';
      if (base == 16 && data%base >= 10){
        *ptr = (data % base) +'7';
      }
      length ++;
      ptr++;
      data = data/base;
  }while(data/base > 0);

  *ptr = (data % base) + '0';
  if (base == 16 && data%base > 10){
    *ptr = (data % base) +'7';
  }
  length++;
  ptr++;
  // add sign and null terminator
  if (sign < 0){
    *ptr = '-';
    length++;
    ptr++;
  } 
  *ptr = '\0';
  // reverse the order
  ptr = ptr - length;
  uint8_t tmp;
  for (int i = 0; i < length/2; i++){
    tmp = *(ptr + i);
    *(ptr + i) = *(ptr + length - 1 - i);
    *(ptr + length - 1 - i) = tmp;
  }

  return length;
}

int32_t my_atoi(uint8_t * ptr, uint8_t digits, uint32_t base){
  int32_t data = 0;
  uint8_t* tmp;
  int sign = 1;

  if (*ptr == '-'){
    sign = -1;
    digits = digits - 1;
    tmp = ptr + 1;
  } else {
    tmp = ptr;
  }

  for (uint8_t i = 0; i < digits; i++){
    if (base == 16 && *(tmp + i) >= 65){
      data = data * base + (*(tmp + i) - '7');
    }
    else{
      data = data * base + (*(tmp + i) - '0');
    }
  }
  data = data * sign;

  return data;
}