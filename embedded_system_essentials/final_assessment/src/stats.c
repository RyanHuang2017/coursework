/******************************************************************************
 * Copyright (C) 2017 by Alex Fosdick - University of Colorado
 *
 * Redistribution, modification or use of this software in source or binary
 * forms is permitted as long as the files maintain this copyright. Users are 
 * permitted to modify this and use it to learn about the field of embedded
 * software. Alex Fosdick and the University of Colorado are not liable for any
 * misuse of this material. 
 *
 *****************************************************************************/
/**
 * @file stats.c 
 * @brief <Add Brief Description Here >
 *
 * @author Sichuan Huang
 * @date 05-16-2023
 *
 */

#include <stdio.h>
#include "stats.h"

/* Size of the Data Set */
#define SIZE (40)

// int main() {

//   unsigned char test[SIZE] = { 34, 201, 190, 154,   8, 194,   2,   6,
//                               114, 88,   45,  76, 123,  87,  25,  23,
//                               200, 122, 150, 90,   92,  87, 177, 244,
//                               201,   6,  12,  60,   8,   2,   5,  67,
//                                 7,  87, 250, 230,  99,   3, 100,  90};

//   /* Other Variable Declarations Go Here */
//   /* Statistics and Printing Functions Go Here */
//   int isSorted = 0;
//   print_statistics(test, SIZE, &isSorted);
//   return 0;
// }

/* Add other Implementation File Code Here */
void print_statistics(unsigned char* arr, int size, int* isSorted){
  
  int min_array = find_minimum(arr, size, isSorted);
  int max_array = find_maximum(arr, size, isSorted);
  float median_array = find_median(arr, size, isSorted);
  float mean_array = find_mean(arr, size);

  printf("The minimum value of the array is: %.1d\n", min_array);
  printf("The maximum value of the array is: %.1d\n", max_array);
  printf("The mean value of the array is: %.1f\n", mean_array);
  printf("The median value of the array is: %.1f\n", median_array);
}

void print_array(unsigned char *arr, int size){
  #ifdef VERBOSE
    if ( arr == NULL){
      printf("empty array");
      return;
    }

    if (size <= 0){
      size = 1;
    }

    int i;
    for (i = 0; i<size; i++){
      printf("%hhu ", arr[i]);
    }
    printf("\n");
  #endif
  return;
}

float find_median(unsigned char* arr, int size, int* isSorted){
  
   if (arr == NULL){
    printf("empty array");
    return 0;
  }

  if (size <= 0){
    size = 1;
  }

  if (!isSorted){
    sort_array(arr, size, isSorted);
  }  

  // array size is odd
  if (size%2 != 0){
    return (float)arr[(size-1)/2];
  }
  // array size is even
  return 0.5*((float)arr[size/2] + (float)arr[size/2 + 1]);
}

float find_mean(unsigned char* arr, int size){
  if (arr == NULL){
    printf("empty array");
    return 0.0;
  }

  float mean = 0;
  for (int i = 0; i<size; i++){
    mean += (float)arr[i];
  }
  return mean/size;
}

int find_maximum(unsigned char* arr, int size, int* isSorted){
  
  if (arr == NULL){
    printf("empty array");
    return 0;
  }

  if (size <= 0){
    size = 1;
  }

  if(*isSorted){
    return arr[0];
  }

  unsigned char temp = *arr;
  for (int i = 0; i<size; i++){
    if ((int)*arr > (int)temp){
      temp = *arr;
    }
    arr ++;
  }
  return temp;
}

int find_minimum(unsigned char* arr, int size, int* isSorted){
  if (arr == NULL){
    printf("empty array");
  }

  if (size <= 0){
    size = 1;
  }

  if(*isSorted){
    return arr[size-1];
  }

  unsigned char temp = *arr;
  for (int i = 0; i<size; i++){
    if ((int)*arr < (int)temp){
      temp = *arr;
    }
    arr ++;
  }
  return temp;
}

void sort_array(unsigned char* arr, int size, int* isSorted){

  if (arr == NULL){
    printf("empty array");
  }

  if (size <= 0){
    size = 1;
  }

  if(*isSorted){
    printf("the data set is sorted.\n");
    return;
  }

  int temp = 0;
  for (int i = 0; i < size-1; i++){
    for (int j = 0; j < size-1; j++){
      if (arr[j] < arr[j+1]){
        temp = arr[j];
        arr[j] = arr[j+1];
        arr[j+1] = temp;
      }
    }
  }

  *isSorted = 1;
}
