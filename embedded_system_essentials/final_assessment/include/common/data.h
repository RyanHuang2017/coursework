/**
 * @file course1.h 
 * @brief This file is to be used to course 1 final assessment.
 *
 * @author Sichuan Huang
 * @date 05-30-2023
 *
 */
#ifndef __DATA_H__
#define __DATA_H__

#include <stdint.h>

/**
 * @brief This function converts data from a standard integer type into an ASCII string.
 *
 * @param data data to be converted
 * @param ptr pointer to the location where the converted data is stored
 * @param base integer to indicate the base you wish the data to be converted to
 *
 * @return length of the converted data.
 */
uint8_t my_itoa(int32_t data, uint8_t * ptr, uint32_t base);

/**
 * @brief This function converts data back from an ASCII represented string into an integer type.
 *
 * @param ptr Pointer to source location
 * @param digits number of digits
 * @param base integer to indicate the base you wish the data to be converted to
 *
 * @return converted integer
 */
int32_t my_atoi(uint8_t * ptr, uint8_t digits, uint32_t base);


#endif
