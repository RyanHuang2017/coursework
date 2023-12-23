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
 * @file stats.h 
 * @brief This file includes all the declaration of functions needed for this study.
 *
 * @author Sichuan Huang
 * @date 05-16-2023
 *
 */
#ifndef __STATS_H__
#define __STATS_H__

/* Add Your Declarations and Function Comments here */ 

/**
 * @brief A function that prints the statistics of an array including minimum, maximum, mean, and median.
 *
 * @param pointer to a given array
 * @param size of the array
 * @param flag to indicate whether the array is sorted or not
 * 
 * @return
 */
void print_statistics(unsigned char* arr, int size, int* isSorted);

/**
 * @brief Given an array of data and a length, prints the array to the screen
 *
 * @param pointer to a given array
 * @param size of the array
 *
 * @return <Add Return Informaiton here>
 */
void print_array(unsigned char* arr, int size);

/**
 * @brief Given an array of data and a length, returns the median value
 *
 * @param pointer to a given array
 * @param size of the array
 * @param flag to indicate whether the array is sorted or not
 *
 * @return the median value of the given array
 */
float find_median(unsigned char* arr, int size, int* isSorted);

/**
 * @brief Given an array of data and a length, returns the mean
 *
 * @param pointer to a given array
 * @param size of the array
 *
 * @return the mean value of the given array
 */
float find_mean(unsigned char* arr, int size);

/**
 * @brief Given an array of data and a length, returns the maximum
 *
 * @param pointer to a given array
 * @param size of the array
 * @param flag to indicate whether the array is sorted or not
 *
 * @return the maximum value of the given array
 */
int find_maximum(unsigned char* arr, int size, int* isSorted);

/**
 * @brief Given an array of data and a length, returns the minimum
 *
 * @param pointer to a given array
 * @param size of the array
 * @param flag to indicate whether the array is sorted or not
 *
 * @return the minimum value of the given array
 */
int find_minimum(unsigned char* arr, int size, int* isSorted);

/**
 * @brief Given an array of data and a length, sorts the array from largest to smallest.
 *
 * @param pointer to a given array
 * @param size of the array
 * @param flag to indicate whether the array is sorted or not
 *
 * @return <Add Return Informaiton here>
 */
void sort_array(unsigned char* arr, int size, int* isSorted);

#endif /* __STATS_H__ */
