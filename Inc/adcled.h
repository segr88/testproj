/**
  ******************************************************************************
  * @file           : adcled.h
	* @author         : serg
  * @brief          : Header file 
  ******************************************************************************
*/

#ifndef ADCLED_H
#define ADCLED_H

/**
  * @brief Adcled-structure for storage state information of LED, AVG voltage value, ADC rate
  */
struct Adcled {
		int Led;              /**< State of LED, 0 - off, 1 - on.  */
		float AVG_V;          /**< result AVG voltage value.  */
		int AVGsum;           /**< variable for calculation AVG. */
		int AVGi;             /**< variable for calculation AVG. */
		int Rate;             /**< ADC rate value in Hz from 10 to 1000, settings by message "SET_ADC_SAMPLE_RATE". */
	  int Period;           /**< Period time for TIM3 calc from Rate: Period = 10000/ Rate. */
		int AVG_V_out;        /**< Flag on UART output AVG voltage value. */
		char stravg[32];      /**< text of value AVG voltage */
};

#ifdef __cplusplus
extern "C" {
#endif

int ALdoit(struct Adcled  *pAL, char *str);
void ALinit(struct Adcled  *pAL);
void add_ADC_Value(struct Adcled *pAL, int value);


#ifdef __cplusplus
}
#endif



#endif
