/**
  ******************************************************************************
  * @file           : adcled.cpp
	* @author         : serg
  * @brief          : Parsing messages and calculation AVG voltage
  ******************************************************************************
*/

#include <string>
#include <cstdlib>
#include "adcled.h"


using namespace std;


/**
  * @brief Trimming spaces in string
	* @note  This function trimming spaces in begin and end of source string and return result string.
	* @param source string
  * @retval result string
  */
string trim(string s)
{
	if(!s.empty())
	{
		while(s[0]==' ') s.erase(0, 1);
		while(s[s.size()-1]==' ') s.erase(s.size()-1, 1);
	}
	return s;
}

/**
  * @brief Check string on number
	* @note  This function check string on accordance integer not negative number, 
  *        and return true if it accordancing, otherwise return false.
	* @param source string
  * @retval check result
  */
bool isnumber(string s)
{
	return !s.empty() && s.find_first_not_of("0123456789") == s.npos;
}


/**
  * @brief Parsing and perform incomming messages function
	* @note  This function read incomming messages and change states of variables of Adcled-structure for control periphery
	* @param variable of Adcled-structure
  * @param incomming message string
  * @retval number of type incomming message, or 0 if message incorrect
  */
int ALdoit(Adcled *pAL, char *str)
{
	string s(str);
	int i, k;
	int res = 0;
	if(s == "LED_ON"){pAL->Led = 1; res=1;}
	if(s == "LED_OFF"){pAL->Led = 0;res=2;}
	if(s == "GET_ADC_AVG_VOLTAGE")
	{
		sprintf(pAL->stravg, "%.3fV\n", pAL->AVG_V);
		pAL->AVG_V_out = 1;
		res=3;
	}
	if(s.find("SET_ADC_SAMPLE_RATE") != s.npos)
	{
		i = sizeof("SET_ADC_SAMPLE_RATE")-1;
		k = s.size();
		string par;
		par = trim(s.substr(i, k-i));
		if(isnumber(par))
		{
			int temp = atoi(par.c_str());
			if ((temp>=10)&&(temp<=1000))
			{
				res=4;
				pAL->Rate = temp;
				pAL->Period = (10000/(float)pAL->Rate + 0.5);
			}
		}
	}
	return res;
}



/**
  * @brief Init Adcled-structure
	* @param variable of Adcled-structure
  * @retval none
  */
void ALinit(Adcled *pAL)
{
	pAL->AVG_V = 0;
	pAL->AVG_V_out = 0;
	pAL->Led = 0;
	pAL->Rate = 1000;               //Hz
	pAL->Period = 10000/pAL->Rate;
	pAL->AVGsum = 0;
	pAL->AVGi = 0;
}


/**
  * @brief Calculation of AVG voltage value
  * @note  This function sum all ADC values for the last second, divide result sum by number of values
  *        conversion it to voltage and write value to AVG_V field of Adcled-structure
	* @param variable of Adcled-structure
  * @param ADC raw value
  * @retval none
  */
void add_ADC_Value(Adcled *pAL, int value)
{
	if(pAL->AVGi>=pAL->Rate) {
		pAL->AVG_V = 3.3*((float) pAL->AVGsum / (float) pAL->Rate)/4095;
		pAL->AVGi = 0;
		pAL->AVGsum = 0;
	}
	pAL->AVGsum += value;
	pAL->AVGi++;
}

