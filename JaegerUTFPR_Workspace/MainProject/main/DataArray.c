/*
 * DataArray.c
 *
 *  Created on: Mar 15, 2023
 *      Author: Thoughtful
 */

#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "freertos/semphr.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "math.h"
#include "main_project.h"
#include "DataArray.h"




inline void dataArrayInit(DataArray *dataArray, uint32_t maxElements)
{
	dataArray->readPos = 0;
	dataArray->writePos = 0;
	dataArray->maxElements = maxElements;
	dataArray->array = malloc(dataArray->maxElements * sizeof(PROJECT_DATA_TYPE));
}
inline void dataArrayDeinit(DataArray *dataArray)
{
	if(dataArray->array != NULL)
		free(dataArray->array);
	dataArray->array = NULL;
}
inline void dataArrayClearData(DataArray *dataArray)
{
	dataArray->readPos = 0;
	dataArray->writePos = 0;
}
inline uint32_t dataArrayElementsToRead(DataArray *dataArray)
{
	 if(dataArray->writePos >= dataArray->readPos)
		 //if the read pointer is behind the speaker pointer
		 return dataArray->writePos - dataArray->readPos;
	 else
		 //Otherwise, we went to the end of the queue and back to the beginning
		 return (dataArray->maxElements - dataArray->readPos) + dataArray->writePos;
		 //Example:
	 	 //r = 5
		 //w = 3
		 //m = 10
		 //position to read (10-5)+3 = 8
}
inline uint32_t dataArraySpaceToWrite(DataArray *dataArray)
{
	return (dataArray->maxElements - dataArrayElementsToRead(dataArray))-1; //-1 because we cannot have posRead == posWrite in a full queue situation, since that this state mean the queue is empty
}
/*
inline uint8_t speakerArrayIncreaseRead(SpeakerArray *speakerArray)
{
	//if there is nothing to read
	if(speakerArray->readPos == speakerArray->writePos)
		return -1; //we should not increment the element if we at the write pos.

	speakerArray->readPos++;

	if(speakerArray->readPos == dataArray->maxElements)
		speakerArray->readPos = 0;

	return 1;
}*/

inline int16_t dataArrayRead(DataArray *dataArray)
{
	int16_t elementRead;

	//if there is nothing to read
	if(dataArray->readPos == dataArray->writePos)
		return -1; //we should not increment or read the element if we are at the write pos.

	//save the read element
	elementRead = dataArray->array[dataArray->readPos];

	dataArray->readPos++;

	if(dataArray->readPos == dataArray->maxElements)
		dataArray->readPos = 0;

	return elementRead;
}

inline uint8_t dataArrayReadToArray(DataArray *dataArray, PROJECT_DATA_TYPE *destArray, uint32_t maxElementsToRead)
{
	uint32_t totalElements;

	totalElements = dataArrayElementsToRead(dataArray);

	if(totalElements == 0)
		return 0; // 0 elements read

	//limit the total elements to read to the desired one
	if(totalElements > maxElementsToRead)
		totalElements = maxElementsToRead;

	//read every element in the array and save it at the destination
	for(int i = 0; i < totalElements; i++)
	{
		destArray[i] = dataArray->array[dataArray->readPos];

		//at this point, we will trust the speakerArrayElementsToRead returned value
		dataArray->readPos++;

		if(dataArray->readPos == dataArray->maxElements)
			dataArray->readPos = 0;
	}

	//return the total read elements
	return totalElements;
}

/*
inline uint8_t speakerArrayIncreaseWrite(SpeakerArray *speakerArray)
{
	uint32_t oldWritePost = speakerArray->writePos;

	speakerArray->writePos++;

	if(speakerArray->writePos == dataArray->maxElements)
		speakerArray->writePos = 0;

	//if, after summing 1 element we are at the write position, it means that we have no room left for the next element.
	//nothing to do other than restore the old write position and inform the caller (return 0)
	if(speakerArray->readPos == speakerArray->writePos)
	{
		speakerArray->writePos = oldWritePost;
		return 0;
	} else {
		return 1;
	}
}
*/
inline int16_t dataArrayAddElement(DataArray *dataArray, PROJECT_DATA_TYPE element)
{
	//write the element
	dataArray->array[dataArray->writePos] = element;

	//update the next write position
	uint32_t oldWritePost = dataArray->writePos;

	dataArray->writePos++;

	if(dataArray->writePos == dataArray->maxElements)
		dataArray->writePos = 0;

	//if, after summing 1 element we are at the read position, it means that we have no room left for the next element.
	//nothing to do other than restore the old write position and inform the caller (return -1).
	//Note that this means that we overwrite the last element and the queue is able to store  dataArray->maxElements-1 elements in practice
	if(dataArray->readPos == dataArray->writePos)
	{
		dataArray->writePos = oldWritePost;
		return -1;
	} else {
		return element;
	}
}
inline uint32_t dataArrayWriteFromArrayUINT8_T(DataArray *dataArray, uint8_t *elements, uint32_t elementsToWrite)
{
	//adc_digi_output_data_t *p;

	uint32_t spaceToWrite = dataArraySpaceToWrite(dataArray);
	if(elementsToWrite > spaceToWrite)
		elementsToWrite = spaceToWrite;

	//save every element in the DataArray
	//at this point, we will trust the dataArraySpaceToWrite returned value
	for(int i = 0; i < elementsToWrite; i++)
	{
		//write the element
		dataArray->array[dataArray->writePos] = elements[i];
        ////_printf(("array[%d] = %d, ", dataArray->writePos, dataArray->array[dataArray->writePos]);
		//if(dataArray->array[dataArray->writePos] > SPEAKER_RANGE)
	    //   //_printf(("OSR array[%d] = %d, ", dataArray->writePos, dataArray->array[dataArray->writePos]);

		dataArray->writePos++;

		if(dataArray->writePos == dataArray->maxElements)
			dataArray->writePos = 0;
	}
    ////_printf(("dataArray->writePos = %d, dataArray->readPos = %d ", dataArray->writePos, dataArray->readPos);

    //vTaskDelay(pdMS_TO_TICKS(1));
	return elementsToWrite;
}
inline uint32_t dataArrayWriteFromArrayINT16_T(DataArray *dataArray, int16_t *elements, uint32_t elementsToWrite)
{
	//adc_digi_output_data_t *p;

	uint32_t spaceToWrite = dataArraySpaceToWrite(dataArray);
	if(elementsToWrite > spaceToWrite)
		elementsToWrite = spaceToWrite;

	//save every element in the DataArray
	//at this point, we will trust the dataArraySpaceToWrite returned value
	for(int i = 0; i < elementsToWrite; i++)
	{
		//write the element
		dataArray->array[dataArray->writePos] = elements[i];
        ////_printf(("array[%d] = %d, ", dataArray->writePos, dataArray->array[dataArray->writePos]);
		//if(dataArray->array[dataArray->writePos] > SPEAKER_RANGE)
	    //   //_printf(("OSR array[%d] = %d, ", dataArray->writePos, dataArray->array[dataArray->writePos]);

		dataArray->writePos++;

		if(dataArray->writePos == dataArray->maxElements)
			dataArray->writePos = 0;
	}
    ////_printf(("dataArray->writePos = %d, dataArray->readPos = %d ", dataArray->writePos, dataArray->readPos);

    vTaskDelay(pdMS_TO_TICKS(1));
	return elementsToWrite;
}
/*
inline uint32_t dataArrayWriteFromArray2(DataArray *dataArray, SPEAKER_DATA_TYPE *elements, uint32_t elementsToWrite)
{
	uint32_t spaceToWrite = dataArraySpaceToWrite(dataArray);
	if(elementsToWrite > spaceToWrite)
		elementsToWrite = spaceToWrite;

	//save every element in the DataArray
	//at this point, we will trust the dataArraySpaceToWrite returned value
	for(int i = 0; i < elementsToWrite; i++)
	{
		//write the element
		dataArray->array[dataArray->writePos] = elements[i];
        ////_printf(("array[%d] = %d, ", dataArray->writePos, dataArray->array[dataArray->writePos]);
		//if(dataArray->array[dataArray->writePos] > SPEAKER_RANGE)
	    //   //_printf(("OSR array[%d] = %d, ", dataArray->writePos, dataArray->array[dataArray->writePos]);

		dataArray->writePos++;

		if(dataArray->writePos == dataArray->maxElements)
			dataArray->writePos = 0;
	}
    //_printf(("dataArray->writePos = %d, dataArray->readPos = %d ", dataArray->writePos, dataArray->readPos);

    vTaskDelay(pdMS_TO_TICKS(1000));
	return elementsToWrite;
}
*/
/**
 * @brief Write from an ADC input (save in an array) to the dataArray. Note that we also convert to voltage using the calibration data.
 *
 * @return
 *         - Number of wrote elements
 */
/*
inline uint32_t dataArrayWriteFromADCArray(DataArray *dataArray, PROJECT_DATA_TYPE *elements, uint32_t numRawElements, uint8_t numMeanValues,  esp_adc_cal_characteristics_t *adc1_chars)
{
	//Obtain the space to write considering that we only will save 1 every numMeanValues in the dataArray.
	uint32_t spaceToWrite = numMeanValues*dataArraySpaceToWrite(dataArray);
	if(numRawElements > spaceToWrite)
		numRawElements = spaceToWrite;

    uint32_t mean = 0, j = 0;
    adc_digi_output_data_t *p;
	//save every element in the DataArray
	//at this point, we will trust the dataArraySpaceToWrite returned value
	for(int i = 0; i < numRawElements; i++)
	{
		p = (void*)&elements[i];
		mean += esp_adc_cal_raw_to_voltage(p->type1.data, adc1_chars); //mean += p->type1.data;
		if(j < numMeanValues-1) //if numMeanValues==4, continue for 0, 1, 2. For j = 3, read and save the data.
		{
			j++;
			continue;
		}
		j = 0;
		mean /= numMeanValues;

		//write the element
		dataArray->array[dataArray->writePos] = mean;

		dataArray->writePos++;

		if(dataArray->writePos == dataArray->maxElements)
			dataArray->writePos = 0;
		mean = 0;
	}

	return numRawElements/numMeanValues;
}
*/
