/*
 * DataArray.h
 *
 *  Created on: Mar 16, 2023
 *      Author: Thoughtful
 */

#ifndef MAIN_DATAARRAY_H_
#define MAIN_DATAARRAY_H_


#define PROJECT_DATA_TYPE uint8_t//uint8_t //int16_t

typedef struct {
	uint32_t readPos; //next read position
	uint32_t writePos; //next write position
	uint32_t maxElements;
	//uint8_t *array;
	PROJECT_DATA_TYPE *array;
} DataArray;

void dataArrayInit(DataArray *dataArray, uint32_t maxElements);
void dataArrayDeinit(DataArray *dataArray);
void dataArrayClearData(DataArray *dataArray);
uint32_t dataArrayElementsToRead(DataArray *dataArray);
uint32_t dataArraySpaceToWrite(DataArray *dataArray);
int16_t dataArrayRead(DataArray *dataArray);
uint8_t dataArrayReadToArray(DataArray *dataArray, PROJECT_DATA_TYPE *destArray, uint32_t maxElementsToRead);
int16_t dataArrayAddElement(DataArray *dataArray, PROJECT_DATA_TYPE element);
uint32_t dataArrayWriteFromArrayUINT8_T(DataArray *dataArray, uint8_t *elements, uint32_t elementsToWrite);
uint32_t dataArrayWriteFromArrayINT16_T(DataArray *dataArray, int16_t *elements, uint32_t elementsToWrite);
//uint32_t dataArrayWriteFromADCArray(DataArray *dataArray, PROJECT_DATA_TYPE *elements, uint32_t numRawElements, uint8_t numMeanValues,  esp_adc_cal_characteristics_t *adc1_chars);


#endif /* MAIN_DATAARRAY_H_ */
