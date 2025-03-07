/*
 * BlockArray.c
 *
 *  Created on: May 18, 2023
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
#include "BlockArray.h"


static const char *TAG = "Block_Array";

void blockArrayInit(BlockArray *blockArray, int blockSize, int numBlocks)
{
	blockArrayClearData(blockArray);
	blockArray->blockSize = blockSize;
	blockArray->numBlocks = numBlocks;
	blockArray->array = malloc(blockArray->numBlocks * sizeof(BlockData));
    if(blockArray->array == NULL)
    {
	    //_ESP_LOGE(TAG,"Could block array. OOM.");
	    return;
    }

	for(int i = 0; i < blockArray->numBlocks; i++)
	{
		blockArray->array[i].data = malloc(blockArray->blockSize * sizeof(PROJECT_DATA_TYPE));
	    if(blockArray->array[i].data == NULL)
	    {
		    //_ESP_LOGE(TAG,"Could block array data. OOM.");
		    return;
	    }
	}
}
void blockArrayDeinit(BlockArray *blockArray)
{
	if(blockArray->array == 0)
		return;

	for(int i = 0; i < blockArray->numBlocks; i++)
	{
		if(blockArray->array[i].data != NULL)
			free(blockArray->array[i].data);
		blockArray->array[i].data = NULL;
	}
	free(blockArray->array);
	blockArray->array = NULL;

	blockArrayClearData(blockArray);
}
void blockArrayClearData(BlockArray *blockArray)
{
	blockArray->nextBlockRead = 0;
	blockArray->nextBlockWrite = 0;
	blockArray->isFull = 0;
}
inline int blockArrayCalculateNextWriteBlock(BlockArray *blockArray)
{
	int next = blockArray->nextBlockWrite + 1;
	if(next == blockArray->numBlocks)
		next = 0;
	return next;
}
//inline int blockArrayIsFull(BlockArray *blockArray)
//{
//	return blockArray->isFull;
//}
//inline int blockArrayIsEmpty(BlockArray *blockArray)
//{
//	//If we are not full and the read block == write block
//	if(blockArray->isFull == 0 && blockArray->nextBlockRead == blockArray->nextBlockWrite)
//		return 1;
//	else
//		return 0;
//}
PROJECT_DATA_TYPE *blockArrayPeakData(BlockArray *blockArray)
{
	//return null if there is nothing to read
	if(blockArrayIsEmpty(blockArray))
		return NULL;
	return blockArray->array[blockArray->nextBlockRead].data;
}

PROJECT_DATA_TYPE *blockArrayReadData(BlockArray *blockArray)
{
	//return null if there is nothing to read
	if(blockArrayIsEmpty(blockArray))
		return NULL;
	int readBlock = blockArray->nextBlockRead;

	blockArray->nextBlockRead++;
	if(blockArray->nextBlockRead ==  blockArray->numBlocks)
		blockArray->nextBlockRead = 0;

	//after reading, for sure we are not full
	blockArray->isFull = false;
	return blockArray->array[readBlock].data;
}
int blockArrayWriteData(BlockArray *blockArray, PROJECT_DATA_TYPE *elements)
{
	//return falase  if the blockarray is full
	if(blockArrayIsFull(blockArray))
		return false;
	memcpy(blockArray->array[blockArray->nextBlockWrite].data, elements, blockArray->blockSize);

	//increment the next block to write
	blockArray->nextBlockWrite++;
	if(blockArray->nextBlockWrite ==  blockArray->numBlocks)
		blockArray->nextBlockWrite = 0;

	//if we got to the same position as read, we are full
	if(blockArray->nextBlockWrite ==  blockArray->nextBlockRead)
		blockArray->isFull = true;
	return true;
}
