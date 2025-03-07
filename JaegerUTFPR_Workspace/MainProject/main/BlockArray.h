/*
 * BlockArray.h
 *
 *  Created on: May 18, 2023
 *      Author: Thoughtful
 */

#ifndef MAIN_BLOCKARRAY_H_
#define MAIN_BLOCKARRAY_H_

#ifndef PROJECT_DATA_TYPE
#define PROJECT_DATA_TYPE uint8_t//uint8_t //int16_t
#endif

//1
//2 <
//3 >
typedef struct
{
	PROJECT_DATA_TYPE *data;
} BlockData;

typedef struct {
	int nextBlockRead; //next block to read
	int nextBlockWrite; //next block to write
	int isFull;
	int blockSize;
	int numBlocks;
	//uint8_t *array;
	BlockData *array;
} BlockArray;

void blockArrayInit(BlockArray *blockArray, int blockSize, int numBlocks);
void blockArrayDeinit(BlockArray *blockArray);
void blockArrayClearData(BlockArray *blockArray);
inline int blockArrayIsFull(BlockArray *blockArray)
{
	return blockArray->isFull;
}
inline int blockArrayIsEmpty(BlockArray *blockArray)
{
	//If we are not full and the read block == write block
	if(blockArray->isFull == 0 && blockArray->nextBlockRead == blockArray->nextBlockWrite)
		return 1;
	else
		return 0;
}
inline uint32_t blockArrayBlocksToRead(BlockArray *blockArray)
{
	 if(blockArray->nextBlockWrite >= blockArray->nextBlockRead)
		 //if the read pointer is behind the speaker pointer
		 return blockArray->nextBlockWrite - blockArray->nextBlockRead;
	 else
		 //Otherwise, we went to the end of the queue and back to the beginning
		 return (blockArray->numBlocks - blockArray->nextBlockRead) + blockArray->nextBlockWrite;
		 //Example:
	 	 //r = 5
		 //w = 3
		 //m = 10
		 //position to read (10-5)+3 = 8
}
PROJECT_DATA_TYPE *blockArrayPeakData(BlockArray *blockArray);
PROJECT_DATA_TYPE *blockArrayReadData(BlockArray *blockArray);
int blockArrayWriteData(BlockArray *blockArray, PROJECT_DATA_TYPE *elements);

#endif /* MAIN_BLOCKARRAY_H_ */
