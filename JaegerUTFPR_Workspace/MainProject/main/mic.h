/*
 * mic.h
 *
 *  Created on: Mar 16, 2023
 *      Author: Thoughtful
 */

#ifndef MAIN_MIC_H_
#define MAIN_MIC_H_

#include "DataArray.h"


#define MIC_ON 1
#define READINGS_PER_FINAL_SAMPLE 3

#define MIC_TRIGGER_RAW_MEAN 1650//from 0 to 3300, with mean 1650
//#define MIC_ROUND_MEAN 20 //+-20 around the mean will be rounded
#define MIC_TRIGGER_RAW_DIF 200//200//from 0 to 3300, with mean 1650, the distance to 1650.

#define MIC_ADJUST_DIFFERENCE 256//


//Triggers to start listening
#define MIC_SAMPLES_AFTER_TRIGGER  (12000) //300ms = 0.5s * 8000 = 4000
#define MIC_SAMPLES_TO_SEND  (4000) //300ms = 0.5s * 8000 = 4000

#define MIC_ARRAY_MAX 12000

void createMicrophoneTask(DataArray *dataArray);


#endif /* MAIN_MIC_H_ */
