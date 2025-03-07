/*
 * speaker.h
 *
 *  Created on: Mar 17, 2023
 *      Author: Thoughtful
 */

#ifndef MAIN_SPEAKER_H_
#define MAIN_SPEAKER_H_

#define SPEAKER_ON 1


//#define SPEAKER_LEDC 1
#define SPEAKER_RMT 1

void forceSpeakerDisable();
extern uint8_t playingSound;

//Speaker Related
#define SOUND_RESOLUTION LEDC_TIMER_8_BIT // LEDC_TIMER_9_BIT //9 bits  //10 bits = 2^10 = 1024, from 0 to 1023.
#define SOUND_RANGE (pow(2,SOUND_RESOLUTION)-1)
#define HALF_SOUND_RANGE (SOUND_RANGE/2)

#define SPEAKER_PIN 1//3//1//14=MOT0 //1=TX UART0 , 3=RX UART0, tests on #define S_0_PIN GPIO_NUM_14

#ifdef SPEAKER_LEDC
	#include "DataArray.h"
#include "main_project.h"

	//Default data type for DataType Structure

	#define LEDC_HS_TIMER          LEDC_TIMER_1
	#define LEDC_HS_CHANNEL        LEDC_CHANNEL_0
	#define LEDC_HS_MODE           LEDC_HIGH_SPEED_MODE
	#define LEDC_HS_GPIO       SPEAKER_PIN//14//1//(14)
	#define LEDC_FREQUENCY		   20000
	#define ISR_FUNCTION_DELAY 20 //in us, the time it takes to execute de ISR

	#define SPEAKER_ARRAY_MAX 12000
	#define SPEAKER_MIN_ELEMENTS_PLAY 4000
	void initSpeaker(DataArray *dataArray);
	void verifySpeakerQueue(DataArray *dataArray);
	void receiveSoundData(uint8_t *bytes, uint16_t length);
#endif
#ifdef SPEAKER_RMT

	#include "driver/rmt.h"
	#include "BlockArray.h"
	#include "main_project.h"

#if (UDP_CON == 1)
	#define SPEAKER_VALUES_PER_ITEM 1334  //1334 //limited by Ethernet/Wifi MTU
	#define SPEAKER_BLOCK_COUNT 9 //How many memory blocks there will be
#else
	#define SPEAKER_VALUES_PER_ITEM 4000  //1334 //limited by Ethernet/Wifi MTU
	#define SPEAKER_BLOCK_COUNT 4 //How many memory blocks there will be
#endif
	#define SPEAKER_TIMES_PER_SAMPLE 3 //what is the frequency multiplier to avoid the bip at the sound

	#define SPEAKER_MAX_BLOCKS_ADDED_ONCE 3 //Max blocks that will be converted to RMT_ITEMS at once
	#define RMT_ITEMS_LOOP_COUNT (SPEAKER_MAX_BLOCKS_ADDED_ONCE*SPEAKER_VALUES_PER_ITEM*SPEAKER_TIMES_PER_SAMPLE)+ 1
	#define RMT_TX_CHANNEL 0
	//#define SOUND_TIME_LENGTH_US 21334 //1000*1000*512/8000/3 = 21333.33333333333 uS
	typedef struct {
		rmt_item32_t *rmt_items_loop;
		uint32_t rmt_items_loop_count;
	} QueueSpeakerElement;

	typedef struct {
		//step_motor_t base;
		//QueueHandle_t speakerQueue;
		//SemaphoreHandle_t notify_semphr;
		BlockArray blockArray;
		QueueSpeakerElement queueElement;
	    int64_t nextSoundTime;
		//uint8_t isPlaying;
	} rmt_speaker_t;

	int addSound(rmt_speaker_t *speaker_t_ptr, uint8_t *newSound);
	void deinitSpeaker(rmt_speaker_t *speaker_t_ptr);
	void initSpeaker(rmt_speaker_t *speaker_t_ptr);
	//int verifyPlaySound(rmt_speaker_t *speaker_t_ptr);
	int verifyPlaySound(rmt_speaker_t *speaker_t_ptr, int forcePlay);
#endif


#endif /* MAIN_SPEAKER_H_ */
