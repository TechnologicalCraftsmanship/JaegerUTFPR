/*
 * mic.c
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
#include "speaker.h"
#include "mic.h"


//#define MAX_BYTES_ADC_READ   256
#define MAX_ELEMENTS_ADC_READ   512
#define GET_UNIT(x)        ((x>>3) & 0x1)
#define ADC_RESULT_BYTE     2
static uint8_t samplingMic = 0;

#ifdef MIC_ON

static int16_t rawADCElements[MAX_ELEMENTS_ADC_READ];
static esp_adc_cal_characteristics_t adc1_chars;

static const char *TAG = "mic";

static inline void startSamplingMic()
{
	if(samplingMic == 0)
	{
		samplingMic = 1;
		adc_digi_start();
	}
}
static inline void stopSamplingMic()
{
	if(samplingMic == 1)
	{
		samplingMic = 0;
		adc_digi_stop();
	}
}




static void continuous_adc_init(uint16_t adc1_chan_mask, adc_channel_t channel, esp_adc_cal_characteristics_t *adc1_chars)
{



    adc_digi_init_config_t adc_dma_config = {
        .max_store_buf_size = 1024,
        .conv_num_each_intr = MAX_ELEMENTS_ADC_READ,
        .adc1_chan_mask = adc1_chan_mask,
        .adc2_chan_mask = 0, //not used
    };
    ESP_ERROR_CHECK(adc_digi_initialize(&adc_dma_config));

    adc_digi_configuration_t dig_cfg = {
        .conv_limit_en = 1, //For ESP32, this should always be set to 1
        .conv_limit_num = 250, //0 -> 255
        .sample_freq_hz = READINGS_PER_FINAL_SAMPLE * 8000,//20 * 1000,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1, //ESP32 only supports ADC1 DMA mode
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
    };

    adc_digi_pattern_config_t adc_pattern = {0};
    dig_cfg.pattern_num = 1;

	uint8_t unit = GET_UNIT(channel);
	uint8_t ch = channel & 0x7;
	adc_pattern.atten = ADC_ATTEN_DB_11; /*!<The input voltage of ADC will be attenuated extending the range of measurement by about 11 dB (3.55 x) */
	adc_pattern.channel = ch;
	adc_pattern.unit = unit;
	adc_pattern.bit_width = SOC_ADC_DIGI_MAX_BITWIDTH; //SOC_ADC_DIGI_MIN_BITWIDTH;

	////_ESP_LOGI(TAG, "adc_pattern.atten is :%x", adc_pattern.atten);
	////_ESP_LOGI(TAG, "adc_pattern.channel is :%x", adc_pattern.channel);
	////_ESP_LOGI(TAG, "adc_pattern.unit is :%x", adc_pattern.unit);
	esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 0, adc1_chars); //ADC_WIDTH_BIT_DEFAULT

    dig_cfg.adc_pattern = &adc_pattern;
    ESP_ERROR_CHECK(adc_digi_controller_configure(&dig_cfg));

}



/**
 * @brief Read from ADC buffer to result up to maximumElementsToRead elements.
 *
 * @return
 *         - Number of elements read.
 */
uint32_t readADC(int16_t *result, uint32_t maximumElementsToRead)
{
    esp_err_t ret;
    uint32_t totalBytesReaded = 0;


	ret = adc_digi_read_bytes((uint8_t *)result, maximumElementsToRead*ADC_RESULT_BYTE, &totalBytesReaded, ADC_MAX_DELAY); //1 = do not wait //ADC_MAX_DELAY
	if (ret == ESP_OK || ret == ESP_ERR_INVALID_STATE)
	{
		if(ret == ESP_ERR_INVALID_STATE)
		{
			//some conversion lost due to not being read fast enough
			////_printf(("ADC data lost.\n");
		//	return 0;
		}
		/*
    	uint32_t mean, numSamples;
		int j = 0;
		for (int i = 0; i < totalReaded; i += ADC_RESULT_BYTE)
		{
			mean = 0;
			if(j < READINGS_PER_FINAL_SAMPLE)
			{
				adc_digi_output_data_t *p = (void*)&result[i];
				mean += esp_adc_cal_raw_to_voltage(p->type1.data, &adc1_chars); //mean += p->type1.data;
				j++;
				continue;
			}

			j = 0;
			mean /= READINGS_PER_FINAL_SAMPLE;
			//_ESP_LOGI(TAG, "V: %d", mean);
			numSamples++;
		}*/
		////_printf(("ADC %d elements read.\n", totalBytesReaded/ADC_RESULT_BYTE);
		return totalBytesReaded/ADC_RESULT_BYTE;
	} else if (ret == ESP_ERR_TIMEOUT) {
		// no data avai
		////_ESP_LOGW(TAG, "No data, increase timeout or reduce conv_num_each_intr");
		//vTaskDelay(1000);
		//_ESP_LOGI(TAG, "ADC no data to read yet.\n");
		return 0;
	} else {
		//conversion lost due to not being read fast enought
		//_ESP_LOGI(TAG, "ADC error: %d.\n", ret);
		return 0;
	}

}

int32_t adjustADCSound(int16_t *samples, int32_t numRawElements, uint8_t numMeanValues, esp_adc_cal_characteristics_t *adc1_chars)
{///*
	//Determine if we should update the readSamplesAfterTrigger
	static int32_t readSamplesAfterTrigger = 0;

	uint32_t min = 10000, max = 0;
	uint32_t dif = 0;
	uint32_t i = 0;

	////_printf(("\n\n");
    uint32_t mean = 0, j = 0, totalConverted = 0;
    adc_digi_output_data_t *p;
	min = MIC_TRIGGER_RAW_MEAN-MIC_ADJUST_DIFFERENCE;
	max = MIC_TRIGGER_RAW_MEAN+MIC_ADJUST_DIFFERENCE;
	//save every element in the DataArray
	//at this point, we will trust the dataArraySpaceToWrite returned value
	for(int i = 0; i < numRawElements; i++)
	{
		p = (void*)&samples[i];
		//mean += p->type1.data;
		mean += esp_adc_cal_raw_to_voltage(p->type1.data, adc1_chars); //mean += p->type1.data;
		j++;
		////_printf(("mean1: %d\n", mean);

		if(j < numMeanValues) //if numMeanValues==4, continue for 0, 1, 2. For j = 3, read and save the data.
		{
			continue;
		}
		j = 0;
		mean /= numMeanValues;

		if(mean < min)
			min = mean;
		else if(mean > max)
			max = mean;

		//if(mean > MIC_TRIGGER_RAW_MEAN - MIC_ROUND_MEAN && mean < MIC_TRIGGER_RAW_MEAN + MIC_ROUND_MEAN)
		//	mean = MIC_TRIGGER_RAW_MEAN;

		if(readSamplesAfterTrigger != MIC_SAMPLES_AFTER_TRIGGER && (mean < MIC_TRIGGER_RAW_MEAN-MIC_TRIGGER_RAW_DIF || mean > MIC_TRIGGER_RAW_MEAN+MIC_TRIGGER_RAW_DIF))
		{
			////_printf(("Trigger: %d\n", mean);
			readSamplesAfterTrigger = MIC_SAMPLES_AFTER_TRIGGER;
		}

		samples[totalConverted] = mean;
		////_printf(("samples[%d]: %d\n", totalConverted, samples[totalConverted]);
		//if()
		////_printf(("%d ", samples[totalConverted]);
		mean = 0;
		totalConverted++;
	}

	//If there is no trigger, discard the samples
	if(readSamplesAfterTrigger <= 0)
	{
		return 0;
	}

	///*
	////_printf(("\n");
	//min = 0;
	//max = 3300;
	dif = max - min;
	//uint32_t sampleI1, sampleI2, sampleI3;
	//dif = (double)SPEAKER_RANGE/dif;
	if(dif == 0)
	{
		for(i = 0; i < totalConverted; i++)
		{
			samples[i] = SOUND_RANGE/2;
			////_printf(("samples[%d]: %d\n", i, samples[i]);
			////_printf(("Sample[%d]: %d mV\n", i, samples[i]);
		}
	} else {
		for(i = 0; i < totalConverted; i++)
		{
			//sampleI1 = samples[i];
			samples[i] = samples[i] - min;
			//sampleI2 = samples[i];
			samples[i] = ((double)samples[i]/dif)*SOUND_RANGE;

			if(samples[i] < 0)
				samples[i] = 0;
			else if(samples[i] > SOUND_RANGE)
				samples[i] = SOUND_RANGE;
			//sampleI3 = samples[i];

			//if(samples[i] > SPEAKER_RANGE)
			//{
				////_printf(("OSR: samples[%d]: %d, %d, %d\n", i, samples[i],  (int16_t)SPEAKER_RANGE, (dif));
				////_printf(("sampleI1: %d, sampleI2: %d, sampleI3: %d\n", sampleI1,  sampleI2, sampleI3);
			//} else {
			//	//_printf(("samples[%d]: %d\n", i, samples[i]);
			//}
			////_printf(("samples[%d]: %d\n", i, samples[i]);
			////_printf(("Sample[%d]: %d mV\n", i, samples[i]);

		}//
	}
	//*/

	readSamplesAfterTrigger -= totalConverted;
	return totalConverted;

}

void printSamples(DataArray *dataArray)
{
	//_printf(("\n\n");

	uint32_t totalElements;

	totalElements = dataArrayElementsToRead(dataArray);

	if(totalElements == 0)
		return; // 0 elements read

	//_printf(("totalElements: %d\n", totalElements);

	//read every element in the array and save it at the destination
	for(int i = 0; i < totalElements; i++)
	{
		//_printf(("%d ", dataArray->array[dataArray->readPos]);

		//at this point, we will trust the speakerArrayElementsToRead returned value
		dataArray->readPos++;

		if(dataArray->readPos == dataArray->maxElements)
			dataArray->readPos = 0;
		if(i % 1000 == 0)
	        vTaskDelay(pdMS_TO_TICKS(10));
	}
	//_printf(("\n");
	//return the total read elements
	return;
}


static void microphone_task(DataArray *dataArray)
{
    uint32_t elementsReaded = 0, totalElementsProcessed = 0, numElementsAttempt;


	//start sampling mic
	startSamplingMic();

	//ledc_fade_
	////_printf(("Sampling MIC.\n");

	//dataArrayClearData(&speakerArray);
	//totalElementsProcessed = 0;
	//vTaskDelay(pdMS_TO_TICKS(500));

	while(1)
	{
		///*
		//do not do anything while we are playing a sound, or if the sound is not active, or if there is a motor running
		if(playingSound == 1 || !soundActive || isMotorRunning)
		{
			////_printf((" e,l: %d, %d .", tmpReadElement, tmpLeft);
			stopSamplingMic();
			vTaskDelay(pdMS_TO_TICKS(200));
			continue;
		}//*/
		startSamplingMic();


		//see how many space we have left
		numElementsAttempt = READINGS_PER_FINAL_SAMPLE*dataArraySpaceToWrite(dataArray);
		////_printf(("Space left %d. \n", numElementsAttempt/READINGS_PER_FINAL_SAMPLE);

		//no room left to save
		if(numElementsAttempt == 0)
		{
			//nothing to do other than wait some task to consume the data
			vTaskDelay(pdMS_TO_TICKS(45));
			continue;
		}

		//cannot be larger than the available space
		if(numElementsAttempt > MAX_ELEMENTS_ADC_READ)
			numElementsAttempt = MAX_ELEMENTS_ADC_READ;

		elementsReaded = readADC(rawADCElements, numElementsAttempt);
		////_printf(("elementsReaded1  %d.\n", elementsReaded);

		if(elementsReaded > 0)
		{
			//convert the raw elements into the playable sound
			//note that this could be done in the compute, and we must do it here in order to play it now
			//the new  value of elementsReaded will be the old one / READINGS_PER_FINAL_SAMPLE.
			//the raw data will be converted also occupying only / READINGS_PER_FINAL_SAMPLE space.
			elementsReaded = adjustADCSound(rawADCElements, elementsReaded, READINGS_PER_FINAL_SAMPLE, &adc1_chars);

			//if we have a trigger
			if(elementsReaded > 0)
				totalElementsProcessed += dataArrayWriteFromArrayINT16_T(dataArray, rawADCElements, elementsReaded);
			else
			{
				//else,  sleep since there is no sound
				stopSamplingMic();
				vTaskDelay(pdMS_TO_TICKS(50));
			}

			//numElementsAttempt = dataArraySpaceToWrite(&speakerArray);
			////_printf(("Spac.Left %d, Read %d, proc %d.\n", numElementsAttempt, elementsReaded, totalElementsProcessed);
		}
		////_printf(("Space left %d, proc %d.", numElementsAttempt/READINGS_PER_FINAL_SAMPLE, totalElementsProcessed);

		vTaskDelay(pdMS_TO_TICKS(15));
	}

    //clear the ADC memory
    adc_digi_deinitialize();

}

void createMicrophoneTask(DataArray *dataArray)
{
    //Configure the ADC for the MIC
    continuous_adc_init(BIT(0), ADC1_CHANNEL_0, &adc1_chars);
    ////_printf(("Sampling MIC.\n");
    //vTaskDelay(pdMS_TO_TICKS(1000));
    xTaskCreate((TaskFunction_t) microphone_task, "microphone", 4096, (void*)dataArray, 5, NULL);
}

#endif
