/*
 * Motor.c
 *
 *  Created on: Feb 28, 2022
 *      Author: Thoughtful
 */
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "hal/gpio_types.h"
//#include "./managed_components/espressif__esp32-camera/driver/include/esp_camera.h"

#include "esp_camera.h"
#include "main_project.h"
#include "driver/rmt.h"
#include "driver/timer.h"

//#include "freertos/semphr.h"

/*

#define DIR_PORT 2
#define STEP_PORT 4
#define MOTOR_SEL0_PORT GPIO_NUM_14
#define MOTOR_SEL1_PORT GPIO_NUM_12
#define MOTOR_SEL2_PORT GPIO_NUM_13
#define MOTOR_SEL3_PORT GPIO_NUM_15
*/

static portMUX_TYPE motorsLock = portMUX_INITIALIZER_UNLOCKED;

static const char *TAG = "motor";

typedef struct MotorTimerTask
{
	bool enabled;
	timer_group_t groupId;
	timer_idx_t timerId;
	int motorId;
	int direction; //1 or -1
	//int ISRSet;
	uint32_t currentStep;
	uint32_t totalSteps;
	uint32_t stepSlowingDown;
	uint32_t initialTimerCount;
	uint32_t currentTimerCount;
	int dcCyclesOn; //used only for DC Motors
	int dcCyclesOff; //used only for DC Motors
	int valueLeftleg; //used only for DC Motors
	int valueRightLeg; //used only for DC Motors
} MotorTimerTask;

#if MOTOR_VERSION == 1
static const float steps_per_rev[12] =
{
		1,  //Used to obtain the information for the left leg
		20 * 165.3817f, //Camera Motor, no reduction
		1,  //DC motors, Used to obtain the information for the right leg
		20 * 165.3817f,
		20 * 165.3817f,
		2210,
		20 * 165.3817f,
		20 * 165.3817f,
		2210,
		1, //Not used
		1 //Not used
};

static const int32_t motorDefaultDirection[12] =
{
		-1,  //Used to obtain the information for the left leg
		1, //Camera Motor
		1,  //DC motor right leg, Used to obtain the information for the right leg
		1, //second motor, right
		-1, //first motor, right arm closer to the shoulder
		-1, //right arm
		-1, //second motor, left arm
		1, //first motor, left arm closer to the shoulder
		1, //left arm
		0, //Not used
		0 //Not used
};
#elif MOTOR_VERSION == 2

static const float steps_per_rev[12] =
{
		1,  //Used to obtain the information for the left leg
		20 * 165.3817f, //Camera Motor, no reduction
		1,  //DC motors, Used to obtain the information for the right leg
		20 * 165.3817f,
		2210,
		2210,
		20 * 165.3817f,
		2210,
		2210,
		1, //Not used
		1 //Not used
};

static const int32_t motorDefaultDirection[12] =
{
		-1,  //Used to obtain the information for the left leg
		1, //Camera Motor
		1,  //DC motor right leg, Used to obtain the information for the right leg
		1, //second motor, right
		-1, //first motor, right arm closer to the shoulder
		-1, //right arm
		-1, //second motor, left arm
		1, //first motor, left arm closer to the shoulder
		1, //left arm
		0, //Not used
		0 //Not used
};
#elif MOTOR_VERSION == 3

static const float steps_per_rev[12] =
{
		1,  //Used to obtain the information for the left leg -> MOTA_PIN -> J2_2
		2210, //Camera Motor
		1,  //DC motors, Used to obtain the information for the right leg -> MOTB_PIN -> J2_1
		2210, //Right Forearm
		2210, //Right arm
		2210, //Right arm, closer to the shoulder
		2210, //Left Forearm
		2210, //Left arm,
		2210, //Left arm, closer to the shoulder
		1, //Not used
		1 //Not used
};

static const int32_t motorDefaultDirection[12] =
{
		-1,  //Used to obtain the information for the left leg
		1, //Camera Motor
		1,  //DC motor right leg, Used to obtain the information for the right leg
		-1,
		-1,
		-1,
		1,
		1,
		1,
		0, //Not used
		0 //Not used
};
#endif

//State of all motors
Motor motors[TOTAL_MOTORS] = { 0 } ;

PinMask motorDirection[MAX_DIRECTION] = {0};
PinMask motorStages[4] = {0};

#define MOTOR_TIMERS_I 1 //1
#define MOTOR_TIMERS_J 2 //2
MotorTimerTask motorTimers[MOTOR_TIMERS_I][MOTOR_TIMERS_J] = {0};
volatile int motorAvailableTimers = 0;
volatile int isMotorRunning = false;
int totalMotorTimers = 0;

MotorState newMotorStates[12] = {0};

//MotorState motors[10] = {0};


int16_t lastRecCentiDegrees[TOTAL_MOTORS] = { 0 } ;

//QueueHandle_t motorQueue;






void configMotorPins()
{

    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = 0;
	io_conf.pin_bit_mask |= BIT64(S_0_PIN) | BIT64(S_1_PIN) | BIT64(S_2_PIN) | BIT64(MOTA_PIN) | BIT64(MOTB_PIN) | BIT64(MOTSLEEP_PIN);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    ESP_ERROR_CHECK(gpio_config(&io_conf));


	ESP_ERROR_CHECK(gpio_set_level(S_0_PIN, 0));
	ESP_ERROR_CHECK(gpio_set_level(S_1_PIN, 0));
	ESP_ERROR_CHECK(gpio_set_level(S_2_PIN, 0));
	ESP_ERROR_CHECK(gpio_set_level(MOTA_PIN, 0));
	ESP_ERROR_CHECK(gpio_set_level(MOTB_PIN, 0));
	ESP_ERROR_CHECK(gpio_set_level(MOTSLEEP_PIN, 0));

}



MotorTimerTask *obtainTask(int motorId, int direction, int totalSteps, int dcDutyCycle)
{
	////_ESP_LOGI(TAG, "motorAvailableTimers %d.", motorAvailableTimers);
	if(motorAvailableTimers == 0)
		return NULL;

	int i, j;
	char found = 0;
	for(i = 0; i < MOTOR_TIMERS_I; i++)
	{
		for(j = 0; j < MOTOR_TIMERS_J; j++)
		{
			if(!motorTimers[i][j].enabled)
			{
				found = 1;
				break;
			}
		}
		if(found)
			break;
	}
	//_ESP_LOGV(TAG, "Obtaining timer %d,%d, motorId %d.", i, j, motorId);
	//were we able to find a available tasK?
	if(!found)
	{
		//_printf(("No timer available! Should not happen here.");
		return NULL;
	}

	motorAvailableTimers--; //one less timer free

	motorTimers[i][j].enabled = 1;
	motorTimers[i][j].groupId = i;
	motorTimers[i][j].timerId = j;
	motorTimers[i][j].motorId = motorId;
	motorTimers[i][j].direction = direction;
	motorTimers[i][j].currentStep = 0;
	motorTimers[i][j].totalSteps = totalSteps;
	//motorTimers[i][j].ISRSet = 0;

	//are we dealing with the DC motors?
	if(motorTimers[i][j].motorId == 2)
	{
		dcDutyCycle++; // just so that 89 is translated to 90, resulting in 9 instead of 8 after division.
		//make sure the dcDutyCycle is within the limits
		if(dcDutyCycle < 1)
			dcDutyCycle = 1;
		else if(dcDutyCycle > 100)
			dcDutyCycle = 100;

		//MINIMALCOUNT cycles = 1ms (for 1000hz) or 1,25ms (for 800hz)
		//calculate the duty cycle
		//example: 10 * (80/(100.0f/(20/1.25f))) = 128 cycles. For 8kz (80M/10k (divider) 128 cycles =  16ms on (80%)
		//example: 10 * (20/(100.0f/(20/1.25f))) = 128 cycles. For 8kz (80M/10k (divider) 32 cycles =  4ms off (20%)

		motorTimers[i][j].dcCyclesOn = MINIMALCOUNT * (dcDutyCycle/(100.0f/(MSPERDCPERIOD/MSPERMINIMALCOUNT)));
		motorTimers[i][j].dcCyclesOff = MINIMALCOUNT * ((100.0f - dcDutyCycle)/(100/(MSPERDCPERIOD/MSPERMINIMALCOUNT)));
		ESP_LOGI(TAG, "DC: %d on, %d off\n", motorTimers[i][j].dcCyclesOn, motorTimers[i][j].dcCyclesOff);
	} else {

	    //we must start the counter at the first time at
		motorTimers[i][j].initialTimerCount = ACCELERATINGSTEPS+MINIMALCOUNT;
		motorTimers[i][j].currentTimerCount = motorTimers[i][j].initialTimerCount;

		//is the total steps enough to fully accelerate and deaccelerate?
		if(ACCELERATINGSTEPS*2 <= totalSteps)
			//then we can de accelerate normally at the end
			motorTimers[i][j].stepSlowingDown = totalSteps-ACCELERATINGSTEPS;
		else
		{
			//if not, we will have to star de accelerating at half of the total number of steps
			motorTimers[i][j].stepSlowingDown = totalSteps/2;
		}
		////_printf(("Group %d, timer %d obtained.\n", motorTimers[i][j].groupId, motorTimers[i][j].timerId);
	}
	return &motorTimers[i][j];
}

inline void motorSleep(MotorTimerTask *motorTask, bool notSleep)
{
	__asm("	NOP; NOP;  ");
	//First we must change the address
	*((int *)GPIO_OUT_REG) = (*((int *)GPIO_OUT_REG) & ~(motors[motorTask->motorId].address.clears)) | motors[motorTask->motorId].address.sets;

	//Wait ~ 15ns
	__asm("	NOP; NOP;  ");

	if(notSleep == 1)
	{
		*((int *)GPIO_OUT_W1TS_REG) = (1 << MOTSLEEP_PIN); // 1 = not sleeping
	} else if(notSleep == 0){
		*((int *)GPIO_OUT_W1TC_REG) = (1 << MOTSLEEP_PIN); // 0 = sleeping
	}
}


inline void motorSleepAndDirection(MotorTimerTask *motorTask, bool notSleep)
{
	__asm("	NOP; NOP;  ");
	//First we must change the address
	*((int *)GPIO_OUT_REG) = (*((int *)GPIO_OUT_REG) & ~(motors[motorTask->motorId].address.clears)) | motors[motorTask->motorId].address.sets;

	//Wait ~ 15ns
	__asm("	NOP; NOP; ");

	if(notSleep == 1)
	{
		*((int *)GPIO_OUT_REG) = (*((int *)GPIO_OUT_REG) & ~(motorDirection[motorTask->direction].clears)) | (motorDirection[motorTask->direction].sets | (1 << MOTSLEEP_PIN)); // 1 = not sleeping
	} else if(notSleep == 0){
		*((int *)GPIO_OUT_REG) = (*((int *)GPIO_OUT_REG) & ~(motorDirection[motorTask->direction].clears | (1 << MOTSLEEP_PIN))) | (motorDirection[motorTask->direction].sets); // 0 = sleeping
	}
}

inline void motorDCDirection(MotorTimerTask *motorTask)
{
	__asm("	NOP; NOP;  ");
	//First we must change the address
	*((int *)GPIO_OUT_REG) = (*((int *)GPIO_OUT_REG) & ~(motors[motorTask->motorId].address.clears)) | motors[motorTask->motorId].address.sets;

	//Wait ~ 15ns
	__asm("	NOP; NOP;  ");

	*((int *)GPIO_OUT_REG) = (*((int *)GPIO_OUT_REG) & ~(motorDirection[motorTask->direction].clears)) | (motorDirection[motorTask->direction].sets);

}

inline void updateMotor(MotorTimerTask *motorTask)
{
	__asm("	NOP; NOP;  ");
	//First we must change the address
	*((int *)GPIO_OUT_REG) = (*((int *)GPIO_OUT_REG) & ~(motors[motorTask->motorId].address.clears)) | motors[motorTask->motorId].address.sets;

	//Wait ~ 15ns
	__asm("	NOP; NOP;  ");

	*((int *)GPIO_OUT_REG) = (*((int *)GPIO_OUT_REG) & ~(motorStages[motors[motorTask->motorId].stage].clears)) | motorStages[motors[motorTask->motorId].stage].sets;

	//*((int *)GPIO_OUT_W1TS_REG) = pinMask2.sets;
	//*((int *)GPIO_OUT_W1TC_REG) = pinMask2.clears;

}
bool IRAM_ATTR motorTimerSteperISR(void * args)
{

	MotorTimerTask *motorTask = (MotorTimerTask *)args;

	//update the motor stage
	motors[motorTask->motorId].stage += motorTask->direction;
	if(motors[motorTask->motorId].stage > 3)
		motors[motorTask->motorId].stage = 0;
	else if(motors[motorTask->motorId].stage < 0)
		motors[motorTask->motorId].stage = 3;

	updateMotor(motorTask);


	motorTask->currentStep++;
	//are we slowing down?
	if(motorTask->currentStep >= motorTask->stepSlowingDown)
	{
		//did we finish already?
		if(motorTask->currentStep >= motorTask->totalSteps)
		{
			//stop the timer
		    timer_pause(motorTask->groupId, motorTask->timerId);
	        timer_disable_intr(motorTask->groupId, motorTask->timerId);
	    	timer_isr_callback_remove(motorTask->groupId, motorTask->timerId);
			//portENTER_CRITICAL_ISR(&motorsLock);
		    //portEXIT_CRITICAL_ISR(&motorsLock);
			////_ESP_LOGI(TAG, "Stopping timer %d, %d, resulting in %d left.", motorTask->groupId, motorTask->timerId, motorAvailableTimers);
			motorSleep(motorTask, false);
			motorTask->enabled = 0; //update this state
			motorAvailableTimers++; //one more timer free
			return true;
		}
		//slowing down = bigger value
		motorTask->currentTimerCount++;
		//cannot (and should not) be slower than the initial count (higher = slower)
		if(motorTask->currentTimerCount > motorTask->initialTimerCount)
			motorTask->currentTimerCount = motorTask->initialTimerCount;

	} else if(motorTask->currentStep <= ACCELERATINGSTEPS)
	{
		//speeding up = reducing count
		motorTask->currentTimerCount--;
		if(motorTask->currentTimerCount < MINIMALCOUNT)
			motorTask->currentTimerCount = MINIMALCOUNT;
	} else {
		motorTask->currentTimerCount = MINIMALCOUNT;
	}
    timer_set_counter_value(motorTask->groupId, motorTask->timerId, motorTask->currentTimerCount);


    return true;
}


//Compare if the current state of the DC motor is equal to the new state
inline int verifyDCMotorKeepGoing(MotorTimerTask *motorTask)
{
	if(motorTask->valueLeftleg == newMotorStates[0].centiDegrees && motorTask->valueRightLeg == newMotorStates[2].centiDegrees)
		return true;
	else
		return false;
}

bool IRAM_ATTR motorTimerDCsISR(void * args)
{

	MotorTimerTask *motorTask = (MotorTimerTask *)args;

	//start at full speed
	/*
	if(motorTask->currentStep < 2)
	{
		//turn on
		//motorSleep(motorTask, true);
		motorSleepAndDirection(motorTask, true);
		//update the time turned on
		timer_set_counter_value(motorTask->groupId, motorTask->timerId, MINIMALCOUNT * (100/(100.0f/MSPERDCPERIOD)));

	} else {
	*/
		//are the motor turned off?
		if(motors[motorTask->motorId].stage == 0)
		{
			//update state
			motors[motorTask->motorId].stage = 1;
			//turn on
			//motorSleep(motorTask, true);
			motorSleepAndDirection(motorTask, true);
			//update the time turned on
			timer_set_counter_value(motorTask->groupId, motorTask->timerId, motorTask->dcCyclesOn);
		} else {
			//update state
			motors[motorTask->motorId].stage = 0;
			//turn off
			//motorSleep(motorTask, false);
			motorSleepAndDirection(motorTask, false);
			//update the time turned on
			timer_set_counter_value(motorTask->groupId, motorTask->timerId, motorTask->dcCyclesOff);
		}
	//}

	motorTask->currentStep++;

	//did we finish already?
	if(motorTask->currentStep >= motorTask->totalSteps)
	{
		//should we keep going?
		if(verifyDCMotorKeepGoing(motorTask))
		{
			//reset the new value so that it wont keep going forever
			newMotorStates[0].centiDegrees = 0;
			newMotorStates[2].centiDegrees = 0;
			//reset the current step
			motorTask->currentStep = 0;
		} else {
			//stop the timer
			timer_pause(motorTask->groupId, motorTask->timerId);
			timer_disable_intr(motorTask->groupId, motorTask->timerId);
			timer_isr_callback_remove(motorTask->groupId, motorTask->timerId);
			motorTask->enabled = 0; //update this state
			motorAvailableTimers++; //one more timer free
			motorSleep(motorTask, false);
			return true;
		}
	}

    return false;
}
void startMotorTimer(MotorTimerTask *motorTask)
{
	//turn the motor on
	//motorSleep(motorTask, true);


    if(motorTask->motorId == 2)
    	timer_set_counter_value(motorTask->groupId, motorTask->timerId, motorTask->dcCyclesOff);
    else
    	timer_set_counter_value(motorTask->groupId, motorTask->timerId, motorTask->initialTimerCount);

    timer_set_alarm_value(motorTask->groupId, motorTask->timerId, 0);

    //portENTER_CRITICAL_SAFE(&motorsLock);
    /*
    if(motorTask->ISRSet == 1)
    {
    	timer_isr_callback_remove(motorTask->groupId, motorTask->timerId);
        timer_disable_intr(motorTask->groupId, motorTask->timerId);
        motorTask->ISRSet = 0;
    }*/


    if(motorTask->motorId == 2)
    {
        timer_isr_callback_add(motorTask->groupId, motorTask->timerId, motorTimerDCsISR, motorTask, 0);
        //motorTask->ISRSet = 1;
    }
    else
    {
        timer_isr_callback_add(motorTask->groupId, motorTask->timerId, motorTimerSteperISR, motorTask, 0);
        //motorTask->ISRSet = 1;
    }
    timer_enable_intr(motorTask->groupId, motorTask->timerId);
    //portEXIT_CRITICAL_SAFE(&motorsLock);

    timer_start(motorTask->groupId, motorTask->timerId);

    //update the flag indicating that the motor is running
	isMotorRunning = true;

}

void resetAllMotors()
{
	MotorTimerTask motorTask;

	for(int motorId = 1; motorId <= 8; motorId++)
	{
		motorTask.motorId = motorId;
		motorSleep(&motorTask, false);
		__asm("	NOP; NOP;  ");
		//vTaskDelay(1);
	}
}

void motorInit()
{
	int i;

	//configure motor pins
	configMotorPins();



    //create motor queues
    //motorQueue = xQueueCreate( MOTOR_QUEUE_SIZE, sizeof( QueueElement ) );

    timer_config_t config = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_DOWN,//TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = TIMER_AUTORELOAD_EN
    };

    isMotorRunning = false;
    motorAvailableTimers = 0;
    //update the number of available timers
	for(int i = 0; i < MOTOR_TIMERS_I; i++)
	{
		for(int j = 0; j < MOTOR_TIMERS_J; j++)
		{

		    timer_init(i, j, &config);

	    	//timer_isr_callback_remove(i, j);
	        //timer_disable_intr(i, j);
			motorTimers[i][j].enabled = 0;
			motorAvailableTimers++;
		}
	}
	//obtain the total number of timers we can use for motors
	totalMotorTimers = motorAvailableTimers;

    //set the default motor values
	for(i = 0; i < TOTAL_MOTORS; i++)
	{
		if(i == 0 || i == 2)
			motors[i].motorType = DC_MOTOR;
		else
			motors[i].motorType = STEP_MOTOR;

		//motors[i].defaultDirection = motorDefaultDirection[i];
		motors[i].currentState.centiDegrees = 0;
		//lastRecCentiDegrees[i] = 0;
		newMotorStates[i].centiDegrees = 0;
		motors[i].stage = 0;

#if MOTOR_VERSION != 2
		//configure the bits that results in the motor address
		switch(i)
		{
			case 1: // Q3 -> M1
				//val = GPIO_OUT_REG;
				motors[i].address.sets =  (1 << S_0_PIN) | (1 << S_1_PIN); //set
				motors[i].address.clears =  (1 << S_2_PIN); //clear
			break;
			case 2: // Q4 -> M2
				//Motor 2 represents 2x CC motors. In order to slowdown, it is necessary to use the sleep pin.
				motors[i].address.sets = (1 << S_2_PIN); //set
				motors[i].address.clears = (1 << S_0_PIN) | (1 << S_1_PIN); //clear
			break;
			case 3: // Q5 -> M3
				motors[i].address.sets =  (1 << S_0_PIN) | (1 << S_2_PIN); //set
				motors[i].address.clears =  (1 << S_1_PIN); //clear
			break;
			case 4: // Q6 -> M4
				motors[i].address.sets =  (1 << S_1_PIN) | (1 << S_2_PIN); //set
				motors[i].address.clears =  (1 << S_0_PIN); //clear
			break;
			case 5: // Q7 -> M5
				motors[i].address.sets =  (1 << S_0_PIN) | (1 << S_1_PIN) | (1 << S_2_PIN); //set
			break;
			case 6: // Q2 -> M6
				motors[i].address.sets = (1 << S_1_PIN); //set
				motors[i].address.clears = (1 << S_0_PIN) | (1 << S_2_PIN); //clear
			break;
			case 7: // Q1 -> M7
				motors[i].address.sets = (1 << S_0_PIN); //set
				motors[i].address.clears = (1 << S_1_PIN) | (1 << S_2_PIN); //clear
			break;
			case 8: // Q0 -> M8
				motors[i].address.clears =  (1 << S_0_PIN) | (1 << S_1_PIN) | (1 << S_2_PIN); //set
			break;
		}
	}
#elif MOTOR_VERSION == 2
	//configure the bits that results in the motor address
	switch(i)
	{
		case 1: // Q3 -> M1
			//val = GPIO_OUT_REG;
			motors[i].address.sets =  (1 << S_0_PIN) | (1 << S_1_PIN); //set
			motors[i].address.clears =  (1 << S_2_PIN); //clear
		break;
		case 2: // Q4 -> M2
			//Motor 2 represents 2x CC motors. In order to slowdown, it is necessary to use the sleep pin.
			motors[i].address.sets = (1 << S_2_PIN); //set
			motors[i].address.clears = (1 << S_0_PIN) | (1 << S_1_PIN); //clear
		break;
		case 3: // Q6 -> M3 //M3 and M4 are inverted
			motors[i].address.sets =  (1 << S_1_PIN) | (1 << S_2_PIN); //set
			motors[i].address.clears =  (1 << S_0_PIN); //clear
		break;
		case 4: // Q5 -> M4  //M3 and M4 are inverted
			motors[i].address.sets =  (1 << S_0_PIN) | (1 << S_2_PIN); //set
			motors[i].address.clears =  (1 << S_1_PIN); //clear
		break;
		case 5: // Q7 -> M5
			motors[i].address.sets =  (1 << S_0_PIN) | (1 << S_1_PIN) | (1 << S_2_PIN); //set
		break;
		case 6: // Q1 -> M6 //M6 and M7 are inverted
			motors[i].address.sets = (1 << S_0_PIN); //set
			motors[i].address.clears = (1 << S_1_PIN) | (1 << S_2_PIN); //clear
		break;
		case 7: // Q2 -> M7 //M6 and M7 are inverted
			motors[i].address.sets = (1 << S_1_PIN); //set
			motors[i].address.clears = (1 << S_0_PIN) | (1 << S_2_PIN); //clear
		break;
		case 8: // Q0 -> M8
			motors[i].address.clears =  (1 << S_0_PIN) | (1 << S_1_PIN) | (1 << S_2_PIN); //set
		break;
	}
}
#endif

	//configure stages to speed up the registe sets
	for(int i = 0; i < 4; i++)
	{
		switch(i)
		{
			case 0:
				motorStages[i].clears =  (1 << MOTA_PIN) | (1 << MOTB_PIN);
				motorStages[i].sets = (1 << MOTSLEEP_PIN); //make sure enable is on
				break;
			case 1:
				motorStages[i].clears =  (1 << MOTA_PIN);
				motorStages[i].sets =  (1 << MOTB_PIN) | (1 << MOTSLEEP_PIN);
				break;
			case 2:
				motorStages[i].sets =  (1 << MOTA_PIN) | (1 << MOTB_PIN) | (1 << MOTSLEEP_PIN);
				break;
			case 3:
				motorStages[i].clears =  (1 << MOTB_PIN);
				motorStages[i].sets =  (1 << MOTA_PIN) | (1 << MOTSLEEP_PIN);
				break;
		}
	}

	//configure the bits for the direction
	//2A -> left leg
	//2B -> right leg
	for(int i = 0; i < MAX_DIRECTION; i++)
	{
		switch(i)
		{
			//must take into account that left motor is inverted
			case FORWARD_DIRECTION:
				//both must move forwards
				motorDirection[i].clears  = (1 << MOTA_PIN) | (1 << MOTB_PIN); //clear both motors, since both legs is inverted
			break;
			case BACKWARD_DIRECTION:
				//both must move backwards
				motorDirection[i].sets = (1 << MOTA_PIN) | (1 << MOTB_PIN); //set both motors, since both legs are inverted
			break;
			case RIGHT_DIRECTION:
				//to turn right, move the left leg forward and the right leg backward
				motorDirection[i].sets = (1 << MOTB_PIN);
				motorDirection[i].clears  = (1 << MOTA_PIN); // since both legs is inverted
			break;
			case LEFT_DIRECTION:
				//to turn left, move the right leg forward and the left leg backward
				motorDirection[i].sets = (1 << MOTA_PIN);
				motorDirection[i].clears  = (1 << MOTB_PIN); // since both legs are inverted
			break;

		/*
			//must take into account that left motor is inverted
			case FORWARD_DIRECTION:
				//both must move forwards
				motorDirection[i].sets = (1 << MOTA_PIN);
				motorDirection[i].clears  = (1 << MOTB_PIN); // since right leg is inverted
			break;
			case BACKWARD_DIRECTION:
				//both must move backwards
				motorDirection[i].sets = (1 << MOTB_PIN); // since right leg is inverted
				motorDirection[i].clears  = (1 << MOTA_PIN);
			break;
			case RIGHT_DIRECTION:
				//to turn right, move the left leg forward and the right leg backward
				motorDirection[i].sets = (1 << MOTA_PIN) | (1 << MOTB_PIN); //set both motors, since right leg is inverted
			break;
			case LEFT_DIRECTION:
				//to turn left, move the right leg forward and the left leg backward
				motorDirection[i].clears  = (1 << MOTA_PIN) | (1 << MOTB_PIN); //clear both motors, since right leg is inverted
			break;
			*/
		}
	}


	//put all motors to sleep
	resetAllMotors();

    //_ESP_LOGI(TAG, "motorInit");
}

void motorDeinit()
{
    //_ESP_LOGI(TAG, "deinit");
}

void queueMotorSteps(char *recMessage) //recMessage after the type byte
{
	int i = 0; //index 0 is the message type
	//int angDif = 0 ;
    ////_ESP_LOGI(TAG, "Steps Rec");


	//Obtain all received values
    int16_t value;
	for(i = 0; i < TOTAL_MOTORS; i++)
	{
		value = (recMessage[(i*2) + 1] << 8) + recMessage[i*2];
		//angDif = value - lastRecCentiDegrees[i];
/*
		//if we are not dealing with the legs
		if(i != 0 && i != 2)
		{
			//is this equal to the last received angle for this motor?
			if(angDif*angDif <= 9) //3 = sqrt(9)
				continue;
		} else {
			//for the legs, 0 means no movement
			if(value == 0 && i == 0) //only for the left leg we can ignore the 0 value.
				//That is because sometimes only the  left leg value is sent, and we need the right one to perform the left moviment
				continue;
			//else
			//    //_ESP_LOGI(TAG, "Leg mov: %d\n", value);
		}*/

		newMotorStates[i].centiDegrees = value;

	}
	//_ESP_LOGI(TAG, "Rec UDP lf %d, %d, %d  - rg: %d, %d, %d.", motors[8].currentState.centiDegrees, motors[7].currentState.centiDegrees, motors[6].currentState.centiDegrees,	motors[5].currentState.centiDegrees, motors[4].currentState.centiDegrees, motors[3].currentState.centiDegrees);
}
/* Old version, using Queues
void queueMotorSteps(char *recMessage) //recMessage after the type byte
{
	int i = 0; //index 0 is the message type
	QueueElement queueElement;
	int angDif = 0 ;
    ////_ESP_LOGI(TAG, "Steps Rec");

    //Empty the queue
    xQueueReset(motorQueue);

	//Obtain all received values
    int16_t value;
	for(i = 0; i < TOTAL_MOTORS; i++)
	{
		value = (recMessage[(i*2) + 1] << 8) + recMessage[i*2];
		angDif = value - lastRecCentiDegrees[i];

		//if we are not dealing with the legs
		if(i != 0 && i != 2)
		{
			//is this equal to the last received angle for this motor?
			if(angDif*angDif <= 9) //3 = sqrt(9)
				continue;
		} else {
			//for the legs, 0 means no movement
			if(value == 0 && i == 0) //only for the left leg we can ignore the 0 value.
				//That is because sometimes only the  left leg value is sent, and we need the right one to perform the left moviment
				continue;
			//else
			//    //_ESP_LOGI(TAG, "Leg mov: %d\n", value);
		}

		lastRecCentiDegrees[i] = value;

		queueElement.motorIndex = i;
		queueElement.state.centiDegrees = value;
		//if(value != 0)
		//    //_printf(("mot %d, %d centidegrees\n", i, value);

		if(!xQueueSend(motorQueue, &queueElement, 0)) //dont block if the queue is full
		{
		    //_ESP_LOGI(TAG, "Motor queue is full");
		}
	}
}
*/
bool motorAlreadyRunning(int motorId)
{
	for(int i = 0; i < MOTOR_TIMERS_I; i++)
	{
		for(int j = 0; j < MOTOR_TIMERS_J; j++)
		{
			if(motorTimers[i][j].enabled && motorTimers[i][j].motorId == motorId)
				return true;
		}
	}
	return false;
}

void verifyMotorQueue()
{
	int32_t steps = 0;
	int32_t centidegreesDiference = 0;

	MotorTimerTask *motorTask;
	//int motorId;
	int direction;
	int dutyCycle = 0;
	int leftLegSteps = 0; //used to deal with the left leg while reading the motor 2 (right leg)

	//_ESP_LOGV(TAG, "MotorAvailableTimers %d.", motorAvailableTimers);

	//receive the queue element without blocking
	for(int motorIndex = 0; motorIndex < 12; motorIndex++)
	{
    	//if there is no timer free, we must wait
    	if(motorAvailableTimers == 0)
    	{
    		break;
    	}

		//moving forward (using right or left legs) does not consider the current angle
		if(motorIndex == 0 || motorIndex == 2)
		{
			//do not accept new tasks for motors that are still running
			if(motorAlreadyRunning(motorIndex))
				continue;

			steps =  (newMotorStates[motorIndex].centiDegrees); //no sense to use motorDefaultDirection[queueElement.motorIndex] here. The direction must be corrected at motor control

	    	if(steps > 100)
	    		steps = 100;
		} else {
			//is the new state equal to the current one?
			if(motors[motorIndex].currentState.centiDegrees == newMotorStates[motorIndex].centiDegrees)
				continue; //next

			//do not accept new tasks of motors that are still running
			if(motorAlreadyRunning(motorIndex))
				continue;

			//obtain the angle difference
			centidegreesDiference = (newMotorStates[motorIndex].centiDegrees - motors[motorIndex].currentState.centiDegrees);

			steps = motorDefaultDirection[motorIndex] * (centidegreesDiference * steps_per_rev[motorIndex])/36000; //360 * 100 = 36000
		}

	    //if we are dealing with motorID 0 (virtual, representing the left leg): just save the number of steps
	    if(motorIndex == 0)
	    {
	    	leftLegSteps = steps;
	    	continue;
	    } else if(motorIndex == 2)
	    {
			//DC motors must have their status reseted after reading to avoid continuous loop
			newMotorStates[0].centiDegrees = 0;
			newMotorStates[2].centiDegrees = 0;

	    	//update the value for left and right leg, so that we can easily verify if we should or shouldn't keep going at the ISR
			int valueLeftleg = leftLegSteps, valueRightleg = steps;

			dutyCycle = (abs(leftLegSteps)+abs(steps))/2;

		    //if there is no duty cycle?
		    if(dutyCycle == 0)
		    	continue;

		    //Note that the DC motors will keep running until we receive new angle
			//_ESP_LOGV(TAG, "Motor 0 and 2. leftLegSteps: %d, steps: %d.", leftLegSteps, steps);

			//the left leg would be saved at leftLegSteps

	    	 //no sense to use motorDefaultDirection[queueElement.motorIndex] here. The direction must be corrected at motor control
			if((leftLegSteps <= 0 && steps > 0) || (leftLegSteps < 0 && steps >= 0))
			{
				direction = LEFT_DIRECTION;
			}
			else if((leftLegSteps >= 0 && steps < 0) || (leftLegSteps > 0 && steps <= 0))
			{
				direction = RIGHT_DIRECTION;
			}
			else if(leftLegSteps < 0 && steps < 0)
			{
				direction = BACKWARD_DIRECTION;
			}
			else //if(leftLegSteps > 0 && steps > 0)
			{
				direction = FORWARD_DIRECTION;
			}

			//_ESP_LOGV(TAG, "Performing Direction %d, dutyCycle %d.", direction, dutyCycle);
    		//2 steps = MSPERDCPERIOD ms
    		steps = 2*500.0f/MSPERDCPERIOD; //500ms

    		ESP_LOGI(TAG, "DC mot %d, dir %d,  %d steps\n", motorIndex, direction, steps);

			////_printf(("Performing %d steps.\n", steps);

	    	motorTask = obtainTask(motorIndex, direction, steps, dutyCycle);

	    	if(motorTask == NULL)
	    	{
	    		//_ESP_LOGI(TAG, "Not enough timers left. Should not happen here!");
	    		break;
	    	}
	    	//update the value for left and right leg, so that we can easly verify if we should or shouldnt keep going at the ISR
	    	motorTask->valueLeftleg = valueLeftleg;
	    	motorTask->valueRightLeg = valueRightleg;

	    	//set the direction (DC)
	    	motorDCDirection(motorTask);

	    } else {

		    //if there is no difference or zero steps?
		    if(steps*steps < 9) //minimum of 3 (sqrt(9) = 3) steps
		    {
		    	continue;
		    }

	    	dutyCycle = 0; //not used for steppers

	    	//_ESP_LOGI(TAG, "deq mot %d, %d centidegrees, %d angdif\n", motorIndex,  newMotorStates[motorIndex].centiDegrees, centidegreesDiference);

			//update the new motor angle according to the number of steps we are really going to perform due to rounding errors
			motors[motorIndex].currentState.centiDegrees +=  motorDefaultDirection[motorIndex] * (steps * 36000)/steps_per_rev[motorIndex];

			//if the number of steps is smaller than 0
			if(steps < 0)
			{
				direction = -1;
				//step_motor_set_step(motorDriver, 1, STEP_MOTOR_DIRECTION_NEGATIVE);
				steps *= -1; //invert the steps sign
			}
			else
			{
				direction = 1;
				//step_motor_set_step(motorDriver, 1, STEP_MOTOR_DIRECTION_POSITIVE);
				//steps *= 1;
			}

			//_ESP_LOGD(TAG, "Performing %d steps.", steps);

	    	motorTask = obtainTask(motorIndex, direction, steps, dutyCycle);

	    	if(motorTask == NULL)
	    	{
	    		//_ESP_LOGE(TAG, "Not enough timers left. Should not happen here!");
	    		break;
	    	}

	    	// wake up (stepper)
	    	motorSleep(motorTask, true);

	    }

    	startMotorTimer(motorTask);
	}
	//verify if we have all timer available, meaning that there is no motor running
	if(motorAvailableTimers == totalMotorTimers)
	{
		isMotorRunning = false; //update the flag indicating that nothing is running
		//make sure all motors are sleeping
		resetAllMotors();
	}

	//if(steps != 0)
		////_ESP_LOGI(TAG, "lf %d, %d, %d  - rg: %d, %d, %d.", motors[8].currentState.centiDegrees, motors[7].currentState.centiDegrees, motors[6].currentState.centiDegrees,	motors[5].currentState.centiDegrees, motors[4].currentState.centiDegrees, motors[3].currentState.centiDegrees);
}


/*
 *

void verifyMotorQueue()
{
	QueueElement queueElement;
	int32_t steps = 0;
	int32_t centidegreesDiference = 0;

	MotorTimerTask *motorTask;
	//int motorId;
	int direction;
	int dutyCycle = 0;
	int leftLegSteps = 0; //used to deal with the left leg while reading the motor 2 (right leg)

	//if there is no timer free, we must wait
	if(motorAvailableTimers == 0)
		return;

	//receive the queue element without blocking
	while(xQueueReceive(motorQueue, &queueElement, 0))//while there is a element in the queue
	{
		//do not accept new casts of motors that are still running
		if(motorAlreadyRunning(queueElement.motorIndex))
		{
			//queue again this element
			if(!xQueueSend(motorQueue, &queueElement, 0)) //dont block if the queue is full
			    //_ESP_LOGI(TAG, "Motor queue is full");
			continue;
		}

		//moving forward (using right or left legs) does not consider the current angle
		if(queueElement.motorIndex == 0 || queueElement.motorIndex == 2)
		{
			//obtain the angle difference
			centidegreesDiference = queueElement.state.centiDegrees;

			steps =  (centidegreesDiference); //no sense to use motorDefaultDirection[queueElement.motorIndex] here. The direction must be corrected at motor control

	    	if(steps > 100)
	    		steps = 100;
		} else {
			//is the motor element equal to the queued one?
			if(motors[queueElement.motorIndex].currentState.centiDegrees == queueElement.state.centiDegrees)
				continue; //next element in the queue

			//obtain the angle difference
			centidegreesDiference = (queueElement.state.centiDegrees - motors[queueElement.motorIndex].currentState.centiDegrees);

			steps = motorDefaultDirection[queueElement.motorIndex] * (centidegreesDiference * steps_per_rev[queueElement.motorIndex])/36000; //360 * 100 = 36000
		}

	    //if we are dealing with motorID 0 (virtual, representing the left leg): just save the number of steps
	    if(queueElement.motorIndex == 0)
	    {
	    	leftLegSteps = steps;
	    	continue;
	    } else if(queueElement.motorIndex == 2)
	    {

			//the left leg would be saved at leftLegSteps

	    	 //no sense to use motorDefaultDirection[queueElement.motorIndex] here. The direction must be corrected at motor control
			if((leftLegSteps <= 0 && steps > 0) || (leftLegSteps < 0 && steps >= 0))
			{
				direction = LEFT_DIRECTION;
			}
			else if((leftLegSteps >= 0 && steps < 0) || (leftLegSteps > 0 && steps <= 0))
			{
				direction = RIGHT_DIRECTION;
			}
			else if(leftLegSteps < 0 && steps < 0)
			{
				direction = BACKWARD_DIRECTION;
			}
			else //if(leftLegSteps > 0 && steps > 0)
			{
				direction = FORWARD_DIRECTION;
			}
			dutyCycle = (abs(leftLegSteps)+abs(steps))/2;

			//dutyCycle = 2*dutyCycle/3; //limit the maximum duty cycle

		    //if there is no duty cycle?
		    if(dutyCycle == 0)
		    	continue;

    		//_printf(("Performin Direction %d, dutyCycle %d.\n", direction, dutyCycle);
    		//2 steps = MSPERDCPERIOD ms
    		steps = 2*900.0f/MSPERDCPERIOD; //900ms

			////_printf(("DC mot %d, %d steps\n", queueElement.motorIndex, queueElement.state.steps_for_dc);

			////_printf(("Performing %d steps.\n", steps);

	    	motorTask = obtainTask(queueElement.motorIndex, direction, steps, dutyCycle);

	    	if(motorTask == NULL)
	    	{
	    		//_printf(("Not enough timers left. Should not happen here!\n");
	    		return;
	    	}

	    	//set the direction (DC)
	    	motorDCDirection(motorTask);

	    } else {
		    //if there is no difference or zero steps?
		    if(steps*steps < 9) //minimum of 3 (sqrt(9) = 3) steps
		    {
		    	continue;
		    }

	    	dutyCycle = 0; //not used for steppers

			//_printf(("deq mot %d, %d centidegrees, %d angdif\n", queueElement.motorIndex, queueElement.state.centiDegrees, centidegreesDiference);

			//update the new motor angle according to the number of steps we are really going to perform due to rounding errors
			motors[queueElement.motorIndex].currentState.centiDegrees +=  motorDefaultDirection[queueElement.motorIndex] * (steps * 36000)/steps_per_rev[queueElement.motorIndex];

			//if the number of steps is smaller than 0
			if(steps < 0)
			{
				direction = -1;
				//step_motor_set_step(motorDriver, 1, STEP_MOTOR_DIRECTION_NEGATIVE);
				steps *= -1; //invert the steps sign
			}
			else
			{
				direction = 1;
				//step_motor_set_step(motorDriver, 1, STEP_MOTOR_DIRECTION_POSITIVE);
				//steps *= 1;
			}

			//_printf(("Performing %d steps.\n", steps);

	    	motorTask = obtainTask(queueElement.motorIndex, direction, steps, dutyCycle);

	    	if(motorTask == NULL)
	    	{
	    		//_printf(("Not enough timers left. Should not happen here!\n");
	    		return;
	    	}

	    	// wake up (stepper)
	    	motorSleep(motorTask, true);

	    }
    	startMotorTimer(motorTask);

    	//if there is no timer free, we must wait
    	if(motorAvailableTimers == 0)
    		break;
	}

	if(steps != 0)
		//_printf(("lf %d, %d, %d  - rg: %d, %d, %d\n", motors[6].currentState.centiDegrees, motors[5].currentState.centiDegrees, motors[4].currentState.centiDegrees,
												motors[9].currentState.centiDegrees, motors[8].currentState.centiDegrees, motors[7].currentState.centiDegrees);
}
    */
