

#ifndef MAIN_PROJECT
#define MAIN_PROJECT 1


//Default data type for DataType Structure
#define PROJECT_DATA_TYPE uint8_t//uint8_t //int16_t


#define TOTAL_MOTORS 12
#define MOTOR_QUEUE_SIZE 24

#define MOTOR_VERSION 3
#define ESP_JAEGER_VERSON 5


//#define ORI_STEPS_PER_REVOLUTION 20
//#define REDUCTION (2.0f*2*2*2)

//#define STEPS_PER_REVOLUTION (ORI_STEPS_PER_REVOLUTION * REDUCTION)


//
#define PORT 3389//3389
#define MAX_PACKET  4400
#define CONFIG_EXAMPLE_IPV4 1
//#define CONFIG_EXAMPLE_IPV6 1
//
//#define UDP_CON 1
#define TCP_CON 1
extern char conBuffer[MAX_PACKET];

#define BOARD_ESP32CAM_AITHINKER 1




/*
 *V5
CAM_PWR - Changed from 32 to 33
CSI_D4 - Changed from 36(SENSOR_VP) to 32
LED was removed from PCB (old 33)
MIC_IN - Added to 36 (SENSOR_VP)
 */
#if ESP_JAEGER_VERSON == 5

//V5: LED was removed from PCB (old 33)
//v5: MIC_IN - Added to 36 (SENSOR_VP)

#define CAM_PIN_PWDN 33 //CAM_PWR - Changed from 32 to 33
#define CAM_PIN_RESET -1 //software reset will be performed
#define CAM_PIN_XCLK 0
#define CAM_PIN_SIOD 26
#define CAM_PIN_SIOC 27

#define CAM_PIN_D7 35
#define CAM_PIN_D6 34
#define CAM_PIN_D5 39
#define CAM_PIN_D4 32 //CSI_D4 - Changed from 36(SENSOR_VP) to 32.  36 (SENSOR_VP)  will be used for the MIC.
#define CAM_PIN_D3 21
#define CAM_PIN_D2 19
#define CAM_PIN_D1 18
#define CAM_PIN_D0 5
#define CAM_PIN_VSYNC 25
#define CAM_PIN_HREF 23
#define CAM_PIN_PCLK 22

#else

#define CAM_PIN_PWDN 32 //TODO: Mudar para 33 com DEFINE
#define CAM_PIN_RESET -1 //software reset will be performed
#define CAM_PIN_XCLK 0
#define CAM_PIN_SIOD 26
#define CAM_PIN_SIOC 27

#define CAM_PIN_D7 35
#define CAM_PIN_D6 34
#define CAM_PIN_D5 39
#define CAM_PIN_D4 36 TODO: ARRUMAR com denife! Na nova placa, deve ser 32 (antigo LED). O  36 (SENSOR_VP) será usado para o MIC
#define CAM_PIN_D3 21
#define CAM_PIN_D2 19
#define CAM_PIN_D1 18
#define CAM_PIN_D0 5
#define CAM_PIN_VSYNC 25
#define CAM_PIN_HREF 23
#define CAM_PIN_PCLK 22

#endif


// Motors Defines/Enums/Structures
// GPIO configuration
#define S_0_PIN GPIO_NUM_14
#define S_1_PIN GPIO_NUM_12
#define S_2_PIN GPIO_NUM_13
#define MOTA_PIN GPIO_NUM_15
#define MOTB_PIN GPIO_NUM_2
#define MOTSLEEP_PIN GPIO_NUM_4

typedef struct PinMask
{
	uint32_t sets;
	uint32_t clears;
} PinMask;

#define TIMER_DIVIDER   (10000) // 80M/10k = 8khz
#define ACCELERATINGSTEPS 40 //40 steps to speed up/speed down
//#define MINIMALCOUNT ((TIMER_BASE_CLK / (TIMER_DIVIDER))/1000) //for clk = 80M, divider=10k, we have 80m/(10k*1k) = 8 // for 1000hz maximum
#define MINIMALCOUNT ((TIMER_BASE_CLK / (TIMER_DIVIDER))/800) //for clk = 80M, divider=10k, we have 80m/(10k*800) = 10 // for 800hz maximum

#define MSPERMINIMALCOUNT (1000.0f/((TIMER_BASE_CLK/TIMER_DIVIDER)/MINIMALCOUNT)) // 1000/(8000/10) = 1.25f
#define MSPERDCPERIOD (20.0f/MSPERMINIMALCOUNT)//20

typedef enum DCDIRECTION
{
	FORWARD_DIRECTION,
	BACKWARD_DIRECTION,
	LEFT_DIRECTION,
	RIGHT_DIRECTION,
	MAX_DIRECTION
} DCDIRECTION;


/*
//it is a unit increment/decrement
typedef struct MotorState
{
	int stage;
	PinMask address;
} MotorState;
*/


typedef struct MotorDirection
{
	//DCDIRECTION direction; //no need for this
	PinMask address;
} MotorDirection;


typedef enum {
    STEP_MOTOR,
	DC_MOTOR
} MotorType;


typedef union {
	int16_t centiDegrees; //from 0 to 200000 centidegrees  (200 degrees), for STEP_MOTORs
	int16_t steps_for_dc; //for DC_MOTOR
} MotorState;

typedef struct {
	MotorType motorType;
	MotorState currentState;
	int stage; //for step motors
	PinMask address;
	//int16_t defaultDirection;
} Motor;

/*
typedef struct {
	int motorIndex;
	MotorState state;
} QueueElement;
*/
//

typedef enum {
	DISCONNECTED,
	CONNECTED,
	FORCE_DISCONNECT
} Connected;

typedef enum {
    CONNECTING,
	DISCONNECTING,
	CHANGE_12_MOTORS_STEPS,
	SOUND_SHORT,//SOUND_AND_CHANGE_12_MOTORS_STEPS,
	SOUND,
	IMAGE,
	COMMAND, //Type+9 bytes of commands
} MessageType;



typedef struct {
	size_t format;         /*!< Format of the pixel data */ //Reordered the information for easy of sending.
    size_t len;                 /*!< Length of the buffer in bytes */ //Reordered the information for easy of sending.
    uint8_t * buf;              /*!< Pointer to the pixel data */  //Reordered the information for easy of sending.
} CompactedImage;


/*
#define STEP_MOTOR_DIRECTION_PIN GPIO_NUM_2
#define STEP_MOTOR_STEP_PIN GPIO_NUM_4
#define STEP_MOTOR_SLEEP_PIN GPIO_NUM_NC //GPIO_NUM_16
#define STEP_MOTOR_RESET_PIN GPIO_NUM_NC //GPIO_NUM_15
#define STEP_MOTOR_MS3_PIN GPIO_NUM_NC //GPIO_NUM_7
#define STEP_MOTOR_MS2_PIN GPIO_NUM_NC //GPIO_NUM_6
#define STEP_MOTOR_MS1_PIN GPIO_NUM_NC //GPIO_NUM_5
#define STEP_MOTOR_ENABLE_PIN GPIO_NUM_NC //GPIO_NUM_4

#define MOTS_0_PIN GPIO_NUM_14
#define MOTS_1_PIN GPIO_NUM_12
#define MOTS_2_PIN GPIO_NUM_13
#define MOTS_3_PIN GPIO_NUM_15
*/


//MOTS_0 -> GPIO_NUM_14 HS2_CLK
//MOTS_1 -> GPIO_NUM_12 HS2_DATA2
//MOTS_2 -> GPIO_NUM_13 HS2_DATA3
//MOTS_3 -> GPIO_NUM_15 HS2_CMD
//MOTD -> GPIO_NUM_2 HS2_DATA0
//MOTP -> GPIO_NUM_4 HS2_DATA1


//#define RMT_TX_CHANNEL RMT_CHANNEL_0

void queueMotorSteps(char *recMessage);
void verifyMotorQueue();
void motorInit();
void motorDeinit();
void start_server(void);
void sendCameraImage(void *pic, int jpgeSource);
void sendRawData(MessageType type, PROJECT_DATA_TYPE *bytes, int length);
//void startMotors();

extern volatile int motorAvailableTimers;
extern volatile int isMotorRunning;
extern int controllerSocket;
extern volatile Connected connected;
extern int soundActive;

#endif
