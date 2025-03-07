/**
 * This example takes a picture every 5s and print its size on serial monitor.
 */

// =============================== SETUP ======================================

// 1. Board setup (Uncomment):
// #define BOARD_WROVER_KIT
// #define BOARD_ESP32CAM_AITHINKER

/**
 * 2. Kconfig setup
 *
 * If you have a Kconfig file, copy the content from
 *  https://github.com/espressif/esp32-camera/blob/master/Kconfig into it.
 * In case you haven't, copy and paste this Kconfig file inside the src directory.
 * This Kconfig file has definitions that allows more control over the camera and
 * how it will be initialized.
 */

/**
 * 3. Enable PSRAM on sdkconfig:
 *
 * CONFIG_ESP32_SPIRAM_SUPPORT=y
 *
 * More info on
 * https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/kconfig.html#config-esp32-spiram-support
 */

// ================================ CODE ======================================
#include <esp_log.h>
#include "esp_wifi.h"
#include <esp_system.h>
#include "esp_event.h"
#include <nvs_flash.h>
#include <sys/param.h>
#include "driver/gpio.h"
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tcpip_adapter_types.h"
#include "protocol_examples_common.h"

// support IDF 5.x
#ifndef portTICK_RATE_MS
#define portTICK_RATE_MS portTICK_PERIOD_MS
#endif

#include "main_project.h"
#include "DataArray.h"
#include "speaker.h"
#include "mic.h"
#include "esp_camera.h"

extern void startCameraServer();
int soundActive = 1;

volatile Connected connected = DISCONNECTED;
char conBuffer[MAX_PACKET];

#if SPEAKER_ON && SPEAKER_RMT
	rmt_speaker_t rmtSpeaker;
#endif

static const char *TAG = "main";

static camera_config_t camera_config = {
    .pin_pwdn = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sscb_sda = CAM_PIN_SIOD,
    .pin_sscb_scl = CAM_PIN_SIOC,

    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4, //Fixed using define @ main_project.h
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,

    //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    .xclk_freq_hz = 20000000,
    //.ledc_timer = LEDC_TIMER_0,
    //.ledc_channel = LEDC_CHANNEL_0,

    //.pixel_format = PIXFORMAT_RGB565, //YUV422,GRAYSCALE,RGB565,JPEG
    .pixel_format   = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_SVGA, // 800x600 //FRAMESIZE_SVGA,    //QQVGA-UXGA Do not use sizes above QVGA when not JPEG

    .jpeg_quality = 9, //0-63 lower number means higher quality
	//5 = ~48kb
	//9 = ~19kb
    .fb_count = 2,       //if more than one, i2s runs in continuous mode. Use only with JPEG
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

static esp_err_t init_camera()
{
    //initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK)
    {
    	//_ESP_LOGI(TAG, "Camera Init Failed");
        return err;
    }
    //LOG_LOCAL_LEVEL //_ESP_LOG_VERBOSE

    return ESP_OK;
}

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        //_ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
    }
}
void setLowPowerPin(uint8_t enable)
{
    gpio_set_direction(CAM_PIN_PWDN, GPIO_MODE_OUTPUT);
    gpio_set_level(CAM_PIN_PWDN, enable); //P-channel: 0 means On and 1 means Off
}
void playTestSound();
void app_main()
{

	//#undef SPEAKER_ON
	//#undef MIC_ON
	//redefine pref. IP to Local
	//@ sdkconfig
	//# CONFIG_EXAMPLE_CONNECT_IPV6_PREF_LOCAL_LINK is not set
	//CONFIG_EXAMPLE_CONNECT_IPV6_PREF_GLOBAL=y

	//@ main_project.h
	////#define CONFIG_EXAMPLE_IPV6 1



	//playTestSound();
	soundActive = 1;

	int cameraStarted = 0;
    //nvs_flash_init();
    //tcpip_adapter_init();

    ////_printf(("1....");
	//Turn off the low power pin so we can use the camera, motors and other resources.
	setLowPowerPin(0);

	//The first thing we must do after turning on VCC motors (and camera) pin is to init the motors so that we make sure they are off
    motorInit();


	///*
	//First, we disable the speaker pin just to make sure it wont be on by default
	forceSpeakerDisable();
	#if SPEAKER_ON && SPEAKER_LEDC
		DataArray speakerArray;
	    dataArrayInit(&speakerArray, SPEAKER_ARRAY_MAX);
	#endif

	#ifdef MIC_ON
	    //Sound Configurations
		DataArray micArray;
		PROJECT_DATA_TYPE *micRawData = malloc(4000);
	    //configure the Mic array and the Speaker Array
	    dataArrayInit(&micArray, MIC_ARRAY_MAX);
	#endif


    ESP_ERROR_CHECK(nvs_flash_init());
	////_printf(("5....");
    ESP_ERROR_CHECK(esp_netif_init());
	////_printf(("6....");

    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL) );
	////_printf(("7....");
    ESP_ERROR_CHECK(example_connect());
	////_printf(("8....");


    if(ESP_OK != init_camera()) {
    	//_ESP_LOGE(TAG, "Camera Failed.\n");
    	cameraStarted = 0;

        ////_ESP_LOGI(TAG, "Camera Ready! Use 'http://");
        //tcpip_adapter_ip_info_t adapterInfo;

        //tcpip_adapter_get_ip_info(0, &adapterInfo); //TCPIP_ADAPTERE_IF_STA = 0
        //char *ip = (char *)&adapterInfo.ip;
        ////_printf((WiFi.localIP());
        ////_ESP_LOGI(TAG, "%d . %d . %d , %d", ip[0], ip[1], ip[2], ip[3]);
        ////_ESP_LOGI(TAG, "' to connect");

        //return;
    } else {
    	cameraStarted = 1;
    }

    start_server();


//*/

    //esp_log_level_set("*", //_ESP_LOG_VERBOSE);

    /*//test with a default sound
    adjustSound(tmpSound, sizeof(tmpSound)/sizeof(SPEAKER_DATA_TYPE));
    dataArrayClearData(&micArray);
    dataArrayWriteFromArrayUINT8_T(&micArray, tmpSound, sizeof(tmpSound)/sizeof(SPEAKER_DATA_TYPE));
	//*/

	#if SPEAKER_ON && SPEAKER_LEDC
    	esp_log_level_set("*", ESP_LOG_ERROR);
			//Configure the LEDC PWM
			initSpeaker(&speakerArray);
	#endif
	#if SPEAKER_ON && SPEAKER_RMT
		//Configure the LEDC PWM
		initSpeaker(&rmtSpeaker);
		//playTestSound(&rmtSpeaker);
		//vTaskDelay(pdMS_TO_TICKS(1000));

    	esp_log_level_set("*", ESP_LOG_ERROR);
#if SPEAKER_PIN ==1 || SPEAKER_PIN ==3
#else
		//esp_log_level_set("*", ESP_LOG_INFO);
#endif
	#endif

	#ifdef MIC_ON
	    //Create the mic task
	    createMicrophoneTask(&micArray);
	#endif

    int64_t fr_start = 0;
    int64_t fr_end = 0;
    int64_t frame_time;


    //int64_t lastExecution = 0;




    while (1)
    {

    	//Sleep in order to let other tasks to process
		vTaskDelay(pdMS_TO_TICKS(40));

    	//if there are free timers
    	if(motorAvailableTimers > 0)
    		verifyMotorQueue();
    	//
        if(connected == CONNECTED)
        {
        	//pastMs += 1000/portTICK_RATE_MS;
///*
#if SPEAKER_ON && SPEAKER_RMT
        	if(soundActive) //only play if the sound is on
        	{
				if(verifyPlaySound(&rmtSpeaker, false))
				{
					//playing sound
				}
        	}
#endif
#ifdef MIC_ON

        	//if we have the sound active and
        	//if there is enough mic data available to send
        	if(soundActive && !playingSound && dataArrayElementsToRead(&micArray) > MIC_SAMPLES_TO_SEND)
        	{
        		////_ESP_LOGI(TAG, "Sending MIC data.");
        		//pastMs = 0;
        		dataArrayReadToArray(&micArray, micRawData, MIC_SAMPLES_TO_SEND);
        		sendRawData(SOUND, micRawData, MIC_SAMPLES_TO_SEND);
        		//continue;
        	}
#endif

        	if(cameraStarted)
        	{
				fr_start = esp_timer_get_time();
				camera_fb_t *pic = esp_camera_fb_get();
				if(pic)
				{
					ESP_LOGI(TAG, "Image start sending.");
					sendCameraImage((void*)pic, 1);
					esp_camera_fb_return(pic);
					ESP_LOGI(TAG, "Image sent.");
					//lastExecution = esp_timer_get_time();
	//				fr_end = esp_timer_get_time();
	//				frame_time = (fr_end - fr_start)/1000;
	//				// use pic->buf to access the image
	//				//_ESP_LOGI(TAG, "%zu with %.1ffps", pic->len, 1000.0 / (uint32_t)frame_time);
				} else {
					ESP_LOGI(TAG, "Could not get image.");
				}
        	}
        } else {
        	vTaskDelay(1000 / portTICK_RATE_MS);
        }
    }
#if SPEAKER_ON && SPEAKER_RMT
    deinitSpeaker(&rmtSpeaker);
#endif
#ifdef MIC_ON
    free(micRawData);
#endif
    motorDeinit();
}

/*
 *
#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "esp_camera.h"

esp_err_t event_handler(void *ctx, system_event_t *event)
{
    return ESP_OK;
}

void app_main(void)
{
    nvs_flash_init();
    tcpip_adapter_init();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    wifi_config_t sta_config = {
        .sta = {
            .ssid = CONFIG_ESP_WIFI_SSID,
            .password = CONFIG_ESP_WIFI_PASSWORD,
            .bssid_set = false
        }
    };

    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &sta_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );

    ESP_ERROR_CHECK( esp_wifi_connect() );

    gpio_set_direction(GPIO_NUM_4, GPIO_MODE_OUTPUT);
    int level = 0;
    while (true) {
        gpio_set_level(GPIO_NUM_4, level);
        level = !level;
        vTaskDelay(300 / portTICK_PERIOD_MS);
    }
}

*/
