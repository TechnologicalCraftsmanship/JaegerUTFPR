/* BSD Socket API Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "main_project.h"
#include "img_converters.h"
#include "DataArray.h"
#include "speaker.h"
#include "hal/gpio_hal.h"
#include "driver/periph_ctrl.h"
#include "soc/uart_periph.h"
#include "hal/clk_gate_ll.h"


#ifdef UDP_CON


#if SPEAKER_ON && SPEAKER_RMT
	extern rmt_speaker_t rmtSpeaker;
#endif

int controllerSocket = -1;
struct sockaddr_storage source_addr;

static const char *TAG = "udp_server";

static int processMessage(char *rx_buffer, int len)
{
	//_ESP_LOGI(TAG, "Received %d data.", len);
	switch(rx_buffer[0])
	{
		case CONNECTING:
			//do nothing :)
			break;
		case DISCONNECTING:
			return 0;
		case CHANGE_12_MOTORS_STEPS:
			if(len < 24)
			{
	            //_ESP_LOGI(TAG, "CHANGE_12_MOTORS_STEPS with Len %d. No recv. performed", len);
				break;
			}
            //_ESP_LOGI(TAG, "RXBuffer Len %d.", len);
			queueMotorSteps(rx_buffer+1);//pass the address of the first angle
			break;
		case SOUND_AND_CHANGE_12_MOTORS_STEPS:
			//if(len < 4024)
			//{
	        //    //_ESP_LOGI(TAG, "SOUND_AND_CHANGE_12_MOTORS_STEPS with Len %d. No recv. performed", len);
			//	break;
			//}
			//_ESP_LOGI(TAG, "Received %d data of sound and motor positions.", len);
			//first, we must receive 4k of sound. Then, the angles.
#if SPEAKER_ON && SPEAKER_LEDC
			receiveSoundData((uint8_t *)(rx_buffer+1), len-25);
#endif
#if SPEAKER_ON && SPEAKER_RMT
			//did we start the speaker already?
			if(rmtSpeaker.blockArray.array != NULL)
			{
//			    gpio_config_t io_conf;
//			    io_conf.intr_type = GPIO_INTR_DISABLE;
//			    io_conf.mode = GPIO_MODE_DISABLE;//GPIO_MODE_OUTPUT_OD;//GPIO_MODE_INPUT;
//				io_conf.pin_bit_mask = BIT64(1); //UART0TX and UART0RX
//			    io_conf.pull_down_en = 0;
//			    io_conf.pull_up_en = 0;
//			    ESP_ERROR_CHECK(gpio_config(&io_conf));

		        uint32_t io_reg = GPIO_PIN_MUX_REG[1];
		        periph_ll_disable_clk_set_rst(uart_periph_signal[0].module);
		        periph_module_disable(uart_periph_signal[0].module);
				gpio_intr_disable(1);
				//gpio_output_disable(1);
	            gpio_hal_iomux_func_sel(io_reg, PIN_FUNC_GPIO);

				addSound(&rmtSpeaker, (uint8_t *)(rx_buffer+1));
				verifyPlaySound(&rmtSpeaker);
		        periph_module_disable(uart_periph_signal[0].module);
				gpio_intr_disable(1);
				//gpio_output_disable(1);
	            gpio_hal_iomux_func_sel(io_reg, PIN_FUNC_GPIO);
			}

#endif
			//then obtain
			//queueMotorSteps(rx_buffer+1+(len-25));//pass the address of the first angle
			break;
	}
	return 1;
}

char bytes[4];
void sendCameraImageRawdata(void *pic)
{
	static int consecutivesErrors = 0; //if there are more than 10 tolerable errors without any success, we disconnect
	//for easy of transmisson, we gonna send the following field of camera_fb_t
/*
    pixformat_t format;         //!< Format of the pixel data/ //Reordered the information for easy of sending.
    size_t len;                 //!< Length of the buffer in bytes //Reordered the information for easy of sending.
    uint8_t * buf;              //!< Pointer to the pixel data  //Reordered the information for easy of sending.
    */

	camera_fb_t *picture = (camera_fb_t*)pic;

	//Sending this way wont work because of the pointer to the buffer: if necessary, we must send 2 fields separed to the buffer.
    //int err = sendto(controllerSocket, picture, (4+4+picture->len), 0, (struct sockaddr *)&source_addr, sizeof(source_addr));

	if(pic != NULL && picture->buf != NULL && picture->len > 0)
	{
//		For now, UDP Packets does not contain the type and size for images
//		//send the message type + 4 bytes of the size
//		bytes[0] = IMAGE;
//		bytes[1] = (picture->len & 0xFF);
//		bytes[2] = ((picture->len >> 8) & 0xFF);
//		bytes[3] = ((picture->len >> 16) & 0xFF);
//		bytes[4] = ((picture->len >> 24) & 0xFF);
//        int err = sendto(controllerSocket, bytes, 5, 0, (struct sockaddr *)&source_addr, sizeof(source_addr));
//		if (err < 0) {
//			//_ESP_LOGI(TAG, "Error occurred during sending: errno %d, len: %d", errno, 5);
//			if(errno == EDQUOT || errno == ENOMEM || errno == EMSGSIZE)
//			{
//				consecutivesErrors++;
//				if(consecutivesErrors < 10)
//					return;//without disconnecting;
//			}
//			connected = FORCE_DISCONNECT;
//			return;
//		}

        //Send image
		int err = sendto(controllerSocket, picture->buf, (picture->len), 0, (struct sockaddr *)&source_addr, sizeof(source_addr));
		if (err < 0) {
			//ENOMEM = 12 = Out of memory
			//EDQUOT = 122 = Quota Exceeded

			//_ESP_LOGI(TAG, "Error occurred during sending: errno %d, len: %d", errno, picture->len);
			if(errno == EDQUOT || errno == ENOMEM || errno == EMSGSIZE)
			{
				consecutivesErrors++;
				if(consecutivesErrors < 10)
					return;//without disconnecting;
			}
			connected = FORCE_DISCONNECT;
			return;
		}
		consecutivesErrors = 0;
	} else {
        //_ESP_LOGI(TAG, "No image to send!");
	}
}

void sendRawData(MessageType type, PROJECT_DATA_TYPE *bytes, int length)
{
	static int consecutivesErrors = 0; //if there are more than 10 tolerable errors without any success, we disconnect

//	For now, UDP Packets does not contain the type for sound
//	//send the message type
//	bytes[0] = type;
//    int err = sendto(controllerSocket, bytes, 1, 0, (struct sockaddr *)&source_addr, sizeof(source_addr));
//	if (err < 0) {
//
//		//_ESP_LOGI(TAG, "Error occurred during sending: errno %d, len: %d", errno, 1);
//		if(errno == EDQUOT || errno == ENOMEM || errno == EMSGSIZE)
//		{
//			consecutivesErrors++;
//			if(consecutivesErrors < 10)
//				return;//without disconnecting;
//		}
//		connected = FORCE_DISCONNECT;
//		return;
//	}

    int err = sendto(controllerSocket, bytes, length, 0, (struct sockaddr *)&source_addr, sizeof(source_addr));
	//int err = sendto(controllerSocket, bytes, length, 0, (struct sockaddr *)&source_addr, sizeof(source_addr));
	if (err < 0) {

		//_ESP_LOGI(TAG, "Error occurred during sending: errno %d, len: %d", errno, length);
		if(errno == EDQUOT || errno == ENOMEM || errno == EMSGSIZE)
		{
			consecutivesErrors++;
			if(consecutivesErrors < 10)
				return;//without disconnecting;
		}
		connected = FORCE_DISCONNECT;
		return;
	}
	consecutivesErrors = 0;
}

void sendCameraImage(void *pic, int jpgeSource)
{
	if(jpgeSource == 1 )
	{
		sendCameraImageRawdata(pic);
	}
	else
	{
		char tmpData[18] = { 0x03, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x00, 0xFF, 0xD8, 0xFF, 0x01, 0x02, 0x03, 0x04, 0x05, 0xFF, 0xD9};
	    sendto(controllerSocket, tmpData, 18, 0, (struct sockaddr *)&source_addr, sizeof(source_addr)); //int err =

        ////_ESP_LOGI(TAG, "Not Implemented");
        return;
        /*
		CompactedImage compactedImage;

		camera_fb_t *picture = (camera_fb_t*)pic;
		compactedImage.format = picture->format;

        bool jpeg_converted = frame2jpg(picture, 80, &compactedImage.buf, &compactedImage.len);
        if(!jpeg_converted){
	        //_ESP_LOGI(TAG, "PEG compression failed");
            return;
        }
        //_printf(("Image size: %d\n", compactedImage.len);

		//Since we are using exacly the same fields for CompactedImage and camera_fb_t, we can use the same function
        //sendCameraImageRawdata(&compactedImage);


        int err = sendto(controllerSocket, &compactedImage, (sizeof(pixformat_t)+sizeof(size_t)+compactedImage.len), 0, (struct sockaddr *)&source_addr, sizeof(source_addr));
        if (err < 0) {
            //_ESP_LOGI(TAG, "Error occurred during sending: errno %d", errno);
            connected = FORCE_DISCONNECT;
            return;
        }


        if(compactedImage.buf){
			free(compactedImage.buf);
			compactedImage.buf = NULL;
		}*/
	}
}

static void udp_server_task(void *pvParameters)
{
    //char addr_str[128];
    int addr_family = (int)pvParameters;
    int ip_protocol = 0;
    struct sockaddr_in6 dest_addr;

    while (1) {
        //
        connected = DISCONNECTED;

        if (addr_family == AF_INET) {
            struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
            dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
            dest_addr_ip4->sin_family = AF_INET;
            dest_addr_ip4->sin_port = htons(PORT);
            ip_protocol = IPPROTO_IP;
            //_ESP_LOGI(TAG, "IPv4");
        } else if (addr_family == AF_INET6) {
            bzero(&dest_addr.sin6_addr.un, sizeof(dest_addr.sin6_addr.un));
            dest_addr.sin6_family = AF_INET6;
            dest_addr.sin6_port = htons(PORT);
            ip_protocol = IPPROTO_IPV6;
            //_ESP_LOGI(TAG, "IPv6");
        }

        controllerSocket = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (controllerSocket < 0) {
            //_ESP_LOGI(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        //_ESP_LOGI(TAG, "Socket created");

#if defined(CONFIG_EXAMPLE_IPV4) && defined(CONFIG_EXAMPLE_IPV6)
        if (addr_family == AF_INET6) {
            // Note that by default IPV6 binds to both protocols, it is must be disabled
            // if both protocols used at the same time (used in CI)
            int opt = 1;
            setsockopt(controllerSocket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
            setsockopt(controllerSocket, IPPROTO_IPV6, IPV6_V6ONLY, &opt, sizeof(opt));
        }
#endif

        int err = bind(controllerSocket, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err < 0) {
            //_ESP_LOGI(TAG, "Socket unable to bind: errno %d", errno);
            continue;
        }
        //_ESP_LOGI(TAG, "Socket bound, port %d", PORT);

        while (1) {
        	//Force Disconnect?
        	if(connected == FORCE_DISCONNECT)
        	{
                //_ESP_LOGI(TAG, "Force disconnect.");
        		break;
        	}

            //_ESP_LOGI(TAG, "Waiting for data");
            //struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
            socklen_t socklen = sizeof(source_addr);
            int len = recvfrom(controllerSocket, conBuffer, sizeof(conBuffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);
            connected = CONNECTED;
            // Error occurred during receiving
            if (len < 0) {
                //_ESP_LOGI(TAG, "recvfrom failed: errno %d", errno);
                break;
            }
            // Data received
            else {
            	//process the message and break receiving loop if it disconnects
            	if(processMessage(conBuffer, len) == 0)
            		break;

                //_ESP_LOGI(TAG, "Received %d bytes.", len);
            	/*
                // Get the sender's ip address as string
                if (source_addr.ss_family == PF_INET) {
                    inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
                } else if (source_addr.ss_family == PF_INET6) {
                    inet6_ntoa_r(((struct sockaddr_in6 *)&source_addr)->sin6_addr, addr_str, sizeof(addr_str) - 1);
                }

                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string...
                //_ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
                //_ESP_LOGI(TAG, "%s", rx_buffer);

                int err = sendto(controllerSocket, rx_buffer, len, 0, (struct sockaddr *)&source_addr, sizeof(source_addr));
                if (err < 0) {
                    //_ESP_LOGI(TAG, "Error occurred during sending: errno %d", errno);
                    break;
                }*/
            }
        	//Sleep in order to let other tasks to process
    		vTaskDelay(pdMS_TO_TICKS(90));
        }

        if (controllerSocket != -1) {
            //_ESP_LOGI(TAG, "Shutting down socket and restarting...");
            shutdown(controllerSocket, 0);
            close(controllerSocket);
        }
    }
    vTaskDelete(NULL);
}

void start_server(void)
{
    //ESP_ERROR_CHECK(nvs_flash_init());
    //ESP_ERROR_CHECK(esp_netif_init());
    //ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    //ESP_ERROR_CHECK(example_connect());

#ifdef CONFIG_EXAMPLE_IPV4
    xTaskCreate(udp_server_task, "udp_server", 4096, (void*)AF_INET, 5, NULL);
#endif
#ifdef CONFIG_EXAMPLE_IPV6
    xTaskCreate(udp_server_task, "udp_server", 4096, (void*)AF_INET6, 5, NULL);
#endif

}
#endif

