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
#include "mic.h"


#ifdef TCP_CON


#if SPEAKER_ON && SPEAKER_RMT
	extern rmt_speaker_t rmtSpeaker;
#endif

int sockClient = -1;
struct sockaddr_storage source_addr;

#define KEEPALIVE_IDLE              5 //Keep-alive idle time. In idle time without receiving any data from peer, will send keep-alive probe packet
#define KEEPALIVE_INTERVAL          5 //Keep-alive probe packet interval time.
#define KEEPALIVE_COUNT             3 //Keep-alive probe packet retry count.

static const char *TAG = "TCP_Server";

/*
static void do_retransmit(const int sock)
{
    int len;
    char rx_buffer[128];

    do {
        len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
        if (len < 0) {
            //_ESP_LOGI(TAG, "Error occurred during receiving: errno %d", errno);
        } else if (len == 0) {
            //_ESP_LOGW(TAG, "Connection closed");
        } else {
            rx_buffer[len] = 0; // Null-terminate whatever is received and treat it like a string
            //_ESP_LOGI(TAG, "Received %d bytes: %s", len, rx_buffer);

            // send() can return less bytes than supplied length.
            // Walk-around for robust implementation.
            int to_write = len;
            while (to_write > 0) {
                int written = send(sock, rx_buffer + (len - to_write), to_write, 0);
                if (written < 0) {
                    //_ESP_LOGI(TAG, "Error occurred during sending: errno %d", errno);
                }
                to_write -= written;
            }
        }
    } while (len > 0);
}*/

static int processMessage(char *rx_buffer)
{
	int len, totalRec;
	//Note that 1 byte was already received
	//_ESP_LOGI(TAG, "Received message type %d.", rx_buffer[0]);

	////_ESP_LOGI(TAG, "Received %d data.", len);
	//determine how many bytes we should receive next based on the type of the message
	switch(rx_buffer[0])
	{
		case CONNECTING:
			//do nothing
			//receive all bytes left at the buffer
			recv(sockClient, rx_buffer, sizeof(rx_buffer) - 1, 0);
			break;
		case DISCONNECTING:
			//receive all bytes left at the buffer
			recv(sockClient, rx_buffer, sizeof(rx_buffer) - 1, 0);
			return 0;
		case CHANGE_12_MOTORS_STEPS:
			//receive the 24 bytes left
			len = recv(sockClient, rx_buffer, 24, 0);
			if(len < 24) //we already received 1 byte. Receive the 24 bytes left (of total 1+12*2=25)
			{
	            //_ESP_LOGI(TAG, "CHANGE_12_MOTORS_STEPS with Len %d. No recv. performed.", len);
				break;
			}
            //_ESP_LOGI(TAG, "RXBuffer Len %d.", len);
			queueMotorSteps(rx_buffer);//pass the address of the first angle
			break;
		case SOUND_SHORT://SOUND_AND_CHANGE_12_MOTORS_STEPS:
			//receive the SPEAKER_VALUES_PER_ITEM bytes left //+24 bytes left
			totalRec = 0;
			while(totalRec < SPEAKER_VALUES_PER_ITEM)//+24)
			{
				len = recv(sockClient, rx_buffer+totalRec, SPEAKER_VALUES_PER_ITEM-totalRec,0);//+24-totalRec, 0);
				if(len < 0)
				{
					//_ESP_LOGI(TAG, "Error occurred while received MIC samples. SOUND_AND_CHANGE_12_MOTORS_STEPS with Len %d. No recv. performed", totalRec);
					return 1;
				}
				totalRec += len;
				if(totalRec < SPEAKER_VALUES_PER_ITEM)//+24)
					vTaskDelay(pdMS_TO_TICKS(30));
			}
			//_ESP_LOGI(TAG, "Received %d data of sound and motor positions.", len);
			//first, we must receive 4k of sound. Then, the angles.
#if SPEAKER_ON && SPEAKER_LEDC
			receiveSoundData((uint8_t *)(rx_buffer+1), len-1);//len-25);
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

//		        uint32_t io_reg = GPIO_PIN_MUX_REG[1];
//		        periph_ll_disable_clk_set_rst(uart_periph_signal[0].module);
//		        periph_module_disable(uart_periph_signal[0].module);
//				gpio_intr_disable(1);
				//gpio_output_disable(1);
//	            gpio_hal_iomux_func_sel(io_reg, PIN_FUNC_GPIO);

	            //Total of 1024 sound bytes
				addSound(&rmtSpeaker, (uint8_t *)(rx_buffer+1)); //fist 512 bytes
				//addSound(&rmtSpeaker, (uint8_t *)(rx_buffer+1+512)); //next 512 bytes
				verifyPlaySound(&rmtSpeaker, false);
//		        periph_module_disable(uart_periph_signal[0].module);
//				gpio_intr_disable(1);
//				//gpio_output_disable(1);
//	            gpio_hal_iomux_func_sel(io_reg, PIN_FUNC_GPIO);
			}

#endif
			//then obtain the angles
			//queueMotorSteps(rx_buffer+MIC_SAMPLES_TO_SEND);//pass the address of the first angle
			break;
		case COMMAND:
			//receive the 9 bytes left
			len = recv(sockClient, rx_buffer, 9, 0);
			if(len < 9) //we already received 1 byte. Receive the 24 bytes left (of total 1+12*2=25)
			{
	            //_ESP_LOGI(TAG, "CHANGE_12_MOTORS_STEPS with Len %d. No recv. performed.", len);
				break;
			}

			//the first byte of command (after type) is sound active
			soundActive = rx_buffer[0];
			//the other are unused for now.

			break;
		default:
			//Unknown message.
			//_ESP_LOGI(TAG, "Unknown message. Freeing the buffer.");
			//receive all bytes left at the buffer
			recv(sockClient, rx_buffer, sizeof(rx_buffer) - 1, 0);
			break;
	}
	return 1;
}
static int consecutivesErrors = 0; //if there are more than 10 tolerable errors without any success, we disconnect
char bytes[5];
void sendCameraImageRawdata(void *pic)
{
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
		//send the message type + 4 bytes of the size
		bytes[0] = IMAGE;
		bytes[1] = (picture->len & 0xFF);
		bytes[2] = ((picture->len >> 8) & 0xFF);
		bytes[3] = ((picture->len >> 16) & 0xFF);
		bytes[4] = ((picture->len >> 24) & 0xFF);
        int err = send(sockClient, bytes, 5, 0);
		if (err < 0) {

			//_ESP_LOGI(TAG, "Error occurred during sending: errno %d, len: %d", errno, 5);
			if(errno == EDQUOT || errno == ENOMEM || errno == EMSGSIZE)
			{
				consecutivesErrors++;
				if(consecutivesErrors < 10)
					return;//without disconnecting;
			}
			connected = FORCE_DISCONNECT;
			return;
		}

        //Send image
        err = send(sockClient, picture->buf, (picture->len), 0);
		//int err = sendto(controllerSocket, picture->buf, (picture->len), 0, (struct sockaddr *)&source_addr, sizeof(source_addr));
		if (err < 0) {

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
	//send the message type + 4 bytes of the size
	bytes[0] = type;
	bytes[1] = (length & 0xFF);
	bytes[2] = ((length >> 8) & 0xFF);
	bytes[3] = ((length >> 16) & 0xFF);
	bytes[4] = ((length >> 24) & 0xFF);
    int err = send(sockClient, bytes, 5, 0);
	if (err < 0) {

		//_ESP_LOGI(TAG, "Error occurred during sending: errno %d, len: %d", errno, 5);
		if(errno == EDQUOT || errno == ENOMEM || errno == EMSGSIZE)
		{
			consecutivesErrors++;
			if(consecutivesErrors < 10)
				return;//without disconnecting;
		}
		connected = FORCE_DISCONNECT;
		return;
	}

    err = send(sockClient, bytes, length, 0);
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
		//char tmpData[18] = { 0x03, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x00, 0xFF, 0xD8, 0xFF, 0x01, 0x02, 0x03, 0x04, 0x05, 0xFF, 0xD9};

	    //int err = send(sockClient, tmpData, 18, 0);
		//sendto(controllerSocket, tmpData, 18, 0, (struct sockaddr *)&source_addr, sizeof(source_addr)); //int err =

        return;
	}
}


static void tcp_server_task(void *pvParameters)
{
    char addr_str[128];
    int addr_family = (int)pvParameters;
    int ip_protocol = 0;
    int keepAlive = 1;
    int keepIdle = KEEPALIVE_IDLE;
    int keepInterval = KEEPALIVE_INTERVAL;
    int keepCount = KEEPALIVE_COUNT;
    struct sockaddr_storage dest_addr;

    while (1) {
        int listen_sock = -1;
        //
        connected = DISCONNECTED;

		if (addr_family == AF_INET) {
			struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
			dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
			dest_addr_ip4->sin_family = AF_INET;
			dest_addr_ip4->sin_port = htons(PORT);
			ip_protocol = IPPROTO_IP;
		}
	#ifdef CONFIG_EXAMPLE_IPV6
		else if (addr_family == AF_INET6) {
			struct sockaddr_in6 *dest_addr_ip6 = (struct sockaddr_in6 *)&dest_addr;
			bzero(&dest_addr_ip6->sin6_addr.un, sizeof(dest_addr_ip6->sin6_addr.un));
			dest_addr_ip6->sin6_family = AF_INET6;
			dest_addr_ip6->sin6_port = htons(PORT);
			ip_protocol = IPPROTO_IPV6;
		}
	#endif

		listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
		if (listen_sock < 0) {
			//_ESP_LOGI(TAG, "Unable to create socket: errno %d", errno);
			continue;
		}
		int opt = 1;
		setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
	#if defined(CONFIG_EXAMPLE_IPV4) && defined(CONFIG_EXAMPLE_IPV6)
		// Note that by default IPV6 binds to both protocols, it is must be disabled
		// if both protocols used at the same time (used in CI)
		setsockopt(listen_sock, IPPROTO_IPV6, IPV6_V6ONLY, &opt, sizeof(opt));
	#endif

		//_ESP_LOGI(TAG, "Socket created");

		int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
		if (err != 0) {
            //_ESP_LOGI(TAG, "Socket unable to bind: errno %d", errno);
		    close(listen_sock);
            continue;
		}
		//_ESP_LOGI(TAG, "Socket bound, port %d", PORT);

		err = listen(listen_sock, 1);
		if (err != 0) {
			//_ESP_LOGI(TAG, "Error occurred during listen: errno %d", errno);
		    close(listen_sock);
            continue;
		}

		//struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
		socklen_t addr_len = sizeof(source_addr);
		sockClient = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
		if (sockClient < 0) {
			//_ESP_LOGI(TAG, "Unable to accept connection: errno %d", errno);
		    close(listen_sock);
            continue;
		}
        connected = CONNECTED;

		// Set tcp keepalive option
		setsockopt(sockClient, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
		setsockopt(sockClient, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(int));
		setsockopt(sockClient, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(int));
		setsockopt(sockClient, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(int));
		// Convert ip address to string
		if (source_addr.ss_family == PF_INET) {
			inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
		}
#ifdef CONFIG_EXAMPLE_IPV6
		else if (source_addr.ss_family == PF_INET6) {
			inet6_ntoa_r(((struct sockaddr_in6 *)&source_addr)->sin6_addr, addr_str, sizeof(addr_str) - 1);
		}
#endif
		//_ESP_LOGI(TAG, "Socket accepted ip address: %s", addr_str);

		while (1) {

        	//Force Disconnect?
        	if(connected == FORCE_DISCONNECT)
        	{
                //_ESP_LOGI(TAG, "Force disconnect.");
        		break;
        	}

            //_ESP_LOGI(TAG, "Waiting for data");
            //struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
            //socklen_t socklen = sizeof(source_addr);
            //int len = recvfrom(controllerSocket, conBuffer, sizeof(conBuffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

            //Receive only 1 byte to determine the type of the message
            int len = recv(sockClient, conBuffer, 1, 0);
            //int len = recv(sockClient, conBuffer, sizeof(conBuffer) - 1, 0);
            // Error occurred during receiving
            if (len < 0) {
                //_ESP_LOGI(TAG, "recvfrom failed: errno %d", errno);
                break;
            }
            // Data received
            else {
            	//process the message and break receiving loop if it disconnects
            	if(processMessage(conBuffer) == 0)
            		break;

            }

			//do_retransmit(sock);
		}

        if (sockClient != -1) {
			shutdown(sockClient, 0);
			close(sockClient);
			sockClient = -1;
        }

        if (listen_sock != -1) {
            //_ESP_LOGI(TAG, "Shutting down socket and restarting...");
            shutdown(listen_sock, 0);
            close(listen_sock);
        }
    }
    vTaskDelete(NULL);
}
void start_server(void)
{
//    ESP_ERROR_CHECK(nvs_flash_init());
//    ESP_ERROR_CHECK(esp_netif_init());
//    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
//    ESP_ERROR_CHECK(example_connect());

#ifdef CONFIG_EXAMPLE_IPV4
    xTaskCreate(tcp_server_task, "tcp_server", 4096, (void*)AF_INET, 5, NULL);
#endif
#ifdef CONFIG_EXAMPLE_IPV6
    xTaskCreate(tcp_server_task, "tcp_server", 4096, (void*)AF_INET6, 5, NULL);
#endif
}

#endif
