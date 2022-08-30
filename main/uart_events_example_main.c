/* UART Events Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"

#include "bluetooth.h"

static const char *TAG = "uart_events";

/**
 * This example shows how to use the UART driver to handle special UART events.
 *
 * It also reads data from UART0 directly, and echoes it to console.
 *
 * - Port: UART0
 * - Receive (Rx) buffer: on
 * - Transmit (Tx) buffer: off
 * - Flow control: off
 * - Event queue: on
 * - Pin assignment: TxD (default), RxD (default)
 */

#define EX_UART_NUM UART_NUM_0
#define PATTERN_CHR_NUM    (3)         /*!< Set the number of consecutive and identical characters received by receiver which defines a UART pattern*/

#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)
static QueueHandle_t uart0_queue;





#include "led_strip.h"
#include "driver/rmt.h"
#include "stdio.h"

#include "string.h"



#define LEDS_NUM 38 //（一共有多少个LED灯泡？）

// Adalight sends a "Magic Word" (defined in /etc/boblight.conf) before sending the pixel data
uint8_t prefix[] = {'A', 'd', 'a'}, hi, lo, chk, i;

static int s_order = 0;
led_strip_t* strip;

typedef struct data_rgb{
	uint8_t rdata[1024];
	size_t len;
}data_rgb_t	;

data_rgb_t scvd;

static void set_pixel(led_strip_t* strip,int idx,uint8_t r,uint8_t g,uint8_t b){
	switch(s_order){
		case 0:
		//R,G,B
		break;
		case 1:{
			//R,B,G
			uint8_t tmp;
			tmp = g;
			g = b;
			b = tmp;
		}
		
		break;
		case 2:{
			//G,R,B
			uint8_t tmp;
			tmp = r;
			r = g;
			g = tmp;
		}
		break;
		case 3:{
		//G,B,R
			uint8_t tmp;
			tmp = r;
			r = g;
			g = b;
			b = tmp;
		}
		break;
		case 4:
		//B,R,G
		break;
		case 5:
		//B,G,R
		break;

	}

	strip->set_pixel(strip, idx, r, g, b);
	
}


void loop_show(uint8_t *strrgb,size_t rlen){

	int ret = 0;




	// Wait for first byte of Magic Word
	if(rlen<5)return;
	for(int i = 0; i < sizeof prefix; ++i) {
		if(prefix[i] == strrgb[ret++]) continue;
		// otherwise, start over
		i = 0;
	}
	

	// Hi, Lo, Checksum  
	hi=strrgb[ret++];
	lo=strrgb[ret++];

	chk=strrgb[ret++];

	// If checksum does not match go back to wait
	if (chk != (hi ^ lo ^ 0x55)) {
		return;
	}

	uint8_t r, g, b;  
	// Read the transmission data and set LED values
	for (uint8_t i = 0; i < LEDS_NUM; i++) {
		if(ret>=rlen)break;
		r=strrgb[ret++];
		g=strrgb[ret++];
		b=strrgb[ret++];
		// Shows new values
		set_pixel(strip, i, r, g, b);
	}

	rlen = 0;
	strip->refresh(strip,50);

}





static void uart_event_task(void *pvParameters)
{
	QueueHandle_t Mailbox;
	Mailbox = (QueueHandle_t)pvParameters;

    uart_event_t event;
    size_t buffered_size;
	uint8_t* dtmp = (uint8_t*) malloc(RD_BUF_SIZE);
    for(;;) {
        //Waiting for UART event.
        if(xQueueReceive(uart0_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
			
			bzero(dtmp, RD_BUF_SIZE);
            ESP_LOGI(TAG, "uart[%d] event:", EX_UART_NUM);
            switch(event.type) {
                //Event of UART receving data
                /*We'd better handler data event fast, there would be much more data events than
                other types of events. If we take too much time on data event, the queue might
                be full.*/
                case UART_DATA:
                    ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
                    uart_read_bytes(EX_UART_NUM, dtmp, event.size, portMAX_DELAY);
					ble_write(dtmp,event.size);
					//loop_show(dtmp,event.size);

//                    ESP_LOGI(TAG, "[DATA EVT]:");
//                    uart_write_bytes(EX_UART_NUM, (const char*) dtmp, event.size);
                    break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGI(TAG, "hw fifo overflow");
                    // If fifo overflow happened, you should consider adding flow control for your application.
                    // The ISR has already reset the rx FIFO,
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(EX_UART_NUM);
                    xQueueReset(uart0_queue);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGI(TAG, "ring buffer full");
                    // If buffer full happened, you should consider encreasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(EX_UART_NUM);
                    xQueueReset(uart0_queue);
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
                    ESP_LOGI(TAG, "uart rx break");
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGI(TAG, "uart parity error");
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGI(TAG, "uart frame error");
                    break;
                //UART_PATTERN_DET
                case UART_PATTERN_DET:
                    uart_get_buffered_data_len(EX_UART_NUM, &buffered_size);
                    int pos = uart_pattern_pop_pos(EX_UART_NUM);
                    ESP_LOGI(TAG, "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos, buffered_size);
                    if (pos == -1) {
                        // There used to be a UART_PATTERN_DET event, but the pattern position queue is full so that it can not
                        // record the position. We should set a larger queue size.
                        // As an example, we directly flush the rx buffer here.
                        uart_flush_input(EX_UART_NUM);
                    } else {
                        uart_read_bytes(EX_UART_NUM, dtmp, pos, 100 / portTICK_PERIOD_MS);
                        uint8_t pat[PATTERN_CHR_NUM + 1];
                        memset(pat, 0, sizeof(pat));
                        uart_read_bytes(EX_UART_NUM, pat, PATTERN_CHR_NUM, 100 / portTICK_PERIOD_MS);
                        ESP_LOGI(TAG, "read data: %s", dtmp);
                        ESP_LOGI(TAG, "read pat : %s", pat);
                    }
                    break;
                //Others
                default:
                    ESP_LOGI(TAG, "uart event type: %d", event.type);
                    break;
            }
        }
    }
//    free(dtmp);
//    dtmp = NULL;
    vTaskDelete(NULL);
}

void app_main(void)
{
	strip =  led_strip_init(RMT_CHANNEL_0,21,LEDS_NUM);
	scvd.len = 0;
	uint8_t r, g, b;  
	vTaskDelay(50);
	for(int j = 0;j <3;j++){
		for(int i=0;i<LEDS_NUM;i++){
			r = (j==0)?255:0;
			g = (j==1)?255:0;
			b = (j==2)?255:0;
			set_pixel(strip, i, r, g, b);
		}
		strip->refresh(strip,50);
		vTaskDelay(100);
	}

	
    esp_log_level_set(TAG, ESP_LOG_INFO);

    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    //Install UART driver, and get the queue.
    uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart0_queue, 0);
    uart_param_config(EX_UART_NUM, &uart_config);

    //Set UART log level
    esp_log_level_set(TAG, ESP_LOG_INFO);
    //Set UART pins (using UART0 default pins ie no changes.)
    uart_set_pin(EX_UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    //Set uart pattern detect function.
    uart_enable_pattern_det_baud_intr(EX_UART_NUM, '+', PATTERN_CHR_NUM, 9, 0, 0);
    //Reset the pattern queue length to record at most 20 pattern positions.
    uart_pattern_queue_reset(EX_UART_NUM, 20);

    //Create a task to handler UART event from ISR
    xTaskCreate(uart_event_task, "uart_event_task", 1024*7, NULL, 12, NULL);
	
	gattc_multi_connect();
}
