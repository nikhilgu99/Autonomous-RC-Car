/* EC444 Team 12 Quest 4 Alex Fatyga, Nikhil Gupta, Justin Morgan
   This is the esp32 code file. The purpose of this file is the following:

        Polling two microlidar sensors and controlling steering based on the 
        discrepancy between their respective measurements.

        Polling a LidarLite sensor at the front of the crawler for collision detection.

        Using a pulse sensor with a printed encorder pattern in order to detect wheel
        speed by using the time between detected pulses.

        Outputting the speed to a seven segment alphanumeric display.

        Controlling the crawler via remote using a UDP server task.

        Controlling/moderating the crawler speed via PID control.
*/

//////////////////////////////////////////////
////////// INCLUDED LIBRARIES ////////////////
//////////////////////////////////////////////

#include <stdio.h>
#include <math.h>
#include "driver/i2c.h"
#include "./ADXL343.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

#include "freertos/queue.h"
#include "driver/periph_ctrl.h"
#include "driver/gpio.h"
#include "driver/pcnt.h"
#include "esp_attr.h"
#include "esp_log.h"

#include "driver/uart.h"
#include "esp_vfs_dev.h"

//Network includes
#include <string.h>
#include <sys/param.h>
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "tcpip_adapter.h"
#include "protocol_examples_common.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

//Timer includes
#include "driver/timer.h"
#include <time.h>
#include "esp_types.h"

//////////////////////////////////////////////
////////// DEFINING CONSTANTS ////////////////
//////////////////////////////////////////////

#define EXAMPLE_WIFI_SSID "Group_12"
#define EXAMPLE_WIFI_PASS "smart444"

//Alphanumeric Display Defines
#define SLAVE_ADDR_ALPHA                   0x70 // alphanumeric address
#define OSC                                0x21 // oscillator cmd
#define HT16K33_BLINK_DISPLAYON            0x01 // Display on cmd
#define HT16K33_BLINK_OFF                  0    // Blink off cmd
#define HT16K33_BLINK_CMD                  0x80 // Blink cmd
#define HT16K33_CMD_BRIGHTNESS             0xE0 // Brightness cmd

// Master I2C (LiderLite)
#define I2C_EXAMPLE_MASTER_SCL_IO          22   // gpio number for i2c clk
#define I2C_EXAMPLE_MASTER_SDA_IO          23   // gpio number for i2c data
#define I2C_EXAMPLE_MASTER_NUM             I2C_NUM_0  // i2c port
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_FREQ_HZ         100000     // i2c master clock freq
#define WRITE_BIT                          I2C_MASTER_WRITE // i2c master write
#define READ_BIT                           I2C_MASTER_READ  // i2c master read
#define ACK_CHECK_EN                       true // i2c master will check ack
#define ACK_CHECK_DIS                      false// i2c master will not check ack
#define ACK_VAL                            0x00 // i2c ack value
#define NACK_VAL                           0xFF // i2c nack value

#define I2C_EXAMPLE_MASTER_SCL_IO_DISPLAY 15
#define I2C_EXAMPLE_MASTER_SDA_IO_DISPLAY 14
#define I2C_EXAMPLE_MASTER_NUM_DISPLAY    I2C_NUM_1  // i2c port for alphanumeric

// ADXL343
#define SLAVE_ADDR                         0x62 // 0x53

#define distance_high_reg                   0x0F
#define distance_low_reg                    0x10
#define velocity_reg                        0x09

//UART Defines
#define ECHO_TEST_TXD  17
#define ECHO_TEST_RXD  16
#define ECHO_TEST_RTS  (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS  (UART_PIN_NO_CHANGE)

//UART Defines
#define ECHO_TEST_TXD_2  25
#define ECHO_TEST_RXD_2  26
#define ECHO_TEST_RTS_2  (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS_2  (UART_PIN_NO_CHANGE)

#define BUF_SIZE (128)

//Server IP
#ifdef CONFIG_EXAMPLE_IPV4
#define HOST_IP_ADDR "192.168.1.145"
#else
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV6_ADDR
#endif

#define RXPORT 8081

#define PCNT_TEST_UNIT      PCNT_UNIT_0
#define PCNT_H_LIM_VAL      1
#define PCNT_L_LIM_VAL     -10000
#define PCNT_THRESH1_VAL    5000
#define PCNT_THRESH0_VAL   -5000
#define PCNT_INPUT_SIG_IO   39  // Pulse Input GPIO
#define PCNT_INPUT_CTRL_IO  32  // Control GPIO HIGH=count up, LOW=count down

//Timer defines
#define TIMER_DIVIDER         16  //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds
#define TIMER_INTERVAL0_SEC   (0.4) // sample test interval for the first timer
#define TIMER_INTERVAL1_SEC   (5.78)   // sample test interval for the second timer
#define TEST_WITHOUT_RELOAD   0        // testing will be done without auto reload
#define TEST_WITH_RELOAD      1        // testing will be done with auto reload

#define COLLISIONRANGE        44
#define SPEEDERROR            0.3
#define STEERINGERROR         18
#define SPEED_THRESHOLD_LOW   0.1
#define SPEED_THRESHOLD_HIGH  0.4
#define SPEED_THRESHOLD_FILTER 0.45


/////////////////////////////////////////
/////INSTANTIATING GLOBAL VARIABLES//////
/////////////////////////////////////////

//Global definitions of the message tag and the paayload
static const char *TAG = "example";
static char *payload = "Message from ESP32 ";

/*
 * A sample structure to pass events
 * from the timer interrupt handler to the main program.
 */
typedef struct {
    int type;  // the type of timer's event
    int timer_group;
    int timer_idx;
    uint64_t timer_counter_value;
} timer_event_t;

xQueueHandle timer_queue;


xQueueHandle pcnt_evt_queue;   // A queue to handle pulse counter events
pcnt_isr_handle_t user_isr_handle = NULL; //user's ISR service handle

/* A sample structure to pass events from the PCNT
 * interrupt handler to the main program.
 */
typedef struct {
    int unit;  // the PCNT unit that originated an interrupt
    uint32_t status; // information on the event type that caused the interrupt
} pcnt_evt_t;

//Global count variable, counts pulses
int16_t count = 0;
double revolutionsPerSec = 0;
double speed;
double oldSpeed;
int newCount = 0;

//RANGE OF THE LIDARLITE
int16_t lidarLiteRange = 0;

//Variables to hold the current/previous time
//to get the time between pulses for wheel speed
uint32_t currentTime;
uint32_t previousTime;


// Variable used to control the speed
int speedVar = 1300;

///////////////////////////////////
/////TIMER FUNCTIONS!//////////////
///////////////////////////////////

/*
 * Timer group0 ISR handler
 *
 * Note:
 * We don't call the timer API here because they are not declared with IRAM_ATTR.
 * If we're okay with the timer irq not being serviced while SPI flash cache is disabled,
 * we can allocate this interrupt without the ESP_INTR_FLAG_IRAM flag and use the normal API.
 */
void IRAM_ATTR timer_group0_isr(void *para)
{
    int timer_idx = (int) para;

    /* Retrieve the interrupt status and the counter value
       from the timer that reported the interrupt */
    timer_intr_t timer_intr = timer_group_intr_get_in_isr(TIMER_GROUP_0);
    uint64_t timer_counter_value = timer_group_get_counter_value_in_isr(TIMER_GROUP_0, timer_idx);

    /* Prepare basic event data
       that will be then sent back to the main program task */
    timer_event_t evt;
    evt.timer_group = 0;
    evt.timer_idx = timer_idx;
    evt.timer_counter_value = timer_counter_value;

    /* Clear the interrupt
       and update the alarm time for the timer with without reload */
    if (timer_intr & TIMER_INTR_T0) {
        evt.type = TEST_WITHOUT_RELOAD;
        timer_group_intr_clr_in_isr(TIMER_GROUP_0, TIMER_0);
        timer_counter_value += (uint64_t) (TIMER_INTERVAL0_SEC * TIMER_SCALE);
        timer_group_set_alarm_value_in_isr(TIMER_GROUP_0, timer_idx, timer_counter_value);
    } else if (timer_intr & TIMER_INTR_T1) {
        evt.type = TEST_WITH_RELOAD;
        timer_group_intr_clr_in_isr(TIMER_GROUP_0, TIMER_1);
    } else {
        evt.type = -1; // not supported even type
    }

    /* After the alarm has been triggered
      we need enable it again, so it is triggered the next time */
    timer_group_enable_alarm_in_isr(TIMER_GROUP_0, timer_idx);

    /* Now just send the event data back to the main program task */
    xQueueSendFromISR(timer_queue, &evt, NULL);
}

/*
 * Initialize selected timer of the timer group 0
 *
 * timer_idx - the timer number to initialize
 * auto_reload - should the timer auto reload on alarm?
 * timer_interval_sec - the interval of alarm to set
 */
static void example_tg0_timer_init(int timer_idx,
    bool auto_reload, double timer_interval_sec)
{
    /* Select and initialize basic parameters of the timer */
    timer_config_t config;
    config.divider = TIMER_DIVIDER;
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en = TIMER_PAUSE;
    config.alarm_en = TIMER_ALARM_EN;
    config.intr_type = TIMER_INTR_LEVEL;
    config.auto_reload = auto_reload;
    timer_init(TIMER_GROUP_0, timer_idx, &config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(TIMER_GROUP_0, timer_idx, 0x00000000ULL);

    /* Configure the alarm value and the interrupt on alarm. */
    timer_set_alarm_value(TIMER_GROUP_0, timer_idx, timer_interval_sec * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, timer_idx);
    timer_isr_register(TIMER_GROUP_0, timer_idx, timer_group0_isr,
        (void *) timer_idx, ESP_INTR_FLAG_IRAM, NULL);

    timer_start(TIMER_GROUP_0, timer_idx);
}

/*
 * The main task of this example program
 */
static void timer_example_evt_task(void *arg)
{
    while (1) {
        timer_event_t evt;
        xQueueReceive(timer_queue, &evt, portMAX_DELAY);

        /* Print information that the timer reported an event */
        if (evt.type == TEST_WITHOUT_RELOAD) {
            printf("\nExample timer without reload\n");
            // newCount = count - newCount;
            // revolutionsPerSec = (double) newCount / 7;
            // //Speed in meters per second?
            // //24 inches (approx wheel circ.) in meters is .6096
            // speed = revolutionsPerSec * .6096;
            // printf("speed: %.2f m/s\n", speed);
            // revolutionsPerSec = 0;
            // newCount = count;

            //COLLISION DETECTION
            if(lidarLiteRange <= COLLISIONRANGE)
            {
                mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1400); // NEUTRAL signal in microseconds
            }

        } else {
            printf("\nUNKNOWN EVENT TYPE\n");
        }
        // printf("Group[%d], timer[%d] alarm event\n", evt.timer_group, evt.timer_idx);

        // /* Print the timer values passed by event */
        // printf("------- EVENT TIME --------\n");
        // print_timer_counter(evt.timer_counter_value);

        //  Print the timer values as visible by this task 
        // printf("-------- TASK TIME --------\n");
        // uint64_t task_counter_value;
        // timer_get_counter_value(evt.timer_group, evt.timer_idx, &task_counter_value);
        // print_timer_counter(task_counter_value);
    }
}

///////////////////////////////
///PCNT FUNCTIONS!!!///////////
///////////////////////////////


/* Decode what PCNT's unit originated an interrupt
 * and pass this information together with the event type
 * the main program using a queue.
 */
static void IRAM_ATTR pcnt_example_intr_handler(void *arg)
{
    uint32_t intr_status = PCNT.int_st.val;
    int i;
    pcnt_evt_t evt;
    portBASE_TYPE HPTaskAwoken = pdFALSE;

    for (i = 0; i < PCNT_UNIT_MAX; i++) {
        if (intr_status & (BIT(i))) {
            evt.unit = i;
            /* Save the PCNT event type that caused an interrupt
               to pass it to the main program */
            evt.status = PCNT.status_unit[i].val;
            PCNT.int_clr.val = BIT(i);
            xQueueSendFromISR(pcnt_evt_queue, &evt, &HPTaskAwoken);
            if (HPTaskAwoken == pdTRUE) {
                portYIELD_FROM_ISR();
            }
        }
    }
}

/* Initialize PCNT functions:
 *  - configure and initialize PCNT
 *  - set up the input filter
 *  - set up the counter events to watch
 */

//Initializes the pulse counter
static void pcnt_example_init(void)
{
    /* Prepare configuration for the PCNT unit */
    pcnt_config_t pcnt_config = {
        // Set PCNT input signal and control GPIOs
        .pulse_gpio_num = PCNT_INPUT_SIG_IO,
        .ctrl_gpio_num = PCNT_INPUT_CTRL_IO,
        .channel = PCNT_CHANNEL_0,
        .unit = PCNT_TEST_UNIT,
        // What to do on the positive / negative edge of pulse input?
        .pos_mode = PCNT_COUNT_INC,   // Count up on the positive edge
        .neg_mode = PCNT_COUNT_DIS,   // Keep the counter value on the negative edge
        // What to do when control input is low or high?
        .lctrl_mode = PCNT_MODE_REVERSE, // Reverse counting direction if low
        .hctrl_mode = PCNT_MODE_KEEP,    // Keep the primary counter mode if high
        // Set the maximum and minimum limit values to watch
        .counter_h_lim = PCNT_H_LIM_VAL,
        .counter_l_lim = PCNT_L_LIM_VAL,
    };
    /* Initialize PCNT unit */
    pcnt_unit_config(&pcnt_config);

    /* Configure and enable the input filter */
    pcnt_set_filter_value(PCNT_TEST_UNIT, 100);
    pcnt_filter_enable(PCNT_TEST_UNIT);

    /* Set threshold 0 and 1 values and enable events to watch */
    pcnt_set_event_value(PCNT_TEST_UNIT, PCNT_EVT_THRES_1, PCNT_THRESH1_VAL);
    pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_THRES_1);
    pcnt_set_event_value(PCNT_TEST_UNIT, PCNT_EVT_THRES_0, PCNT_THRESH0_VAL);
    pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_THRES_0);
    /* Enable events on zero, maximum and minimum limit values */
    pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_ZERO);
    pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_H_LIM);
    pcnt_event_enable(PCNT_TEST_UNIT, PCNT_EVT_L_LIM);

    /* Initialize PCNT's counter */
    pcnt_counter_pause(PCNT_TEST_UNIT);
    pcnt_counter_clear(PCNT_TEST_UNIT);

    /* Register ISR handler and enable interrupts for PCNT unit */
    pcnt_isr_register(pcnt_example_intr_handler, NULL, 0, &user_isr_handle);
    pcnt_intr_enable(PCNT_TEST_UNIT);

    /* Everything is set up, now go to counting */
    pcnt_counter_resume(PCNT_TEST_UNIT);
}

//Wheel speed task
//Takes the time between pulses and then converts to wheel speed
static void pulseTask()
{
    timer_queue = xQueueCreate(10, sizeof(timer_event_t));
    example_tg0_timer_init(TIMER_0, TEST_WITHOUT_RELOAD, TIMER_INTERVAL0_SEC);
    // example_tg0_timer_init(TIMER_1, TEST_WITH_RELOAD,    TIMER_INTERVAL1_SEC);
    xTaskCreate(timer_example_evt_task, "timer_evt_task", 2048, NULL, 2, NULL);

    /* Initialize PCNT event queue and PCNT functions */
    pcnt_evt_queue = xQueueCreate(10, sizeof(pcnt_evt_t));
    pcnt_example_init();

    pcnt_evt_t evt;
    portBASE_TYPE res;
    while (1) {
        /* Wait for the event information passed from PCNT's interrupt handler.
         * Once received, decode the event type and print it on the serial monitor.
         */

        //Time between pulses * 6 = time for a rotation (seconds per rotation?) (take inverse)
        res = xQueueReceive(pcnt_evt_queue, &evt, 1000 / portTICK_PERIOD_MS);
        if (res == pdTRUE) {




            pcnt_get_counter_value(PCNT_TEST_UNIT, &count);
            printf("Event PCNT unit[%d]; cnt: %d\n", evt.unit, count);
            if (evt.status & PCNT_STATUS_THRES1_M) {
                printf("THRES1 EVT\n");
            }
            if (evt.status & PCNT_STATUS_THRES0_M) {
                printf("THRES0 EVT\n");
            }
            if (evt.status & PCNT_STATUS_L_LIM_M) {
                printf("L_LIM EVT\n");
            }
            if (evt.status & PCNT_STATUS_H_LIM_M) {
                printf("H_LIM EVT\n");
                currentTime = (uint64_t) esp_timer_get_time();
                double period = currentTime - previousTime;
                double secPeriod = period / 1000000;
                printf("period is: %.2f\n", period);
                // speed = (.62/12) * (secPeriod);
                double sPerRotation = secPeriod * 12;
                double tempSpeed = .62/sPerRotation;
                if(abs(tempSpeed - oldSpeed) >= SPEEDERROR || speed > SPEED_THRESHOLD_FILTER)
                {
                    speed = oldSpeed;
                }
                else {
                    speed = tempSpeed;
                    oldSpeed = speed;
                }
                // printf("Seconds per rotation: %.2f", sPerRotation);
                previousTime = currentTime;
                printf("speed: %.2f m/s\n", speed);

                if(speed < SPEED_THRESHOLD_LOW && speedVar > 1290)
                {
                    printf("Speed too low! Speed: %d Adding 2\n", speedVar);
                    speedVar -= 2;
                    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, speedVar);
                } else if(speed > SPEED_THRESHOLD_HIGH && speedVar < 1310) {
                    printf("Speed too high! Speed: %d subtracting 2\n", speedVar);
                    speedVar += 2;
                    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, speedVar);
                }

            }
            // if (evt.status & PCNT_STATUS_ZERO_M) {
            //     printf("ZERO EVT\n");
            // }
        } else {
            pcnt_get_counter_value(PCNT_TEST_UNIT, &count);
            printf("Current counter value :%d\n", count);
            // currentTime = (uint32_t) (clock() * 1000 / CLOCKS_PER_SEC);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    if(user_isr_handle) {
        //Free the ISR service handle.
        esp_intr_free(user_isr_handle);
        user_isr_handle = NULL;
    }

}

/////////////////////////////////////////
////////////UDP SERVER///////////////////
/////////////////////////////////////////
//Task for recieving UDP messages from the server (Start/Stop the crawler)
static void udp_server_task(void *pvParameters)
{
    char rx_buffer[128];
    char addr_str[128];
    int addr_family;
    int ip_protocol;

    while (1) {
#ifdef CONFIG_EXAMPLE_IPV4
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(RXPORT); //RXPORT is used here since RXPORT receives messages
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
        inet_ntoa_r(dest_addr.sin_addr, addr_str, sizeof(addr_str) - 1);
#else // IPV6
        struct sockaddr_in6 dest_addr;
        bzero(&dest_addr.sin6_addr.un, sizeof(dest_addr.sin6_addr.un));
        dest_addr.sin6_family = AF_INET6;
        dest_addr.sin6_port = htons(RXPORT);
        addr_family = AF_INET6;
        ip_protocol = IPPROTO_IPV6;
        inet6_ntoa_r(dest_addr.sin6_addr, addr_str, sizeof(addr_str) - 1);
#endif

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created");

        int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err < 0) {
            ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        }
        ESP_LOGI(TAG, "Socket bound, port %d", RXPORT);

        while (1) {

            ESP_LOGI(TAG, "Waiting for data");
            struct sockaddr_in6 source_addr; // Large enough for both IPv4 or IPv6
            socklen_t socklen = sizeof(source_addr);
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                break;
            }
            // Data received
            else {
                // Get the sender's ip address as string
                if (source_addr.sin6_family == PF_INET) {
                    inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);
                } else if (source_addr.sin6_family == PF_INET6) {
                    inet6_ntoa_r(source_addr.sin6_addr, addr_str, sizeof(addr_str) - 1);
                }

                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string...
                ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
                ESP_LOGI(TAG, "%s", rx_buffer);
                char delim[] = " ";
                char *ptr = strtok(rx_buffer, delim);
                int index = 0;
                char *tokenArray[4];
                while(ptr != NULL) {
                    tokenArray[index] = ptr;
                    ptr = strtok(NULL, delim);
                    index+=1;
                }

                if(*(tokenArray[0]) == '1' && lidarLiteRange > COLLISIONRANGE)
                {
                    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1310); // HIGH signal in microseconds
                } else {
                    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1400); // NEUTRAL signal in microseconds
                }

                int err = sendto(sock, rx_buffer, len, 0, (struct sockaddr *)&source_addr, sizeof(source_addr));
                if (err < 0) {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                    break;
                }
            }
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}

//////////////////////////////////////////////////////////////////////////
// Alphanumeric Functions ////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

//Hex values for the alphanumeric display numbers
const uint16_t FourteenSegmentASCII[10] = {
  0b000110000111111, /* 0 */
  0b000010000000110, /* 1 */
  0b000000011011011, /* 2 */
  0b000000010001111, /* 3 */
  0b000000011100110, /* 4 */
  0b010000001101001, /* 5 */
  0b000000011111101, /* 6 */
  0b000000000000111, /* 7 */
  0b000000011111111, /* 8 */
  0b000000011101111, /* 9 */
};

int alpha_oscillator() {
  int ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SLAVE_ADDR_ALPHA << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, OSC, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM_DISPLAY, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  vTaskDelay(200 / portTICK_RATE_MS);
  return ret;
}

// Set blink rate to off
int no_blink() {
  int ret;
  i2c_cmd_handle_t cmd2 = i2c_cmd_link_create();
  i2c_master_start(cmd2);
  i2c_master_write_byte(cmd2, ( SLAVE_ADDR_ALPHA << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd2, HT16K33_BLINK_CMD | HT16K33_BLINK_DISPLAYON | (HT16K33_BLINK_OFF << 1), ACK_CHECK_EN);
  i2c_master_stop(cmd2);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM_DISPLAY, cmd2, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd2);
  vTaskDelay(200 / portTICK_RATE_MS);
  return ret;
}

// Set Brightness
int set_brightness_max(uint8_t val) {
  int ret;
  i2c_cmd_handle_t cmd3 = i2c_cmd_link_create();
  i2c_master_start(cmd3);
  i2c_master_write_byte(cmd3, ( SLAVE_ADDR_ALPHA << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd3, HT16K33_CMD_BRIGHTNESS | val, ACK_CHECK_EN);
  i2c_master_stop(cmd3);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM_DISPLAY, cmd3, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd3);
  vTaskDelay(200 / portTICK_RATE_MS);
  return ret;
}

// Task for running the alphanumeric display
static void alphaTask(){
  // Debug
    int ret;
    printf(">> Test Alphanumeric Display: \n");
    long temp;

    // Set up routines
    // Turn on alpha oscillator
    ret = alpha_oscillator();
    if(ret == ESP_OK) {printf("- oscillator: ok \n");}
    else printf("ret is: %d\n", ret);
    // Set display blink off
    ret = no_blink();
    if(ret == ESP_OK) {printf("- blink: off \n");}
    else printf("ret is: %d\n", ret);
    ret = set_brightness_max(0xF);
    if(ret == ESP_OK) {printf("- brightness: max \n");}
    else printf("ret is: %d\n", ret);

    // Write to characters to buffer
    
    uint16_t displaybuffer[8];
    // Continually writes the same command
    while (1) {

    if(speed < 0.5)
    {
        temp=speed * 1000;
    } else {
        temp=0.35 * 1000;
    }

    for(int x = 3; x >= 0; x--)
    {
      int rem = temp % 10;
      // printf("%d\n", rem);
      displaybuffer[x] = FourteenSegmentASCII[rem];
      temp = temp / 10;
    }

    //Add dot to the first number
    displaybuffer[0] |= 1 << 14;


      // Send commands characters to display over I2C
      i2c_cmd_handle_t cmd4 = i2c_cmd_link_create();
      i2c_master_start(cmd4);
      i2c_master_write_byte(cmd4, ( SLAVE_ADDR_ALPHA << 1 ) | WRITE_BIT, ACK_CHECK_EN);
      i2c_master_write_byte(cmd4, (uint8_t)0x00, ACK_CHECK_EN);
      for (uint8_t i=0; i<8; i++) {
        i2c_master_write_byte(cmd4, displaybuffer[i] & 0xFF, ACK_CHECK_EN);
        i2c_master_write_byte(cmd4, displaybuffer[i] >> 8, ACK_CHECK_EN);
      }
      i2c_master_stop(cmd4);
      ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM_DISPLAY, cmd4, 1000 / portTICK_RATE_MS);
      i2c_cmd_link_delete(cmd4);

      // for (int i = 0; i < 8; i++) {
      //     printf("%04x\n", displaybuffer[i]);
      // }

      /*if(ret == ESP_OK) {
        printf("- wrote: H.E.L.P. \n\n");
      }*/

      vTaskDelay(50 / portTICK_RATE_MS);
    }
}


////////////////////////////////////
////////UART FUNCTIONS//////////////
////////////////////////////////////

//UART task for polling the microLidar sensors
static void echo_task(void *arg)
{

    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    //Connfigure and initialize the UARTs
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS);
    uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);

    uart_param_config(UART_NUM_2, &uart_config);
    uart_set_pin(UART_NUM_2, ECHO_TEST_TXD_2, ECHO_TEST_RXD_2, ECHO_TEST_RTS_2, ECHO_TEST_CTS_2);
    uart_driver_install(UART_NUM_2, BUF_SIZE * 2, 0, 0, NULL, 0);


    // Configure a temporary buffer for the incoming data
    uint8_t *data1 = (uint8_t *) malloc(BUF_SIZE);
    uint8_t *data2 = (uint8_t *) malloc(BUF_SIZE);

    // Variables for holding the high and low bytes
    uint8_t highData1=0;
    uint8_t lowData1=0;

    uint8_t highData2=0;
    uint8_t lowData2=0;

    while (1) {
        // Read data from the UART
        int len1 = uart_read_bytes(UART_NUM_1, data1, BUF_SIZE, 20 / portTICK_RATE_MS);
        int len2 = uart_read_bytes(UART_NUM_2, data2, BUF_SIZE, 20 / portTICK_RATE_MS);
        // Write data back to the UART
        //uart_write_bytes(UART_NUM_1, (const char *) data, len);
        for(int i=0; i < BUF_SIZE; i++)
        {
            // If two 0x59s detected, you're at the beginning of the frame
            if(data1[i] == 0x59 && data1[i+1] == 0x59) {
                // Grab the high and low bytes
                lowData1 = data1[i+2];
                highData1 = data1[i+3];
                break;
            }
        }

        for(int i=0; i < BUF_SIZE; i++)
        {
            if(data2[i] == 0x59 && data2[i+1] == 0x59) {
                lowData2 = data2[i+2];
                highData2 = data2[i+3];
                break;
            }
        }
        uint16_t final1 = (highData1 << 8) + lowData1;
        uint16_t final2 = (highData2 << 8) + lowData2;
        //measured_value = (double)final;
        printf("MicroLIDAR 1: %d centimeters\n", final1);
        printf("MicroLIDAR 2: %d centimeters\n", final2);

        //Change steering based on a distance discrepancy threshold
        if ((final1 - final2) > STEERINGERROR){
            mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, 1100); // HIGH signal in microseconds => may have to change values

        }
        else if ((final2 - final1) > STEERINGERROR){
            mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, 1700); // HIGH signal in microseconds => may have to change values
        }
        else{
            mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, 1400); // NEUTRAL signal in microseconds

        }

        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}


////////////////////////////////////
////////I2C FUNCTIONS//////////////
////////////////////////////////////

// Function to initiate i2c -- note the MSB declaration!
// i2c communication is initialized here for both i2c devices
// Alphanumeric display and the LiderLite
static void i2c_master_init(){
  // Debug
  printf("\n>> i2c Config\n");
  int err;

  // Port configuration
  int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;
  // Port configuration (alphanumeric)
  int i2c_master_port_display = I2C_EXAMPLE_MASTER_NUM_DISPLAY;

  /// Define I2C configurations for LidarLite
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;                              // Master mode
  conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;              // Default SDA pin
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
  conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;              // Default SCL pin
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
  conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;       // CLK frequency
  err = i2c_param_config(i2c_master_port, &conf);           // Configure
  if (err == ESP_OK) {printf("- parameters: ok\n");}

  // Install I2C driver
  err = i2c_driver_install(i2c_master_port, conf.mode,
                     I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                     I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
  if (err == ESP_OK) {printf("- initialized: yes\n");}

  /// Define I2C configurations for alphanumeric
  conf.mode = I2C_MODE_MASTER;                              // Master mode
  conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO_DISPLAY;              // Default SDA pin
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
  conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO_DISPLAY;              // Default SCL pin
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
  conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;       // CLK frequency
  err = i2c_param_config(i2c_master_port_display, &conf);           // Configure
  if (err == ESP_OK) {printf("- parameters: ok\n");}

  // Install I2C driver
  err = i2c_driver_install(i2c_master_port_display, conf.mode,
                     I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                     I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
  if (err == ESP_OK) {printf("- initialized: yes\n");}

  // Data in MSB mode
  i2c_set_data_mode(i2c_master_port, I2C_DATA_MODE_MSB_FIRST, I2C_DATA_MODE_MSB_FIRST);

  // Data in MSB mode
  i2c_set_data_mode(i2c_master_port_display, I2C_DATA_MODE_MSB_FIRST, I2C_DATA_MODE_MSB_FIRST);
}

// Utility  Functions //////////////////////////////////////////////////////////

// Utility function to test for I2C device address -- not used in deploy
int testConnection(uint8_t devAddr, int32_t timeout) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  int err = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  return err;
}

// Utility function to scan for i2c device
static void i2c_scanner() {
  int32_t scanTimeout = 1000;
  printf("\n>> I2C scanning ..."  "\n");
  uint8_t count = 0;
  for (uint8_t i = 1; i < 127; i++) {
    // printf("0x%X%s",i,"\n");
    if (testConnection(i, scanTimeout) == ESP_OK) {
      printf( "- Device found at address: 0x%X%s", i, "\n");
      count++;
    }
  }
  if (count == 0) {printf("- No I2C devices found!" "\n");}
}

////////////////////////////////////////////////////////////////////////////////

// ADXL343 Functions ///////////////////////////////////////////////////////////

// Get Device ID
int getDeviceID(uint8_t *data) {
  int ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, ADXL343_REG_DEVID, ACK_CHECK_EN);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | READ_BIT, ACK_CHECK_EN);
  i2c_master_read_byte(cmd, data, ACK_CHECK_DIS);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  return ret;
}

// Write one byte to register
int writeRegister(uint8_t reg, uint8_t data) {
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);

    i2c_master_write_byte(cmd, data, ACK_CHECK_EN);

    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
//    printf("\n%d",ret);
    return ret;

}

// Read register
uint8_t readRegister(uint8_t reg) {
//    int ret;
    uint8_t data = 0;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_start(cmd);

    i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | READ_BIT, ACK_CHECK_EN);

    i2c_master_read_byte(cmd, &data, ACK_CHECK_DIS);

    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return data;
}

// read 16 bits (2 bytes)
// Function used to read the LidarLite according to the garmin data sheet
uint16_t read16Dis() {
    uint8_t data = 0;
    uint8_t data2 = 0;
    uint16_t overall = 0;
 
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_cmd_handle_t cmd2 = i2c_cmd_link_create();
    i2c_cmd_handle_t cmd3 = i2c_cmd_link_create();

    uint8_t reg2 = 0x00;
    uint8_t reg3 = 0x04;
    uint8_t reg4 = 0x8F;

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg2, ACK_CHECK_EN);

    i2c_master_write_byte(cmd, reg3, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);

    vTaskDelay(20 / portTICK_RATE_MS);

    i2c_master_start(cmd2);
    i2c_master_write_byte(cmd2, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd2, reg4, ACK_CHECK_EN);
    i2c_master_stop(cmd2);
    i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd2, 1000 / portTICK_RATE_MS);
    //i2c_cmd_link_delete(cmd);

    i2c_master_start(cmd3);
    i2c_master_write_byte(cmd3, ( SLAVE_ADDR << 1 ) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd3, &data, ACK_VAL);
    i2c_master_read_byte(cmd3, &data2, ACK_VAL);

    i2c_master_stop(cmd3);

    i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd3, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    i2c_cmd_link_delete(cmd2);
    i2c_cmd_link_delete(cmd3);

    overall = (data << 8) + data2;
    return overall;
}

void setRange(range_t range) {
  /* Read the data format register to preserve bits */
  uint8_t format = readRegister(ADXL343_REG_DATA_FORMAT);

  /* Update the data rate */
  format &= ~0x0F;
  format |= range;

  /* Make sure that the FULL-RES bit is enabled for range scaling */
  format |= 0x08;

  /* Write the register back to the IC */
  writeRegister(ADXL343_REG_DATA_FORMAT, format);

}

range_t getRange(void) {
  /* Red the data format register to preserve bits */
  return (range_t)(readRegister(ADXL343_REG_DATA_FORMAT) & 0x03);
}

dataRate_t getDataRate(void) {
  return (dataRate_t)(readRegister(ADXL343_REG_BW_RATE) & 0x0F);
}

////////////////////////////////////////////////////////////////////////////////

void getDistance(float * dis){
    lidarLiteRange = read16Dis();
    printf("LIDARLite: %.2d  centimeters\n", lidarLiteRange);
}

// Task to continuously poll acceleration and calculate roll and pitch
static void test_lidarLite() {
  printf("\n>> Polling LidarLite\n");
  while (1) {
    float output;
    vTaskDelay(400 / portTICK_RATE_MS);
    getDistance(&output);
  }
}

// Function for initializing PWM
static void mcpwm_example_gpio_initialize(void)
{
    printf("initializing mcpwm servo control gpio......\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, 12);    //Set GPIO 18 as PWM0A, to which servo is connected
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0A, 33);    //Set GPIO 33 as PWM0A, to which servo is connected
}

// Calibrates the ESC!
void calibrateESC() {
    printf("HIGH!\n");
  vTaskDelay(3000 / portTICK_PERIOD_MS);  // Give yourself time to turn on crawler
  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 2100); // HIGH signal in microseconds
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  printf("LOW!\n");
  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 700);  // LOW signal in microseconds
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  printf("NEUTRAL!\n");
  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1400); // NEUTRAL signal in microseconds
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 1400); // reset the ESC to neutral (non-moving) value
}

// Tests the steering on the ESC!
void cycleSteering() {
  printf("STEER ONE WAY!\n");
  vTaskDelay(3000 / portTICK_PERIOD_MS);  // Give yourself time to turn on crawler
  mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, 2100); // HIGH signal in microseconds
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  printf("STEER THE OTHER WAY!\n");
  mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, 700);  // LOW signal in microseconds
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  printf("STEER NEUTRAL!\n");
  mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, 1400); // NEUTRAL signal in microseconds
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, 1400); // reset the ESC to neutral (non-moving) value
}

void app_main(void)
{

    /* Install UART driver for interrupt-driven reads and writes */
    ESP_ERROR_CHECK( uart_driver_install(UART_NUM_0,
      256, 0, 0, NULL, 0) );

    /* Tell VFS to use UART driver */
    esp_vfs_dev_uart_use_driver(UART_NUM_0);
    printf("Testing servo motor.......\n");
    //2. initial mcpwm configuration
    printf("Configuring Initial Parameters of mcpwm......\n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50;    //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings
    //1. mcpwm gpio initialization
    mcpwm_example_gpio_initialize();


    // Routine
    i2c_master_init();
    i2c_scanner();

    ESP_ERROR_CHECK(nvs_flash_init());
    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
    * Read "Establishing Wi-Fi or Ethernet Connection" section in
    * examples/protocols/README.md for more information about this function.
    */
    ESP_ERROR_CHECK(example_connect());

    // cycleSteering();
    calibrateESC();

    // Create task to poll ADXL343
    xTaskCreate(alphaTask, "alphaTask", 2048, NULL, 13, NULL);
    xTaskCreate(pulseTask, "pulseTask", 2048, NULL, 14, NULL);
    xTaskCreate(test_lidarLite,"test_lidarLite", 4096, NULL, 5, NULL);
    xTaskCreate(echo_task, "uart_echo_task", 2048, NULL, 10, NULL);
    xTaskCreate(udp_server_task, "udp_server_task", 4096, NULL, 6, NULL);
}