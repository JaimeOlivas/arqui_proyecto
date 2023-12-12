/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "driver/i2c.h"
#include <driver/adc.h>
#include <driver/uart.h>
#include "esp_err.h"
#include "esp_log.h"
#include "lvgl.h"
#include "esp_lvgl_port.h"
#include "gpio_JRL/gpio.h"
#include "gpio_JRL/gpio.c"
#include "esp_lcd_panel_vendor.h"


static const char *TAG = "example";

#define I2C_HOST  0

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// Please update the following configuration according to your LCD spec //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define EXAMPLE_LCD_PIXEL_CLOCK_HZ    (400 * 1000)
#define EXAMPLE_PIN_NUM_SDA           21
#define EXAMPLE_PIN_NUM_SCL           22
#define EXAMPLE_PIN_NUM_RST           -1
#define EXAMPLE_I2C_HW_ADDR           0x3C

// The pixel number in horizontal and vertical

#define EXAMPLE_LCD_H_RES              128
#define EXAMPLE_LCD_V_RES              64

// Bit number used to represent command and parameter
#define EXAMPLE_LCD_CMD_BITS           8
#define EXAMPLE_LCD_PARAM_BITS         8
#define S_IN    PIN18 //SW1
#define S_OUT   PIN19 //SW2
#define TEMCOR  PIN2
#define BTN_ENC PIN4
#define LED_ENC PIN5
#define DOOR    PIN16
#define FAN     PIN17
#define ITERACIONES 250

#define RED     PIN14
#define GREEN   PIN13
#define BLUE    PIN12
void init_io(void){
    gpio_pinMode(RED, OUTPUT);
    gpio_pinMode(GREEN, OUTPUT);
    gpio_pinMode(BLUE, OUTPUT);
    gpio_pinMode(LED_ENC, OUTPUT);
    gpio_pinMode(DOOR, OUTPUT);
    gpio_pinMode(FAN, OUTPUT);
    gpio_pinMode(S_IN, INPUT_PULLUP);
    gpio_pinMode(S_OUT, INPUT_PULLUP);
    gpio_pinMode(BTN_ENC, INPUT_PULLUP);
    gpio_pinMode(TEMCOR, OUTPUT);
    gpio_write(RED, HIGH);
    gpio_write(GREEN, HIGH);
    gpio_write(BLUE, HIGH);
    gpio_write(TEMCOR, LOW);
    gpio_write(LED_ENC, LOW);
    gpio_write(DOOR, LOW);
    gpio_write(FAN, LOW);
   
}
#define TXD_PIN (UART_PIN_NO_CHANGE)
#define RXD_PIN (UART_PIN_NO_CHANGE)
#define BAUD_RATE 9600
char *status = "";
void init_uart(void){
    uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_param_config(UART_NUM_0, &uart_config);
    uart_set_pin(UART_NUM_0, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_0, 1024, 0, 0, NULL, 0); 
}
extern void example_lvgl_demo_ui(lv_disp_t *disp, uint8_t selec);

/* The LVGL port component calls esp_lcd_panel_draw_bitmap API for send data to the screen. There must be called
lvgl_port_flush_ready(disp) after each transaction to display. The best way is to use on_color_trans_done
callback from esp_lcd IO config structure. In IDF 5.1 and higher, it is solved inside LVGL port component. */
static bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_disp_t * disp = (lv_disp_t *)user_ctx;
    lvgl_port_flush_ready(disp);
    return false;
}
    float tv, tr, y, TEMPAMB, TEMPCOR, tempcor[5] = {0};
    int adc_read1 = 0, adc_read2 = 0;
void init_adc(void){

    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);
    adc1_config_width(ADC_WIDTH_BIT_12);
    
    adc2_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_11); //PIN34
    //adc2_config_width(ADC_WIDTH_BIT_12);
}    
#define MAX_PEOPLE 8 //maximo 8 personas dentro
#define MAX_TEMP  29 
#define MIN_TEMP  12

bool FLAG_ENC =  false;
bool MODO = false; //false: AUTO true: ON
bool COOL = false; //false: cool true: heat
uint8_t people_in = 0;
uint16_t iteraciones = 0;
void app_main(void)
{   
     init_io();
     init_adc();
     init_uart();
    ESP_LOGI(TAG, "Initialize I2C bus");
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = EXAMPLE_PIN_NUM_SDA,
        .scl_io_num = EXAMPLE_PIN_NUM_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_HOST, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_HOST, I2C_MODE_MASTER, 0, 0, 0));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = EXAMPLE_I2C_HW_ADDR,
        .control_phase_bytes = 1,               // According to SSD1306 datasheet
        .lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS,   // According to SSD1306 datasheet
        .lcd_param_bits = EXAMPLE_LCD_CMD_BITS, // According to SSD1306 datasheet
        .dc_bit_offset = 6,                     // According to SSD1306 datasheet

    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)I2C_HOST, &io_config, &io_handle));

    ESP_LOGI(TAG, "Install SSD1306 panel driver");
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .bits_per_pixel = 1,
        .reset_gpio_num = EXAMPLE_PIN_NUM_RST,
    };

    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(io_handle, &panel_config, &panel_handle));


    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));


    ESP_LOGI(TAG, "Initialize LVGL");
    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    lvgl_port_init(&lvgl_cfg);

    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = panel_handle,
        .buffer_size = EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES,
        .double_buffer = true,
        .hres = EXAMPLE_LCD_H_RES,
        .vres = EXAMPLE_LCD_V_RES,
        .monochrome = true,
        .rotation = {
            .swap_xy = false,
            .mirror_x = false,
            .mirror_y = false,
        }
    };
    lv_disp_t * disp = lvgl_port_add_disp(&disp_cfg);
    /* Register done callback for IO */
    const esp_lcd_panel_io_callbacks_t cbs = {
        .on_color_trans_done = notify_lvgl_flush_ready,
    };
    esp_lcd_panel_io_register_event_callbacks(io_handle, &cbs, disp);

    /* Rotation of the screen */
   /// lv_disp_set_rotation(disp, LV_DISP_ROT_NONE);

   //ESP_LOGI(TAG, "Display LVGL Scroll Text");
    lv_obj_t *scr = lv_disp_get_scr_act(disp);
    lv_obj_t *label = lv_label_create(scr);
    lv_label_set_text(label, "Sistema: OFF\0");
    while(1){

        vTaskDelay(100/ (( TickType_t ) 1000 / 100));
        
        if(gpio_read(BTN_ENC) == 0X00){  //encender y apagar sistema
            if(!FLAG_ENC){
                //printf("Sistema: ON\n\r");
                status = "SISTEMA: ON\n\r";
                uart_write_bytes(UART_NUM_0, status, strlen(status));
                vTaskDelay(10/ (( TickType_t ) 1000 / 100));
                gpio_write(LED_ENC, HIGH);
                esp_lcd_panel_reset(panel_handle);
                //example_lvgl_demo_ui(disp, 0);
                lv_label_set_text(label, "SISTEMA: ON\nDOOR: closed\0");
                vTaskDelay(10/ (( TickType_t ) 1000 / 100)); 
                FLAG_ENC = true;    
            }else {
                status = "SISTEMA: OFF\n\r";
                uart_write_bytes(UART_NUM_0, status, strlen(status));
                vTaskDelay(10/ (( TickType_t ) 1000 / 100));
                gpio_write(LED_ENC, HIGH); 
                esp_lcd_panel_reset(panel_handle);
                lv_label_set_text(label, "SISTEMA: OFF\0");
                //example_lvgl_demo_ui(disp, 1);   
                vTaskDelay(10/ (( TickType_t ) 1000 / 100));    
                FLAG_ENC = false;
                gpio_write(LED_ENC, LOW);
            }
            while(gpio_read(BTN_ENC) == 0X00);   

        }
        if(FLAG_ENC){


            adc_read1 = adc1_get_raw(ADC1_CHANNEL_0);

            for(int l = 0; l <= 4; ++l){
               tempcor[l] = adc1_get_raw( ADC1_CHANNEL_4); 
            }

            TEMPCOR = 0;
            for (int l = 0; l <= 4; ++l)
            {
                TEMPCOR += tempcor[l];
            }

            TEMPCOR /= 5;
            adc_read2 = adc1_get_raw( ADC1_CHANNEL_4);
            TEMPCOR = adc_read2*330/4095.0;
            tv = 3.3 * adc_read1 / 4095.0;
            tr = tv * 10000.0 / (3.3 - tv);
            y = log(tr/10000.0);
            y = (1.0/298.15) + (y *(1.0/4050.0));
            TEMPAMB = 1.0/y;
            TEMPAMB = TEMPAMB -273.15;  //debe ser temperatura del lm
           // printf("TEMPCOR = %0.2f\n\r", TEMPCOR);
           // vTaskDelay(1000/ (( TickType_t ) 1000 / 100));

            if(gpio_read(S_IN) == 0X00){

                if(people_in < MAX_PEOPLE && TEMPCOR < MAX_TEMP && TEMPCOR > MIN_TEMP){
                    status = "DOOR: OPEN \n\r";
                    uart_write_bytes(UART_NUM_0, status, strlen(status));
                    vTaskDelay(10/ (( TickType_t ) 1000 / 100)); 
                    gpio_write(LED_ENC, HIGH);  
                    esp_lcd_panel_reset(panel_handle);
                    lv_label_set_text(label, "SISTEMA: ON\nDOOR: open\0");
                    vTaskDelay(5000/ (( TickType_t ) 1000 / 100));
                    people_in += 1;
                    status = "DOOR: CLOSED\n\r";
                    uart_write_bytes(UART_NUM_0, status, strlen(status));
                    vTaskDelay(10/ (( TickType_t ) 1000 / 100));
                    esp_lcd_panel_reset(panel_handle); 
                    lv_label_set_text(label, "SISTEMA: ON\nDOOR: closed\0");

                }else if (people_in == MAX_PEOPLE && TEMPCOR < MAX_TEMP && TEMPCOR > MIN_TEMP){
                    status = "WE ARE FULL WAIT\n\r";
                    uart_write_bytes(UART_NUM_0, status, strlen(status));
                    vTaskDelay(10/ (( TickType_t ) 1000 / 100)); 
                    esp_lcd_panel_reset(panel_handle); 
                    lv_label_set_text(label, "SISTEMA: ON\nWe are full,\nwait\0"); 

                } else if ( (TEMPCOR > MAX_TEMP || TEMPCOR < MIN_TEMP)){
                    status = "TEMP_OUT_OF_RANGE\n\r";
                    uart_write_bytes(UART_NUM_0, status, strlen(status));
                    vTaskDelay(10/ (( TickType_t ) 1000 / 100)); 
                    esp_lcd_panel_reset(panel_handle); 
                    lv_label_set_text(label, "SISTEMA: ON\nTemp_out\nof_range\0");

                    for(int r = 0; r < 5;++r){
                        gpio_write(RED, HIGH);
                        gpio_write(BLUE, LOW);
                        vTaskDelay(500/ (( TickType_t ) 1000 / 100));
                        gpio_write(BLUE, HIGH);
                        gpio_write(RED, LOW);
                        vTaskDelay(500/ (( TickType_t ) 1000 / 100));
                    }
                    gpio_write(RED, HIGH);
                    status = "DOOR: CLOSE\n\r";
                    uart_write_bytes(UART_NUM_0, status, strlen(status));                    
                    vTaskDelay(10/ (( TickType_t ) 1000 / 100)); 
                    esp_lcd_panel_reset(panel_handle); 
                    lv_label_set_text(label, "SISTEMA: ON\nDOOR: closed\0");

                }
                while(gpio_read(S_IN) == 0X00);
            } else if(gpio_read(S_OUT) == 0X00){
                
                if(people_in>0) people_in -= 1;
                lv_label_set_text(label, "SISTEMA: ON\nDOOR: closed\0");
                while(gpio_read(S_OUT) == 0X00);
            }
        }
    }
}