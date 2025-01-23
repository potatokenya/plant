/*
 * A components demo for course 02112
 */

#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include <string.h>
#include "esp_log.h"
#include "esp_spiffs.h"

//Driver libraries
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/adc.h"

//Display libraies
#include "ssd1306.h"
#include "font8x8_basic.h"

//Temperature/humidity sensor library
#include <am2320.h>

//Stemma soil sensor library
#include "Adafruit_Stemma_soil_sensor.h"

#define tag "EXAMPLE_ALL"

#define RED_LED_GPIO 8
//#define BUTTON_1_GPIO_PIN 18
#define BUTTON_2_GPIO_PIN 19

#define I2C_MASTER_FREQ_HZ 75000 //Reduce it to 50000 if the temperature/umidity sensor fails
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0
#define I2C_MASTER_SDA_GPIO 2
#define I2C_MASTER_SCL_GPIO 3
#define I2C_NUM 0

//PWM library to control LED intensity and/or play tone on buzzer
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO_3R       (4) // Define the output GPIO for red
#define LEDC_OUTPUT_IO_3G       (5) // Define the output GPIO for green
#define LEDC_OUTPUT_IO_2R       (6) // Define the output GPIO for red
#define LEDC_OUTPUT_IO_2G       (7) // Define the output GPIO for green
#define LEDC_OUTPUT_IO_1R       (9) // Define the output GPIO for red
#define LEDC_OUTPUT_IO_1G       (8) // Define the output GPIO for green

#define LEDC_CHANNEL_RED1       LEDC_CHANNEL_0
#define LEDC_CHANNEL_GREEN1     LEDC_CHANNEL_1
#define LEDC_CHANNEL_RED2       LEDC_CHANNEL_2
#define LEDC_CHANNEL_GREEN2     LEDC_CHANNEL_3
#define LEDC_CHANNEL_RED3       LEDC_CHANNEL_4
#define LEDC_CHANNEL_GREEN3     LEDC_CHANNEL_5
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (4096) // Set duty to 50%. (2 ** 13) * 50% = 4096
#define LEDC_FREQUENCY          (1000) // Frequency in Hertz. Set frequency at 1 kHz

#define BUZZ_TIMER              LEDC_TIMER_1
#define BUZZ_MODE               LEDC_LOW_SPEED_MODE
#define BUZZ_OUTPUT_IO          (1) // Define the output GPIO for red
#define BUZZ_CHANNEL            ADC1_CHANNEL_1
#define BUZZ_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define BUZZ_DUTY               (4096) // Set duty to 50%. (2 ** 13) * 50% = 4096
#define BUZZ_FREQUENCY          (1000) // Frequency in Hertz. Set frequency at 1 kHz
#define TICKS_PER_DAY (24 * 60 * 60 * configTICK_RATE_HZ)
TickType_t start_tick = 0;  

void buzz_start(int delay);
void sound(int freq, int delay);
void buzz_stop(int delay);

int warnings = 0; //Global variable for warnings

float temperature_out = 0;
float humidity_out = 0;
float temperature_in = 0;
float humidity_in = 0;

typedef struct {
    float temperature_out;
    float humidity_out;
    float temperature_in;
    float humidity_in;
    int light;
} SensorData;

SensorData sensor_data;

void print_info(){
	/* Print chip information */
	esp_chip_info_t chip_info;
	uint32_t flash_size;
	esp_chip_info(&chip_info);
	printf("This is %s chip with %d CPU core(s), WiFi%s%s, ",
		   CONFIG_IDF_TARGET,
		chip_info.cores,
		(chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
		   (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

	unsigned major_rev = chip_info.revision / 100;
	unsigned minor_rev = chip_info.revision % 100;
	printf("silicon revision v%d.%d, ", major_rev, minor_rev);
	if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
		printf("Get flash size failed");
		return;
	}

	printf("%luMB %s flash\n", flash_size / (1024 * 1024),
		   (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

	printf("Minimum free heap size: %ld bytes\n", esp_get_minimum_free_heap_size());
}
void display_demo(int *LIGHT_POINTER, int *TEMP_OUT_POINTER, int *HUM_OUT_POINTER, int *HUM_IN_POINTER, int *TEMP_IN_POINTER){
    SSD1306_t dev;

    //Initialize the display (shared i2c) only once after boot.
    i2c_master_shared_i2c_init(&dev);

    ssd1306_init(&dev, 128, 64);

	int TOO_MUCH_LIGHT = *LIGHT_POINTER;//ADDED: Light pointer.
    int TEMP_OUT = *TEMP_OUT_POINTER;
    int HUM_OUT = *HUM_OUT_POINTER;
    int TEMP_IN = *TEMP_IN_POINTER;
    int HUM_IN = *HUM_IN_POINTER;

    //Display warnings
    if(warnings > 0){

        ESP_LOGI(tag, "Displaying warnings");
        ssd1306_clear_screen(&dev, false);
        ssd1306_clear_screen(&dev, false);
        ssd1306_clear_screen(&dev, false);
        ssd1306_clear_screen(&dev, false);
        ssd1306_contrast(&dev, 0xff);

        ssd1306_display_text(&dev, 3, "WARNING", 7, false); //ADDED: changed it to be in the center
		ssd1306_hardware_scroll(&dev, SCROLL_RIGHT);

		vTaskDelay(3000 / portTICK_PERIOD_MS);

        ssd1306_hardware_scroll(&dev, SCROLL_STOP);

        ssd1306_clear_screen(&dev, false);
        ssd1306_clear_screen(&dev, false);
        ssd1306_clear_screen(&dev, false);
        ssd1306_clear_screen(&dev, false);

        char warning_text[20];
        snprintf(warning_text, sizeof(warning_text), "  WARNING(S): %d", warnings);
        ssd1306_display_text(&dev, 0, warning_text, strlen(warning_text), false);

        if(TOO_MUCH_LIGHT == 1){//ADDED: display warning about light pointer.
            ssd1306_display_text(&dev, 2, " Too much light! ", 17, false);
        }

        if(TEMP_OUT == 1){
            ssd1306_display_text(&dev, 3, "  Temp(out)  low! ", 17, false);
        } else if(TEMP_OUT == 2){
            ssd1306_display_text(&dev, 3, "  Temp(out) high! ", 17, false);
        }

        if(HUM_OUT == 1){
            ssd1306_display_text(&dev, 4, "  Hum(out)  low!  ", 17, false);
        } else if(HUM_OUT == 2){
            ssd1306_display_text(&dev, 4, "  Hum(out)  high! ", 17, false);
        }


        if(TEMP_IN == 1){
            ssd1306_display_text(&dev, 5, "  Temp(in)  low!  ", 17, false);
        } else if(TEMP_IN == 2){
            ssd1306_display_text(&dev, 5, "  Temp(in) high!  ", 17, false);
        }

        if(HUM_IN == 1){
            ssd1306_display_text(&dev, 6, "  Hum(in)   low!  ", 17, false);
        } else if(HUM_IN == 2){
            ssd1306_display_text(&dev, 6, "  Hum(in)  high!  ", 17, false);
        }

        vTaskDelay(5000 / portTICK_PERIOD_MS);

        ssd1306_clear_screen(&dev, false);
        ssd1306_clear_screen(&dev, false);
    } 

    //Display plant info/messurements
    ESP_LOGI(tag, "Plant info.");
    ssd1306_clear_screen(&dev, false);
    ssd1306_clear_screen(&dev, false);
    ssd1306_clear_screen(&dev, false);
    ssd1306_clear_screen(&dev, false);
    ssd1306_contrast(&dev, 0xff);
    ssd1306_display_text(&dev, 0, "   Plant info   ", 17, false);
    ssd1306_display_text(&dev, 1, "Sen:  mes:  id:  ", 18, false);

    char tempout_text[20];
    snprintf(tempout_text, sizeof(tempout_text), "T(O) %.1f  10-30", temperature_out);
    ssd1306_display_text(&dev, 2, tempout_text, 17, false);

    char tempin_text[20];
    snprintf(tempin_text, sizeof(tempin_text), "T(I) %.1f  10-30", temperature_in);
    ssd1306_display_text(&dev, 3, tempin_text, 17, false);

    char humout_text[20];
    snprintf(humout_text, sizeof(humout_text), "H(O) %.1f%% 40-60", humidity_out);
    ssd1306_display_text(&dev, 4, humout_text, 17, false);

    char humin_text[20];
    snprintf(humin_text, sizeof(humin_text), "H(I) %.1f%% 60-70", humidity_in);
    ssd1306_display_text(&dev, 5, humin_text, 17, false);

    char light_text[20];
    
    if(TOO_MUCH_LIGHT == 1){
        snprintf(light_text, sizeof(light_text), "L    too bright");
        ssd1306_display_text(&dev, 6, light_text, 17, false);
    } 

    if(warnings == 0){
        ssd1306_display_text(&dev, 7, "      (^_^)      ", 17, false);
    } else if(warnings > 0){
        ssd1306_display_text(&dev, 7, "      (T-T)      ", 17, false);
    }
}

void temperaure_humidity_demo(int *TEMP_OUT_POINTER, int *HUM_OUT_POINTER){
    i2c_dev_t dev = {0};

    //Initialize the sensor (shared i2c) only once after boot.
    ESP_ERROR_CHECK(am2320_shared_i2c_init(&dev, I2C_NUM));

    float temperature, humidity;


    esp_err_t res = am2320_get_rht(&dev, &temperature, &humidity);
    if (res == ESP_OK)
        ESP_LOGI(tag, "Temperature: %.1f°C, Humidity: %.1f%%", temperature, humidity);
    else
        ESP_LOGE(tag, "Error reading data: %d (%s)", res, esp_err_to_name(res));

    //500 ms delay
    vTaskDelay((500) / portTICK_PERIOD_MS);

    //Temperature

    //temperature >= 10 && temperature <= 28

    if(temperature >= 10 && temperature <= 30){
        ESP_LOGI(tag, "Temperature is good!");
        *TEMP_OUT_POINTER = 0;
       
    } else if (temperature <= 10) {
        ESP_LOGI(tag, "Temperature is too low!");
        *TEMP_OUT_POINTER = 1;
        warnings++;

    } else if (temperature >= 30) {
        ESP_LOGI(tag, "Temperature is too high!");
        *TEMP_OUT_POINTER = 2;
        warnings++;

    }else{
        return;
    }

    vTaskDelay((1000) / portTICK_PERIOD_MS);

    //Humidity
    if(humidity >= 40 && humidity <= 60){
        ESP_LOGI(tag, "Humidity is good!");
        *HUM_OUT_POINTER = 0;
       
    } else if (humidity < 40) {
        ESP_LOGI(tag, "Humidity is too low! Ideal: 40 - 60 %%");
        *HUM_OUT_POINTER = 1;
        warnings++;
        
    } else if (humidity > 60) {
        ESP_LOGI(tag, "Humidity is too high! Ideal: 40 - 60 %%");
        *HUM_OUT_POINTER = 2;
        

    }else{
        return;
    }

    temperature_out = temperature;
    humidity_out = humidity;
    sensor_data.temperature_out = temperature;
    sensor_data.humidity_out = humidity;
}

void stemma_soil_demo(int *HUM_IN_POINTER, int *TEMP_IN_POINTER){
    int ret = ESP_OK;
    uint16_t moisture_value = 0;
    // uint16_t dev = {0};
    float temperature_value;
    //float moisture_value = 0;

    //int *MOISTURE_BAD = (int*) MST_POINTER;//ADDED: Moisture Pointer

    //Initialize the sensor (shared i2c) only once after boot.
    ESP_ERROR_CHECK(adafruit_stemma_soil_sensor_shared_i2c_init());


    ret = adafruit_stemma_soil_sensor_read_moisture(I2C_NUM, &moisture_value);


    if (ret == ESP_OK)
    {
        ESP_LOGI(tag, "Adafruit Stemma sensor value: = %u", moisture_value);
    }

    ret = adafruit_stemma_soil_sensor_read_temperature(I2C_NUM, &temperature_value);

    if (ret == ESP_OK)
    {
        ESP_LOGI(tag, "Adafruit Stemma sensor value: = %.1f %%", temperature_value);
    }

    //find the moisutre value in percentage in the range 0-800
    humidity_in = (moisture_value / 1200.0) * 100;

    
    if(humidity_in >= 60 && humidity_in <= 70){
        ESP_LOGI(tag, "Moisture is good, moisture value: %.1f %%", humidity_in);
        *HUM_IN_POINTER = 0;

    } else if (humidity_in < 60){
        ESP_LOGI(tag, "Too dry, moisture value: %.1f %%. Ideal: 60 - 70 %%", humidity_in);
        *HUM_IN_POINTER = 1;
        warnings++;
        
    } else if (humidity_in > 70){
        ESP_LOGI(tag, "Too wet, moisture value: %.1f %%. Ideal: 60 - 70 %%", humidity_in);
        *HUM_IN_POINTER = 2;
        warnings++;

    } else {
        return;
    }

    if(temperature_value >= 10 && temperature_value <= 30){
        ESP_LOGI(tag, "Temperature is good, moisture value: %.1f", temperature_value);
        *TEMP_IN_POINTER = 0;

    } else if (temperature_value < 10){
        ESP_LOGI(tag, "Too cold, temperature value: %.1f °C. Ideal: 60 - 70 %%", temperature_value);
        *TEMP_IN_POINTER = 1;
        warnings++;
        
    } else if (temperature_value > 30){
        ESP_LOGI(tag, "Too warm, temperature value: %.1f °C. Ideal: 60 - 70 %%", temperature_value);
        *TEMP_IN_POINTER = 2;
        warnings++;

    } else {
        return;
    }
    
    //500 ms delay
    vTaskDelay((500) / portTICK_PERIOD_MS);

    temperature_in = temperature_value;
    // humidity_in = moisture_value;
    sensor_data.temperature_in = temperature_value;
    sensor_data.humidity_in = humidity_in;
}

void led_fade_demo(int *LIGHT_POINTER, int *TEMP_OUT_POINTER, int *HUM_OUT_POINTER, int *HUM_IN_POINTER, int *TEMP_IN_POINTER){
    // Prepare and then apply the LEDC PWM timer configuration (one time can drive multiple channels)
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 1 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

//Here comes the RGB LED 1 for the light sensor
   // Prepare and then apply the LEDC PWM channel configuration
	ledc_channel_config_t ledc_channel_red1 = {
		.speed_mode     = LEDC_MODE,
		.channel        = LEDC_CHANNEL_RED1,
		.timer_sel      = LEDC_TIMER,
		.intr_type      = LEDC_INTR_DISABLE,
		.gpio_num       = LEDC_OUTPUT_IO_1R,
		.duty           = 0, // Set duty to 0%
		.hpoint         = 0
	};
	ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_red1));

	ledc_channel_config_t ledc_channel_green1 = {
		.speed_mode     = LEDC_MODE,
		.channel        = LEDC_CHANNEL_GREEN1,
		.timer_sel      = LEDC_TIMER,
		.intr_type      = LEDC_INTR_DISABLE,
		.gpio_num       = LEDC_OUTPUT_IO_1G,
		.duty           = 0, // Set duty to 0%
		.hpoint         = 0
	};
	ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_green1));


    // Now the initialization is done
    ESP_LOGI(tag, "Initialization complete. Displaying either green or red.");

	ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_RED1, 0));
	ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_GREEN1, 0));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_RED2, 0));
	ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_GREEN2, 0));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_RED3, 0));
	ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_GREEN3, 0));
	//ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_BLUE, 0));
	// Update duty to apply the new value
	ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_RED1));
	ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_GREEN1));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_RED2));
	ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_GREEN2));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_RED3));
	ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_GREEN3));
	//ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_BLUE));
	//1000 ms delay
	vTaskDelay((10) / portTICK_PERIOD_MS);

    int duty = 8192-1;

	int TOO_MUCH_LIGHT = *LIGHT_POINTER;//ADDED: Light pointer.

    ESP_LOGI(tag, "Light sensor: %d\n", TOO_MUCH_LIGHT);

	if(TOO_MUCH_LIGHT == 0){//ADDED: RGB LED turns red as a warning
		// Set duty
		ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_RED1, 4096));
		ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_GREEN1, duty));
		//ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_BLUE, 0));
		// Update duty to apply the new value
		ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_RED1));
		ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_GREEN1));
		//ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_BLUE));
		//1000 ms delay
		vTaskDelay((10) / portTICK_PERIOD_MS);
		printf("Toomuchlight = 0 %d\n", duty);//ADDED: insert code for red RGB LED, that should stay on until all warnings are taken care of and the code refreshes.
	} else if (TOO_MUCH_LIGHT == 1){
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_RED1, duty));
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_GREEN1, 4096));
        // ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_BLUE, duty));
        // Update duty to apply the new value
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_RED1));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_GREEN1));
        // ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_BLUE));
        //1000 ms delay
        vTaskDelay((10) / portTICK_PERIOD_MS);
        printf("Toomuchlight = 1 %d\n", duty);
    } else {
        return;
    }
//RGB LED 1 for light sensor is done

//RGB LED 2 for soil sensor begins
// Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel_red2 = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_RED2,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO_2R,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_red2));

    ledc_channel_config_t ledc_channel_green2 = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_GREEN2,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO_2G,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_green2));

    ESP_LOGI(tag,"Temperature and humidity (inside/soil sensor): %d\n", (*TEMP_IN_POINTER || *HUM_IN_POINTER));

    int soil = (*TEMP_IN_POINTER || *HUM_IN_POINTER);

    if (soil >= 1){ //Red
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_RED2, 4096));
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_GREEN2, duty));
        // ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_BLUE, duty));
        // Update duty to apply the new value
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_RED2));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_GREEN2));
        // ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_BLUE));
        //1000 ms delay
        vTaskDelay((10) / portTICK_PERIOD_MS);

    } else if (soil == 0){ //Green

        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_RED2, duty));
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_GREEN2, 4096));
        // ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_BLUE, duty));
        // Update duty to apply the new value
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_RED2));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_GREEN2));
        // ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_BLUE));
        //1000 ms delay
        vTaskDelay((10) / portTICK_PERIOD_MS);
        //printf("%d\n", duty);
    } else {
        return;
    }

//RGB LED 2 for soil sensor is done

//RGB LED 3 for Temperature and humidity sensor begins

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel_red3 = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_RED3,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO_3R,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_red3));

    ledc_channel_config_t ledc_channel_green3 = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_GREEN3,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO_3G,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_green3));

    ESP_LOGI(tag,"Temperature and humidity (outside): %d\n", (*TEMP_OUT_POINTER || *HUM_OUT_POINTER));

    int n = (*TEMP_OUT_POINTER || *HUM_OUT_POINTER);

    if (n >= 1){ //Red
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_RED3, 4096));
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_GREEN3, duty));
        // ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_BLUE, duty));
        // Update duty to apply the new value
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_RED3));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_GREEN3));
        // ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_BLUE));
        //1000 ms delay
        vTaskDelay((10) / portTICK_PERIOD_MS);

    } else if (n == 0){ //Green

        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_RED3, duty));
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_GREEN3, 4096));
        // ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_BLUE, duty));
        // Update duty to apply the new value
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_RED3));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_GREEN3));
        // ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_BLUE));
        //1000 ms delay
        vTaskDelay((10) / portTICK_PERIOD_MS);
        //printf("%d\n", duty);
    } else {
        return;
    }

//RGB LED 3 for Temperature and humidity sensor is done

}

void buzz_start(int delay){
	ESP_ERROR_CHECK(ledc_set_duty(BUZZ_MODE, BUZZ_CHANNEL, 4096)); //50% duty
	// Update duty to apply the new value
	ESP_ERROR_CHECK(ledc_update_duty(BUZZ_MODE, BUZZ_CHANNEL));
	ESP_LOGI(tag, "Started buzzer.");
	vTaskDelay((10) / portTICK_PERIOD_MS);
}



void buzz_stop(int delay){
	ESP_ERROR_CHECK(ledc_set_duty(BUZZ_MODE, BUZZ_CHANNEL, 0)); //0% duty
	ESP_ERROR_CHECK(ledc_update_duty(BUZZ_MODE, BUZZ_CHANNEL)); //updated channel, so it plays.
	ESP_LOGI(tag, "Stopped buzzer.");
	vTaskDelay((delay) / portTICK_PERIOD_MS);
}

void sound(int freq, int delay){
	ESP_ERROR_CHECK(ledc_set_freq(BUZZ_MODE, BUZZ_TIMER, freq)); //50% duty
	ESP_LOGI(tag, "Playing %d Hz.", freq);
	vTaskDelay((delay) / portTICK_PERIOD_MS);
}


void buzzer_demo(){
	// Prepare and then apply the LEDC PWM timer configuration (we use it for the buzzer)
	ledc_timer_config_t ledc_timer_buzz = {
		.speed_mode       = BUZZ_MODE,
		.duty_resolution  = BUZZ_DUTY_RES,
		.timer_num        = BUZZ_TIMER,
		.freq_hz          = BUZZ_FREQUENCY,  // Set output frequency at 1 kHz
		.clk_cfg          = LEDC_AUTO_CLK
	};
	ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer_buzz));

	// Prepare and then apply the LEDC PWM channel configuration
	ledc_channel_config_t ledc_channel_buzz = {
		.speed_mode     = BUZZ_MODE,
		.channel        = BUZZ_CHANNEL,
		.timer_sel      = BUZZ_TIMER,
		.intr_type      = LEDC_INTR_DISABLE,
		.gpio_num       = BUZZ_OUTPUT_IO,
		.duty           = 0, // Set duty to 0%
		.hpoint         = 0
	};
	ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_buzz));

	// Now the initialization is done
	ESP_LOGI(tag, "Initialization complete.");

	if(warnings >= 1){//ADDED
		ESP_LOGI(tag, "Buzzer goes off, because there's too much light.");
		buzz_start(10);
		sound(1000, 800);
		sound(800, 800);
		sound(600, 800);
		sound(400, 800);
		buzz_stop(10);

	} else {//ADDED
		ESP_LOGI(tag, "Everything is fine, plant is happy, plays a happy tone.");
		buzz_start(10);
		sound(400, 800);
		sound(600, 800);
		sound(800, 800);
		sound(1000, 800);
		buzz_stop(10);

	}//this buzzer goes off to say that the plant is happy.
}

void light_adc_demo(int *LIGHT_POINTER){
    //Configuring the ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_0); //ADC1_CHANNEL_0 is on GPIO0 (GPIOzero)

    int val = adc1_get_raw(ADC1_CHANNEL_0);
    ESP_LOGI(tag, "Light sensor ADC value: %d", val);
    vTaskDelay(pdMS_TO_TICKS(500));  // Delay for 1 second

    int *TOO_MUCH_LIGHT = (int*)LIGHT_POINTER;//ADDED: Light pointer.
    //ADDED
    if(adc1_get_raw(ADC1_CHANNEL_0) <= (800)){
        ESP_LOGI(tag, "Warning: Too much light. Light sensor ADC value: %d. Ideal: 0-800", val);
        vTaskDelay(pdMS_TO_TICKS(500));  // Delay for 1 second
        *TOO_MUCH_LIGHT = 1;
        warnings++;

    } else if (adc1_get_raw(ADC1_CHANNEL_0) >= (801)){//301 instead, because we've decided that anything above 300 is fine.
        //turns off the alarms for TOO_MUCH_LIGHT, once there isn't too much light anymore.
        ESP_LOGI(tag, "Good lightning!");
        *TOO_MUCH_LIGHT = 0;
    }

    sensor_data.light = adc1_get_raw(ADC1_CHANNEL_0);

}


#define ESP_INTR_FLAG_DEFAULT 0

static QueueHandle_t gpio_evt_queue = NULL;

static volatile bool restart_loop = 0;

static void IRAM_ATTR gpio_isr_handler(void* arg) {
    //ESP_EARLY_LOGI("ISR", "Button Pressed", (uint32_t)arg);
    restart_loop = 1;  // Signal the main loop to restart
}

static void gpio_task_example(void* arg) {
    uint32_t io_num;
    for (;;) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            ESP_LOGI("BUTTON", "Restarting main loop");
            restart_loop = 1;  // Set the restart flag
        }
    }
}

void init_spiffs() {
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",    
        .partition_label = NULL,  
        .max_files = 8,            
        .format_if_mount_failed = true 
    };

    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE("SPIFFS", "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE("SPIFFS", "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE("SPIFFS", "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return;
    }

    size_t total = 0, used = 0; //to show how much space is used 
    ret = esp_spiffs_info(NULL, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE("SPIFFS", "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    } else {
        ESP_LOGI("SPIFFS", "Partition size: total: %d, used: %d", total, used);
    }
}

void write_to_file(const char *path, const char *data) {
    FILE *file = fopen(path, "w");  // Open file for writing
    if (file == NULL) {
        ESP_LOGE("SPIFFS", "Failed to open file for writing");
        return;
    }
    fprintf(file, "%s", data);  // Write data to the file
    fclose(file);  // Close the file
    ESP_LOGI("SPIFFS", "File written successfully");
}

void read_from_file(const char *path) {
    FILE *file = fopen(path, "r");  // Open file for reading
    if (file == NULL) {
        ESP_LOGE("SPIFFS", "Failed to open file for reading");
        return;
    }

    char buffer[64];
    while (fgets(buffer, sizeof(buffer), file) != NULL) {
        ESP_LOGI("SPIFFS", "Read from file: %s", buffer);
    }
    fclose(file);  // Close the file
}

void log_data_to_spiffs(const char *file_name, const char *data) {
    FILE *file = fopen(file_name, "a");
    if (file == NULL) {
        ESP_LOGE("SPIFFS", "Failed to open file for writing.");
        return;
    }

    fprintf(file, "%s\n", data);  // Write the data to the file
    fclose(file);
    ESP_LOGI("SPIFFS", "Data written to %s", file_name);
}


int get_current_day() {
    int day = 1;  //day counter
    if (xTaskGetTickCount() - start_tick >= TICKS_PER_DAY) {
        start_tick = xTaskGetTickCount();  // Reset the starting tick
        day = (day % 7) + 1;  
    }
    return day;
}



void write_data_to_daily_log() {
    // Format the header and data
    const char *header = "temperature_out,humidity_out,temperature_in,humidity_in,light";
    char log_entry[128];
    snprintf(log_entry, sizeof(log_entry), "%.2f,%.2f,%.2f,%.2f,%d",
             sensor_data.temperature_out,
             sensor_data.humidity_out,
             sensor_data.temperature_in,
             sensor_data.humidity_in,
             sensor_data.light);

    // Determine the current day and file path
    int day = get_current_day(); 
    char file_path[32];
    snprintf(file_path, sizeof(file_path), "/spiffs/log_day_%d.txt", day);

    // Check if the file exists
    FILE *file = fopen(file_path, "r");
    if (file == NULL) {
        // File doesn't exist; write the header
        file = fopen(file_path, "w");  // Create the file
        if (file == NULL) {
            ESP_LOGE("SPIFFS", "Failed to create file: %s", file_path);
            return;
        }
        fprintf(file, "%s\n", header);  // Write the header
        fclose(file);
    } else {
        fclose(file);  // File exists; no need to write the header
    }

    // Append the data to the file
    file = fopen(file_path, "a");
    if (file == NULL) {
        ESP_LOGE("SPIFFS", "Failed to open file %s for appending.", file_path);
        return;
    }
    fprintf(file, "%s\n", log_entry);  // Write the data
    fclose(file);
    ESP_LOGI("SPIFFS", "Appended data to %s: %s", file_path, log_entry);
}


void clear_daily_log(int day) {
    char file_path[32];
    snprintf(file_path, sizeof(file_path), "/spiffs/log_day_%d.txt", day);

    FILE *file = fopen(file_path, "w");
    if (file != NULL) {
        fclose(file);
        ESP_LOGI("SPIFFS", "Cleared log file %s.", file_path);
    } else {
        ESP_LOGE("SPIFFS", "Failed to clear log file %s.", file_path);
    }
}


void manage_daily_logs() {
    static int last_day = -1;  // -1 ensures we detect the first day properly
    int current_day = get_current_day();

    if (current_day != last_day) {
        ESP_LOGI("SPIFFS", "Day changed: clearing log for day %d.", current_day);
        clear_daily_log(current_day);  // Clear the new day's log file
        last_day = current_day;        // Update the last day to the current day
    }
}




void clear_file(const char *path) {
    FILE *file = fopen(path, "w");  // Open file in write mode (truncates existing content)
    if (file == NULL) {
        ESP_LOGE("SPIFFS", "Failed to open file %s for clearing.", path);
        return;
    }
    fclose(file);  // Close the file immediately after truncating
    ESP_LOGI("SPIFFS", "File %s cleared successfully.", path);
}

void remove_old_log() {
    const char *log_path = "/spiffs/log.txt";
    if (remove(log_path) == 0) {
        ESP_LOGI("SPIFFS", "File %s removed successfully.", log_path);
    } else {
        ESP_LOGW("SPIFFS", "File %s does not exist or could not be removed.", log_path);
    }
}

void read_current_day_spiffs_file() {
    // Get the current day
    int current_day = get_current_day();

    // Construct the file path for the current day's file
    char file_path[64];
    snprintf(file_path, sizeof(file_path), "/spiffs/log_day_%d.txt", current_day);

    // Open the file
    FILE *file = fopen(file_path, "r");
    if (!file) {
        ESP_LOGE("SPIFFS", "Failed to open file: %s", file_path);
        return;
    }

    ESP_LOGI("SPIFFS", "Reading file: %s", file_path);
    char line[64];
    while (fgets(line, sizeof(line), file)) {
        ESP_LOGI("SPIFFS", "%s", line);
    }

    fclose(file);
}




void app_main(void)
{


	//Initialize common I2C port for display, soil sensor, and temperature/umidity sensor
	//Initialized it as follows only once here in the main, then use the shared_init
	//functions for the different components as shown in this demo (see _demo functions).
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = I2C_MASTER_SDA_GPIO;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_io_num = I2C_MASTER_SCL_GPIO;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
	conf.clk_flags = 0;
	i2c_param_config(I2C_NUM, &conf);
	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0));

	//zero-initialize the config structure.
	gpio_config_t io_conf;//original setup from code
	//interrupt of rising edge
	io_conf.intr_type = GPIO_INTR_POSEDGE;
	//bit mask of the pins, using the original button 2, because that's the port we're using.
	io_conf.pin_bit_mask = (1ULL<<BUTTON_2_GPIO_PIN);
	//set as input mode
	io_conf.mode = GPIO_MODE_INPUT;
	//enable pull-up mode
	io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
	gpio_config(&io_conf);

	//enables interrupt on falling edge for the button
	gpio_set_intr_type(BUTTON_2_GPIO_PIN, GPIO_INTR_ANYEDGE);

	//create a queue to handle gpio event from isr
	gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
	//start gpio task
	xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);

	//install gpio isr service
	gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
	//hook isr handler for specific gpio pin
	gpio_isr_handler_add(BUTTON_2_GPIO_PIN, gpio_isr_handler, (void*) BUTTON_2_GPIO_PIN);

	int TOO_MUCH_LIGHT = 0, *LIGHT_POINTER = &TOO_MUCH_LIGHT;//the value of the alarm and a pointer to it

    int TEMP_OUT = 0, *TEMP_OUT_POINTER = &TEMP_OUT;

    int HUM_OUT = 0, *HUM_OUT_POINTER = &HUM_OUT;

    int HUM_IN = 0, *HUM_IN_POINTER = &HUM_IN;

    int TEMP_IN = 0, *TEMP_IN_POINTER = &TEMP_IN;

    TickType_t start_tick = xTaskGetTickCount();
    TickType_t last_wake_time = xTaskGetTickCount(); // Get the current tick count
    ESP_LOGI("SPIFFS", "Initializing SPIFFS"); //initialise the spiffs
    init_spiffs();

    while(1){

        printf("Hello! Starting now with the demos ;-)\n");

        printf("\nPrinting device information:\n");
        print_info();

        printf("\nRunning the light ADC demo (20 reads - cover/uncover the sensor):\n");
        light_adc_demo(LIGHT_POINTER);

        printf("\nRunning STEMMA soil sensor demo: (20 reads - touch the sensor to see changes)\n");
        stemma_soil_demo(HUM_IN_POINTER, TEMP_IN_POINTER);

        printf("\nRunning temperature/humidity sensor demo (20 reads - touch/blow on the sensor to see changes):\n");
        temperaure_humidity_demo(TEMP_OUT_POINTER, HUM_OUT_POINTER);
        
        printf("\nRunning display demo (look at the display!):\n");
        display_demo(LIGHT_POINTER, TEMP_OUT_POINTER, HUM_OUT_POINTER, HUM_IN_POINTER, TEMP_IN_POINTER);

        printf("\nRunning RGB LED demo (look at the LED!):\n");
        led_fade_demo(LIGHT_POINTER, TEMP_OUT_POINTER, HUM_OUT_POINTER, HUM_IN_POINTER, TEMP_IN_POINTER);
        ESP_LOGI(tag, "\nWarnings: %d\n", warnings);

        printf("Running the buzzer demo:\n");
        buzzer_demo();

        warnings = 0;

        // Check if the day has changed and clear the file for the new day
        manage_daily_logs();
        // Write sensor data to the current day's log
        write_data_to_daily_log();

        // uncomment this to see contents of spiffs file
        read_current_day_spiffs_file();

        printf("\nThe demos are finished. Prees the reset button if you want to restart or wait for the timer.\n");
        fflush(stdout);

        const TickType_t wait_ticks = pdMS_TO_TICKS(1000); // Check every 1 second
        TickType_t remaining_delay = pdMS_TO_TICKS(60000);

        while (remaining_delay > 0) {
            if (restart_loop) {
                restart_loop = 0; // Clear the flag
                break;            // Exit the waiting loop
            }
            vTaskDelay(wait_ticks);    // Delay in small increments
            remaining_delay -= wait_ticks; // Reduce remaining delay
        }
    }
    //esp_restart();
}
