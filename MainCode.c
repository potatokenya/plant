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
#define LEDC_OUTPUT_IO_1R       (8) // Define the output GPIO for red
#define LEDC_OUTPUT_IO_1G       (9) // Define the output GPIO for green

#define LEDC_CHANNEL_RED        LEDC_CHANNEL_0
#define LEDC_CHANNEL_GREEN      LEDC_CHANNEL_1
#define LEDC_CHANNEL_BLUE       LEDC_CHANNEL_2
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (4096) // Set duty to 50%. (2 ** 13) * 50% = 4096
#define LEDC_FREQUENCY          (1000) // Frequency in Hertz. Set frequency at 1 kHz

#define BUZZ_TIMER              LEDC_TIMER_1
#define BUZZ_MODE               LEDC_LOW_SPEED_MODE
#define BUZZ_OUTPUT_IO          (9) // Define the output GPIO for red
#define BUZZ_CHANNEL            LEDC_CHANNEL_4
#define BUZZ_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define BUZZ_DUTY               (4096) // Set duty to 50%. (2 ** 13) * 50% = 4096
#define BUZZ_FREQUENCY          (1000) // Frequency in Hertz. Set frequency at 1 kHz

int warnings = 0; //Global variable for warnings

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

    printf("%uMB %s flash\n", flash_size / (1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());
}

void display_demo(int *LIGHT_POINTER){
    SSD1306_t dev;
    //int center, top; //, bottom;
    //char lineChar[20];

    //Initialize the display (shared i2c) only once after boot.
    i2c_master_shared_i2c_init(&dev);

    //Uncomment this if you want to flip the display
    //dev._flip = true;

    ssd1306_init(&dev, 128, 64);

	int *TOO_MUCH_LIGHT = (int*)LIGHT_POINTER;//ADDED: Light pointer.

	if(*TOO_MUCH_LIGHT){//ADDED: display warning about light pointer.

		ESP_LOGI(tag, "Displaying warning about light.");
		ssd1306_clear_screen(&dev, false);
		ssd1306_clear_screen(&dev, false);
		ssd1306_contrast(&dev, 0xff);

		ssd1306_display_text(&dev, 3, "WARNING", 7, false); //ADDED: changed it to be in the center
		ssd1306_hardware_scroll(&dev, SCROLL_RIGHT);

		vTaskDelay(5000 / portTICK_PERIOD_MS);

		ssd1306_hardware_scroll(&dev, SCROLL_STOP);

		ssd1306_clear_screen(&dev, false);
		ssd1306_clear_screen(&dev, false);

		ssd1306_display_text(&dev, 3, "     WARNING     ", 17, false);
		ssd1306_display_text(&dev, 4, " Too much light! ", 17, false);

		vTaskDelay(10000 / portTICK_PERIOD_MS);

		ssd1306_clear_screen(&dev, false);
		ssd1306_clear_screen(&dev, false);
		ssd1306_display_text_x3(&dev, 3, "(T-T)", 5, false);
		ssd1306_hardware_scroll(&dev, SCROLL_RIGHT);

		vTaskDelay(10000 / portTICK_PERIOD_MS);
		ssd1306_hardware_scroll(&dev, SCROLL_STOP);
		//delay 10 seconds

		ssd1306_clear_screen(&dev, false);
		ssd1306_clear_screen(&dev, false);
		ssd1306_display_text_x3(&dev, 3, "(T-T)", 5, false);
		vTaskDelay(2000 / portTICK_PERIOD_MS);

	} else {
		ESP_LOGI(tag, "Plant is happy, displaying emoji.");
		ssd1306_clear_screen(&dev, false);
		ssd1306_clear_screen(&dev, false);
		ssd1306_contrast(&dev, 0xff);
		ssd1306_display_text_x3(&dev, 3, "(^_^)", 5, false);
		ssd1306_hardware_scroll(&dev, SCROLL_RIGHT);

		vTaskDelay(10000 / portTICK_PERIOD_MS);
		ssd1306_hardware_scroll(&dev, SCROLL_STOP);
		//delay 10 seconds

		ssd1306_clear_screen(&dev, false);
		ssd1306_clear_screen(&dev, false);
		ssd1306_display_text_x3(&dev, 3, "(^_^)", 5, false);
		vTaskDelay(2000 / portTICK_PERIOD_MS);
	}

}

void temperaure_humidity_demo(){
    i2c_dev_t dev = {0};

    //Initialize the sensor (shared i2c) only once after boot.
    ESP_ERROR_CHECK(am2320_shared_i2c_init(&dev, I2C_NUM));

    float temperature, humidity;

    for (int i = 0; i < 20; i++)
    {
        esp_err_t res = am2320_get_rht(&dev, &temperature, &humidity);
        if (res == ESP_OK)
            ESP_LOGI(tag, "Temperature: %.1f°C, Humidity: %.1f%%", temperature, humidity);
        else
            ESP_LOGE(tag, "Error reading data: %d (%s)", res, esp_err_to_name(res));

        //500 ms delay
        vTaskDelay((500) / portTICK_PERIOD_MS);
    }
}

void stemma_soil_demo(){
    int ret = ESP_OK;
    uint16_t moisture_value = 0;
    float temperature_value = 0;

    //Initialize the sensor (shared i2c) only once after boot.
    ESP_ERROR_CHECK(adafruit_stemma_soil_sensor_shared_i2c_init());

    for (int i = 0; i < 10; i++)
    {
        ret = adafruit_stemma_soil_sensor_read_moisture(I2C_NUM, &moisture_value);

        if (ret == ESP_OK)
        {
            ESP_LOGI(tag, "Adafruit Stemma sensor value: =%u", moisture_value);
        }

        ret = adafruit_stemma_soil_sensor_read_temperature(I2C_NUM, &temperature_value);

        if (ret == ESP_OK)
        {
            ESP_LOGI(tag, "Adafruit Stemma sensor value: =%f", temperature_value);
        }
        
        //500 ms delay
        vTaskDelay((500) / portTICK_PERIOD_MS);
    }
}

void led_fade_demo(){
    // Prepare and then apply the LEDC PWM timer configuration (one time can drive multiple channels)
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 1 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel_red = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_RED,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO_RED,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_red));
//Here comes the RGB LED 1 for the light sensor
   // Prepare and then apply the LEDC PWM channel configuration
	ledc_channel_config_t ledc_channel_red = {
		.speed_mode     = LEDC_MODE,
		.channel        = LEDC_CHANNEL_RED,
		.timer_sel      = LEDC_TIMER,
		.intr_type      = LEDC_INTR_DISABLE,
		.gpio_num       = LEDC_OUTPUT_IO_1G,
		.duty           = 0, // Set duty to 0%
		.hpoint         = 0
	};
	ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_red));

	ledc_channel_config_t ledc_channel_green = {
		.speed_mode     = LEDC_MODE,
		.channel        = LEDC_CHANNEL_GREEN,
		.timer_sel      = LEDC_TIMER,
		.intr_type      = LEDC_INTR_DISABLE,
		.gpio_num       = LEDC_OUTPUT_IO_1G,
		.duty           = 0, // Set duty to 0%
		.hpoint         = 0
	};
	ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_green));


    // Now the initialization is done
    ESP_LOGI(tag, "Initialization complete. Displaying either green or red.");

	ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_RED, 0));
	ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_GREEN, 4096));
	//ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_BLUE, 0));
	// Update duty to apply the new value
	ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_RED));
	ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_GREEN));
	//ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_BLUE));
	//1000 ms delay
	vTaskDelay((10) / portTICK_PERIOD_MS);

	int *TOO_MUCH_LIGHT = (int*)LIGHT_POINTER;//ADDED: Light pointer.

	if(*TOO_MUCH_LIGHT){//ADDED: RGB LED turns red as a warning
		// Set duty
		ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_RED, 4096));
		ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_GREEN, 0));
		//ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_BLUE, 0));
		// Update duty to apply the new value
		ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_RED));
		ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_GREEN));
		//ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_BLUE));
		//1000 ms delay
		vTaskDelay((10) / portTICK_PERIOD_MS);
		//printf("%d\n", duty);//ADDED: insert code for red RGB LED, that should stay on until all warnings are taken care of and the code refreshes.
	}
//RGB LED 1 for light sensor is done

//RGB LED 2 for soil sensor begins

//RGB LED 2 for soil sensor is done

//RGB LED 3 for Temperature and humidity sensor begins

//RGB LED 3 for Temperature and humidity sensor is done

}

void buzzer_demo(int *LIGHT_POINTER){
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
    ESP_LOGI(tag, "Initialization complete. Playing 3 tones.");

    int *TOO_MUCH_LIGHT = (int*)LIGHT_POINTER;//ADDED pointer to photocell.

	if(*TOO_MUCH_LIGHT){//ADDED
		ESP_LOGI(tag, "Buzzer goes off, because there's too much light.");

		// Set duty
			ESP_ERROR_CHECK(ledc_set_duty(BUZZ_MODE, BUZZ_CHANNEL, 4096)); //50% duty
		// Update duty to apply the new value
			ESP_ERROR_CHECK(ledc_update_duty(BUZZ_MODE, BUZZ_CHANNEL));
		//1000 ms delay
			ESP_ERROR_CHECK(ledc_set_freq(BUZZ_MODE, BUZZ_TIMER, 1000)); //50% duty
			ESP_LOGI(tag, "Playing 1000 Hz.");
			vTaskDelay((600) / portTICK_PERIOD_MS);

		// update duty to 800 hz
			ESP_ERROR_CHECK(ledc_set_freq(BUZZ_MODE, BUZZ_TIMER, 800)); //50% duty
			ESP_LOGI(tag, "Playing 800 Hz.");
			vTaskDelay((600) / portTICK_PERIOD_MS);

		// update duty to 600 hz
			ESP_ERROR_CHECK(ledc_set_freq(BUZZ_MODE, BUZZ_TIMER, 400)); //50% duty
			ESP_LOGI(tag, "Playing 600 Hz.");
			vTaskDelay((600) / portTICK_PERIOD_MS);

		// update duty to 600 hz
			ESP_ERROR_CHECK(ledc_set_freq(BUZZ_MODE, BUZZ_TIMER, 200)); //50% duty
			ESP_LOGI(tag, "Playing 400 Hz.");// is it really 400?
			vTaskDelay((600) / portTICK_PERIOD_MS);

		// Turning off buzzer
			ESP_ERROR_CHECK(ledc_set_duty(BUZZ_MODE, BUZZ_CHANNEL, 0)); //0% duty//the buzzer goes off for a longer time.
			ESP_ERROR_CHECK(ledc_update_duty(BUZZ_MODE, BUZZ_CHANNEL)); //updated channel, so it plays.

		vTaskDelay((10) / portTICK_PERIOD_MS);

	} else {//ADDED
		ESP_LOGI(tag, "Everything is fine, plant is happy, plays a happy tone.");

		// Set duty
		ESP_ERROR_CHECK(ledc_set_duty(BUZZ_MODE, BUZZ_CHANNEL, 4096)); //50% duty
		// Update duty to apply the new value
		ESP_ERROR_CHECK(ledc_update_duty(BUZZ_MODE, BUZZ_CHANNEL));
		//1000 ms delay

		ESP_ERROR_CHECK(ledc_set_freq(BUZZ_MODE, BUZZ_TIMER, 200)); //50% duty
		ESP_LOGI(tag, "Playing 200 Hz.");
		vTaskDelay((600) / portTICK_PERIOD_MS);

		// update duty to 800 hz
		ESP_ERROR_CHECK(ledc_set_freq(BUZZ_MODE, BUZZ_TIMER, 400)); //50% duty
		ESP_LOGI(tag, "Playing 600 Hz.");
		vTaskDelay((600) / portTICK_PERIOD_MS);

		// update duty to 600 hz
		ESP_ERROR_CHECK(ledc_set_freq(BUZZ_MODE, BUZZ_TIMER, 800)); //50% duty
		ESP_LOGI(tag, "Playing 800 Hz.");
		vTaskDelay((600) / portTICK_PERIOD_MS);

		// update duty to 400 hz
		ESP_ERROR_CHECK(ledc_set_freq(BUZZ_MODE, BUZZ_TIMER, 1000)); //50% duty
		ESP_LOGI(tag, "Playing 1000 Hz.");// is it really 400?
		vTaskDelay((600) / portTICK_PERIOD_MS);

		// Turning off buzzer
		ESP_ERROR_CHECK(ledc_set_duty(BUZZ_MODE, BUZZ_CHANNEL, 0)); //0% duty//the buzzer goes off for a longer time.
		ESP_ERROR_CHECK(ledc_update_duty(BUZZ_MODE, BUZZ_CHANNEL)); //updated channel, so it plays.

		vTaskDelay((10) / portTICK_PERIOD_MS);

	}//this buzzer goes off to say that the plant is happy.
}

void light_adc_demo(int *LIGHT_POINTER){
    //Configuring the ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11); //ADC1_CHANNEL_0 is on GPIO0 (GPIOzero)

    for (int i = 0; i < 5; i++)
    {
        int val = adc1_get_raw(ADC1_CHANNEL_0);
        ESP_LOGI(tag, "Light sensor ADC value: %d", val);
        vTaskDelay(pdMS_TO_TICKS(500));  // Delay for 1 second

int *TOO_MUCH_LIGHT = (int*)LIGHT_POINTER;//ADDED: Light pointer.

			//ADDED
			if(adc1_get_raw(ADC1_CHANNEL_0) <= (300)){
				ESP_LOGI(tag, "Warning: Too much light. Light sensor ADC value: %d", val);
				vTaskDelay(pdMS_TO_TICKS(500));  // Delay for 1 second
				*TOO_MUCH_LIGHT = 1;

			} else if (adc1_get_raw(ADC1_CHANNEL_0) >= (301)){//301 instead, because we've decided that anything above 300 is fine.
				//turns off the alarms for TOO_MUCH_LIGHT, once there isn't too much light anymore.
				*TOO_MUCH_LIGHT = 0;
			}

    }
}


#define ESP_INTR_FLAG_DEFAULT 0

static QueueHandle_t gpio_evt_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
	uint32_t gpio_num = (uint32_t) arg;
	xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void gpio_task_example(void* arg)
{
	uint32_t io_num;
	for (;;) {
		if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
			esp_restart();
		}
	}
}
void app_main(void)
{
    printf("Hello! Starting now with the demos ;-)\n");

    printf("\nPrinting device information:\n");
    print_info();

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

	int TOO_MUCH_LIGHT = 0, *LIGHT_POINTER = &TOO_MUCH_LIGHT;//ADDED: the value of the alarm and a pointer to it

    printf("\nRunning the light ADC demo (20 reads - cover/uncover the sensor):\n");
    light_adc_demo(LIGHT_POINTER);

    printf("\nRunning the buzzer demo:\n");
    buzzer_demo(LIGHT_POINTER);

    printf("\nRunning RGB LED demo (look at the LED!):\n");
    led_fade_demo(LIGHT_POINTER);

    printf("\nRunning display demo (look at the display!):\n");
    display_demo(LIGHT_POINTER);

    printf("\nRunning temperature/humidity sensor demo (20 reads - touch/blow on the sensor to see changes):\n");
    temperaure_humidity_demo();

    printf("\nRunning STEMMA soil sensor demo: (20 reads - touch the sensor to see changes)\n");
    stemma_soil_demo();

    printf("\nThe demos are finished. Prees the reset button if you want to restart.\n");
    printf("Use the code in the demo in your own software. Goodbye!\n");
    fflush(stdout);

//This would automatically restart the ESP32
	vTaskDelay(pdMS_TO_TICKS(1800000));//ADDED: This is a one hour delay = 500/*that's one seconde*/ * 60 * 60 = 1800000. EDIT: I set it to 2 min.

    //esp_restart();
}
