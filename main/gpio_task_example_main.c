// Implemented successfully on May 5th 2024
// Reference docs: https://docs.espressif.com/projects/esp-idf/en/stable/esp32c3/hw-reference/esp32c3/user-guide-devkitc-02.html

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

/**
 * Brief:
 * This test code shows how to configure gpio and how to use gpio interrupt.
 *
 * GPIO0: input, pulled up, interrupt disabled, onboard button
 * GPIO2: output, interrupt disabled, onbuard ledterrupt on GPIO4/5
 *
 */

#define GPIO_OUT_0     2 // Indication of the board's pin we want to use
#define GPIO_IN_0      0 // Same as previous line.
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUT_0))
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_IN_0))
#define ESP_INTR_FLAG_DEFAULT 0
#define ON 1
#define OFF 0
#define LED_CHANGE_DELAY_MS    500

static void led_toggle_task(void* arg)
{
    static uint8_t led_state = OFF; // Variable declaration for the LED status.

    while(true) {
        if (led_state == OFF) {
            led_state = ON;
            gpio_set_level(GPIO_OUT_0, ON);
        }
        else {
                led_state = OFF;
                gpio_set_level(GPIO_OUT_0, OFF);
        }

        vTaskDelay(LED_CHANGE_DELAY_MS / portTICK_PERIOD_MS); // Wait 250 millisencond between executions
        printf("Toggle LED\n");
    }
}

static void one_shot_task(void* arg) // "static" provides visibility of the funtion only within the curren module.
{
    printf("One shot task excecuted and deleted\n");

    vTaskDelete(NULL); // The taks auto eliminates.
}


static void counter_task(void* arg)
{
    int cnt = 0;
    while(true) {
        printf("DAIoT Counter_Task - Counts: %d\n", cnt++);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    // GPIO OUTPUTS CONFIG
    //zero-initialize the config structure. Structure declaration.
    gpio_config_t out_conf = {};
    //disable interrupt. An interruption is a call that has a priority attention of the microcontroller.
    out_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode.
    out_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    out_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL; // This variable is defined upper in the code.
    //disable pull-down mode. Since we are configuring outputs, there's no need to configure either pull up or down. Still a declaration has to be made.
    out_conf.pull_down_en = 0;
    //disable pull-up mode
    out_conf.pull_up_en = 0;
    //configure GPIO with the given settings. Port configuration is configured passig by parameter the file config.
    gpio_config(&out_conf);

    // GPIO INPUTS CONFIG
    //zero-initialize the config structure. We declare a new port configuration structure using the data type "gpio_config_t"
    gpio_config_t in_conf = {};
    //disable interrupt
    in_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    in_conf.mode = GPIO_MODE_INPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    in_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //disable pull-down mode
    in_conf.pull_down_en = 0;
    //enable pull-up mode
    in_conf.pull_up_en = 1;
    //configure GPIO with the given settings
    gpio_config(&in_conf); // THe ampersand provides the indication that the parameter sent is a pointer to the memory address.

    // 5 seg delay
    printf("Waiting 5 sec\n");
    vTaskDelay(5000 / portTICK_PERIOD_MS);

    //start one shot task
    xTaskCreate(one_shot_task, "one_shot_task", 2048, NULL, 10, NULL); // One time execution task. Each time is called ths OS loads it.
    //Las parameter: task handler where we can pass a task identifier.

    // 5 seg delay
    printf("Waiting 5 sec\n");
    vTaskDelay(5000 / portTICK_PERIOD_MS);

    //start toggle led task (CONTINUOUS / INFINITE)
    xTaskCreate(led_toggle_task, "led_toggle_task", 2048, NULL, 10, NULL);

    //start counter task (IT RUNS WHILE IT IS NOT DELETED)
    TaskHandle_t counter_task_handle = NULL;
    xTaskCreate(counter_task, "counter_task", 2048, NULL, 10, &counter_task_handle); // At the last parameter we load the taks identifier that we'll nee to delete the task.

    printf("Minimum free heap size: %ld bytes\n", esp_get_minimum_free_heap_size()); // To show the free memory available of the free RTOS. 

    // After all tasks have been completed, the program reaches the followin infinite loop.
    while(1) {
        vTaskDelay(100 / portTICK_PERIOD_MS);
        // idle
        if (gpio_get_level(GPIO_IN_0) == 0) {

            printf("Button pressed\n");

            if (counter_task_handle != NULL) {
                vTaskDelete(counter_task_handle); // If the handler still exists, then delete it.
                counter_task_handle = NULL;
                printf("counter_task deleted\n");
            }

        }        
    }
}
