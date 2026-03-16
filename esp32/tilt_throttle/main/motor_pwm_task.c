#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"
#include "esp_log.h"


#define MCPWM_TIMER_RESOLUTION_HZ 1000000 // 1MHz, 1 tick = 1us
#define MCPWM_PERIOD              1000    // 1000us, 1KHz

#define PWM_GPIO_PIN 0
#define DIRECTION_GPIO_PIN_1 22
#define DIRECTION_GPIO_PIN_2 23

#define MAX_DUTY_CYCLE_percent 40

static const char *TAG = "motor_pwm_task";

static mcpwm_cmpr_handle_t comparator = NULL;
static int activeDuty = 0;


int dutyToComparatorThresh(int duty) {
    if(duty < 0) {
        return 0;
    }
    return ((MCPWM_PERIOD * ((duty<=MAX_DUTY_CYCLE_percent) ? duty : MAX_DUTY_CYCLE_percent)) / 100); 
}

void blockingDriveMotor(int duty) {
    // truncate to safe values
    duty = ((duty<(-1*MAX_DUTY_CYCLE_percent)) ? (-1*MAX_DUTY_CYCLE_percent) : duty);
    duty = ((duty>MAX_DUTY_CYCLE_percent) ? MAX_DUTY_CYCLE_percent : duty);

    if(duty == activeDuty) {
        // do nothing
        return;
    }

    if(duty*activeDuty < 0) {
        // deadtime to prevent short circtuits on active direction changes
        mcpwm_comparator_set_compare_value(comparator, dutyToComparatorThresh(0));
        activeDuty = 0;
        gpio_set_level(DIRECTION_GPIO_PIN_1, 0);
        gpio_set_level(DIRECTION_GPIO_PIN_2, 0);
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }

    if(activeDuty==0) {
        // set a direction
        if(duty > 0) {
            gpio_set_level(DIRECTION_GPIO_PIN_1, 0);
            gpio_set_level(DIRECTION_GPIO_PIN_2, 1);
        }
        else {
            gpio_set_level(DIRECTION_GPIO_PIN_1, 1);
            gpio_set_level(DIRECTION_GPIO_PIN_2, 0);
        }
    }

    mcpwm_comparator_set_compare_value(comparator, dutyToComparatorThresh(abs(duty)));
    activeDuty = duty;
}

void motor_pwm_task(void *arg)
{
    // configure GPIO pins for direction control
    gpio_config_t direction_gpio_1_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << DIRECTION_GPIO_PIN_1,
    };
    gpio_config_t direction_gpio_2_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << DIRECTION_GPIO_PIN_2,
    };
    ESP_ERROR_CHECK(gpio_config(&direction_gpio_1_config));
    ESP_ERROR_CHECK(gpio_config(&direction_gpio_2_config));
    gpio_set_level(DIRECTION_GPIO_PIN_1, 0);
    gpio_set_level(DIRECTION_GPIO_PIN_2, 0);

    ESP_LOGI(TAG, "Create timer and operator");
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = MCPWM_TIMER_RESOLUTION_HZ,
        .period_ticks = MCPWM_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t operator_config = {
        .group_id = 0, // operator must be in the same group to the timer
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));

    ESP_LOGI(TAG, "Connect timer and operator");
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    ESP_LOGI(TAG, "Create comparator and generator from the operator");
    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator));

    mcpwm_gen_handle_t generator = NULL;
    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = PWM_GPIO_PIN,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &generator));

    // set the initial compare value, output duty cycle = 0 initially
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, 0));

    ESP_LOGI(TAG, "Set generator action on timer and compare event");
    // go high on counter empty
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator,
                                                              MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    // go low on compare threshold
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator,
                                                                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)));


    ESP_LOGI(TAG, "Enable and start timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));


    // argument to this task is the queue to receive motor commands from imu_task on CORE0
    QueueHandle_t queue = (QueueHandle_t)arg;

    while(1) {
        int newDuty = 0;

        // blocks until queue contains data
        if(xQueueReceive(queue, (void *)&newDuty, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG, "received motor command = %d", newDuty);

            // blocks as needed to keep timings safe for motor on instant direction changes
            blockingDriveMotor(newDuty);  
        }      
    }

    vTaskDelete(NULL);
}