#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_log.h"

#define CORE0 0
#define CORE1 ((CONFIG_FREERTOS_NUMBER_OF_CORES > 1) ? 1 : tskNO_AFFINITY)
#define TASK_PRIO_2 (2)

#define TAG "tilt_throttle_main"

extern void imu_task(void *arg);
extern void motor_pwm_task(void *arg);

void app_main(void)
{
    printf("#### Tilt Throttle ####\n");

    /* Print chip information */
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), %s%s%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
           (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
           (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    printf("silicon revision v%d.%d, ", major_rev, minor_rev);
    if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        printf("Get flash size failed");
        return;
    }

    printf("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());

    // create queue to send motor commands to motor_pwm_task
    QueueHandle_t queue = xQueueGenericCreate(5, sizeof(int), queueQUEUE_TYPE_SET);
    if (queue == NULL) {
        ESP_LOGE(TAG, "Failed to create queue (to send motor commands from imu_task)");
    }

    // Create IMU input task on CPU0 (this core)
    xTaskCreatePinnedToCore(imu_task, "imu_task", 4096, queue, 5, NULL, CORE0);

    // Create Motor PWM output task on CPU1
    xTaskCreatePinnedToCore(motor_pwm_task, "motor_pwm_task", 4096, queue, TASK_PRIO_2, NULL, CORE1);
}
