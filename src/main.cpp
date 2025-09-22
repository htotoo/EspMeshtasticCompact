#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"

extern "C" void app_main(void) {
    printf("Hello, world!\n");
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}