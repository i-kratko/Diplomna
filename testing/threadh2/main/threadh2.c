#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <esp_log.h>
#include <openthread/instance.h>
#include <openthread/thread.h>

static const char *TAG = "THREAD_BR";

static void thread_task(void *pvParameter) {
    while (1) {
        otTaskletsProcess(otInstanceGet());
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void app_main() {
    // Initialize OpenThread
    otInstance *ot = otInstanceInitSingle();
    assert(ot != NULL);

    // Start OpenThread task
    otIp6SetEnabled(ot, true);

    xTaskCreate(&thread_task, "thread_task", 4096, NULL, 5, NULL);
}
