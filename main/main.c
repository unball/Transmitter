#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

// Callback do timer One-Shot
void callbackOneShot(TimerHandle_t xTimer)
{
    printf("One-shot: executou uma vez!\n");
}

// Callback do timer Auto-Reload
void callbackAutoReload(TimerHandle_t xTimer)
{
    printf("Auto-reload: executando periodicamente!\n");
}

void app_main(void)
{
    // Timer de 3 segundos, sem repetição
    TimerHandle_t timerOneShot = xTimerCreate(
        "OneShot",
        pdMS_TO_TICKS(3000),
        pdFALSE,
        NULL,
        callbackOneShot
    );

    // Timer de 1 segundo, com repetição automática
    TimerHandle_t timerAutoReload = xTimerCreate(
        "AutoReload",
        pdMS_TO_TICKS(1000),
        pdTRUE,
        NULL,
        callbackAutoReload
    );

    if (timerOneShot == NULL || timerAutoReload == NULL) {
        printf("Erro ao criar timers!\n");

        if (timerOneShot != NULL)
            xTimerDelete(timerOneShot, 0);

        if (timerAutoReload != NULL)
            xTimerDelete(timerAutoReload, 0);

        return;
    }

    // Inicia os dois timers
    xTimerStart(timerOneShot, portMAX_DELAY);
    xTimerStart(timerAutoReload, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(5000));
    xTimerReset(timerOneShot,0);
}
