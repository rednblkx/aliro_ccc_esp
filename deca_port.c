#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <rom/ets_sys.h>

#include "deca_version.h"
#ifdef DW3000_DRIVER_VERSION // == 0x040000
#include "deca_device_api.h"
#else
#include "deca_interface.h"
#endif

static portMUX_TYPE dw3000_mutex = portMUX_INITIALIZER_UNLOCKED;

decaIrqStatus_t decamutexon(void)
{
	portENTER_CRITICAL(&dw3000_mutex);
	return 0;
}

void decamutexoff(decaIrqStatus_t s)
{
	portEXIT_CRITICAL(&dw3000_mutex);
}

void deca_sleep(unsigned int time_ms)
{
	vTaskDelay(pdMS_TO_TICKS(time_ms));
}

void deca_usleep(unsigned long time_us)
{
	ets_delay_us(time_us);
}