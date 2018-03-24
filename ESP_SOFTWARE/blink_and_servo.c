#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_attr.h"	
#include "freertos/queue.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

#define SERVO_MIN_PULSEWIDTH 1000
#define SERVO_MAX_PULSEWIDTH 2000
#define SERVO_MAX_DEGREE 90

#define BLINK_GPIO GPIO_NUM_27
#define SERVO_GPIO 18
#define LED_LIM 60

QueueHandle_t q = NULL;

void blink_task(void *pvParameter) {
	gpio_pad_select_gpio(BLINK_GPIO);
	gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

	unsigned long recieve = 0;

	while (1) {
		if (q == NULL) {
			//gpio_set_level(BLINK_GPIO, 0);
			printf("Empty Queue!");
		} else {
			xQueueReceive(q, &recieve,
					(TickType_t) (1000 / portTICK_PERIOD_MS));
			gpio_set_level(BLINK_GPIO, recieve);
			printf("value received on queue: %lu \n", recieve);
		}
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
}

static void mcpwm_example_gpio_initialize() {
	printf("initializing mcpwm servo control gpio......\n");
	mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, SERVO_GPIO);
}

static uint32_t servo_per_degree_init(uint32_t degree_of_rotation) {
	uint32_t cal_pulsewidth = 0;
	cal_pulsewidth = (SERVO_MIN_PULSEWIDTH
			+ (((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH)
					* (degree_of_rotation)) / (SERVO_MAX_DEGREE)));
	return cal_pulsewidth;
}

void mcpwm_example_servo_control(void *arg) {
	uint32_t angle, count;
	mcpwm_example_gpio_initialize();

	printf("Configuring Initial Parameters of mcpwm......\n");
	mcpwm_config_t pwm_config;
	pwm_config.frequency = 50;
	pwm_config.cmpr_a = 0;
	pwm_config.cmpr_b = 0;
	pwm_config.counter_mode = MCPWM_UP_COUNTER;
	pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
	mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);

	unsigned long send = 1;

	while (1) {
		for (count = 0; count < SERVO_MAX_DEGREE; count++) {
			printf("Angle of rotation: %d\n", count);
			angle = servo_per_degree_init(count);
			printf("pulse width: %dus\n", angle);
			mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A,
					angle);
			if (count == LED_LIM) {
				send = 1;
				xQueueSend(q, (void * ) &send, (TickType_t ) 0);
				printf("Recorded %lu \n", send);
			}
			vTaskDelay(2);
		}
		for (count = SERVO_MAX_DEGREE; count > 0; count--) {
			printf("Angle of rotation: %d\n", count);
			angle = servo_per_degree_init(count);
			printf("pulse width: %dus\n", angle);
			mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A,
					angle);
			if (count == LED_LIM) {
				send = 0;
				xQueueSend(q, (void * ) &send, (TickType_t ) 0);
				printf("Recorded %lu \n", send);
			}
			vTaskDelay(4);
		}
	}
}

void app_main() {
	printf("Launched.\n");
	q = xQueueCreate(20, sizeof(unsigned long));
	printf("Queue created.\n");
	printf("Initiating blink...\n");
	xTaskCreate(&blink_task, "blink_task", 4096, NULL, 5,
	NULL);
	printf("Initiating servo...");
	xTaskCreate(&mcpwm_example_servo_control, "mcpwm_example_servo_control",
			4096, NULL, 5, NULL);

}
