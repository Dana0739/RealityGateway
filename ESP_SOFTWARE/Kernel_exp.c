/* RGW-P1 prototype kernel
 *
 * Inclusions
 */

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

/*
 * Boolean implementation
 */

typedef int boolean;
#define true 1
#define false 0

/*
 * Data structures
 */

struct dataEng {
	int l, r;
};

struct dataServo {
	int pitch, yaw;
};

struct data {
	int l, r, pitch, yaw;
};

/*
 * Communication queues
 */

QueueHandle_t queue_e, queue_s, queue_d = NULL;

/*
 ********************************
 * ENGINE BLOCK
 ********************************
 */

/*
 * Connection pins
 */

#define ELP 12
#define ELCP 11
#define ELCN 10
#define EL 9

#define ERP 5
#define ERCP 4
#define ERCN 3
#define ER 2

/*
 * System constants
 */

#define PER 50
#define ZERO_EDGE 5
#define IMPULSE 2
#define IDLE_LIMIT 250
#define LEFT 0
#define RIGHT 1
#define FWD 0
#define BCK 1

/*
 * Global variables
 */

long cur_t, prev_t, inter;
int l, r;
double ctrl_l, ctrl_r, ctrl_l_p, ctrl_r_p;
boolean engaged_l, engaged_r, front_l, front_r;
long eng_t_l, eng_t_r, idl_t_l, idl_t_r, zero_l, zero_r, zero_countdown;
/*
 * Power controllers
 */

void turnOff(int x) {
	gpio_set_level(x, 0);
}

void turnOn(int x) {
	gpio_set_level(x, 1);
}

/*
 * Engine director
 */

void direct(int side, int hdg) {
	if (side == LEFT) {
		gpio_set_level(ELP, 0);
		vTaskDelay(IMPULSE / portTICK_PERIOD_MS);
		if (hdg == BCK) {
			gpio_set_level(ELCP, 0);
			gpio_set_level(ELCN, 1);
		} else {
			gpio_set_level(ELCP, 1);
			gpio_set_level(ELCN, 0);
		}
		gpio_set_level(ELP, 1);
	} else {
		gpio_set_level(ERP, 0);
		vTaskDelay(IMPULSE / portTICK_PERIOD_MS);
		if (hdg == BCK) {
			gpio_set_level(ERCP, 0);
			gpio_set_level(ERCN, 1);
		} else {
			gpio_set_level(ERCP, 1);
			gpio_set_level(ERCN, 0);
		}
		gpio_set_level(ERP, 1);
	}
}

/*
 * Engine subsystem initiator
 */

void enigne_init(void) {
	gpio_pad_select_gpio(ELP);
	gpio_set_direction(ELP, GPIO_MODE_OUTPUT);
	gpio_pad_select_gpio(ELCP);
	gpio_set_direction(ELCP, GPIO_MODE_OUTPUT);
	gpio_pad_select_gpio(ELCN);
	gpio_set_direction(ELCN, GPIO_MODE_OUTPUT);
	gpio_pad_select_gpio(EL);
	gpio_set_direction(EL, GPIO_MODE_OUTPUT);

	gpio_pad_select_gpio(ERP);
	gpio_set_direction(ERP, GPIO_MODE_OUTPUT);
	gpio_pad_select_gpio(ERCP);
	gpio_set_direction(ERCP, GPIO_MODE_OUTPUT);
	gpio_pad_select_gpio(ERCN);
	gpio_set_direction(ERCN, GPIO_MODE_OUTPUT);
	gpio_pad_select_gpio(ER);
	gpio_set_direction(ER, GPIO_MODE_OUTPUT);

	turnOff(EL);
	turnOff(ER);

	direct(LEFT, FWD);
	direct(RIGHT, FWD);

	front_l = true;
	front_r = true;
}

/*
 * Engine operator
 */

void app_engine(void) {
	while (1) {
		zero_countdown++;
		struct dataEng dat;
		if (queue_e != NULL) {
			xQueueReceive(queue_e, &dat,
					(TickType_t) (1000 / portTICK_PERIOD_MS));
			l = dat->l;
			r = dat->r;
		} else {
			if (zero_countdown > IDLE_LIMIT) {
				l = 0;
				r = 0;
			}
		}

		cur_t = xTaskGetTickCount() / (portTICK_RATE_MS * 1000);
		inter = cur_t - prev_t;
		prev_t = cur_t;

		ctrl_r = r;
		ctrl_l = l;
		ctrl_r /= 512;
		ctrl_l /= 512;
		ctrl_r *= PER;
		ctrl_l *= PER;

		if (abs(ctrl_l) < ZERO_EDGE)
			ctrl_l = 0;
		if (abs(ctrl_r) < ZERO_EDGE)
			ctrl_r = 0;

		if ((ctrl_l_p < ZERO_EDGE) && (ctrl_l >= ZERO_EDGE)) {
			ctrl_l = PER;
		}
		if ((ctrl_l_p > -ZERO_EDGE) && (ctrl_l <= -ZERO_EDGE)) {
			ctrl_l = -PER;
		}
		if ((ctrl_r_p < ZERO_EDGE) && (ctrl_r >= ZERO_EDGE)) {
			ctrl_r = PER;
		}
		if ((ctrl_r_p > -ZERO_EDGE) && (ctrl_r <= -ZERO_EDGE)) {
			ctrl_r = -PER;
		}

		if ((front_r) && (ctrl_r < 0)) {
			direct(RIGHT, FWD);
			front_r = false;
		}
		if ((!front_r) && (ctrl_r > 0)) {
			direct(1, BCK);
			front_r = true;
		}
		if ((front_l) && (ctrl_l < 0)) {
			direct(LEFT, FWD);
			front_l = false;
		}
		if ((!front_l) && (ctrl_l > 0)) {
			direct(LEFT, BCK);
			front_l = true;
		}

		if (engaged_l) {
			eng_t_l += inter;
			if (eng_t_l > ctrl_l) {
				engaged_l = false;
				turnOff(EL);
				idl_t_l = 0;
			}
		} else {
			idl_t_l += inter;
			if (idl_t_l > PER - ctrl_l) {
				engaged_l = true;
				turnOn(EL);
				eng_t_l = 0;
			}
		}

		if (engaged_r) {
			eng_t_r += inter;
			if (eng_t_r > ctrl_r) {
				engaged_r = false;
				turnOff(ER);
				idl_t_r = 0;
			}
		} else {
			idl_t_r += inter;
			if (idl_t_r > PER - ctrl_r) {
				engaged_r = true;
				turnOn(ER);
				eng_t_r = 0;
			}
		}

		ctrl_l_p = ctrl_l;
		ctrl_r_p = ctrl_r;
	}

/*
 ********************************
 * SERVO BLOCK
 ********************************
 */

}
