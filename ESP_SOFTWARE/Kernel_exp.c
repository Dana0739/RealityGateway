/* RGW-P1 prototype kernel
 *
 * Inclusions.
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
#include <string.h>
#include "cJSON.h"

/*
 * Boolean implementation.
 */

typedef int boolean;
#define true 1
#define false 0

/*
 * Data structures
 */

struct dataEngine {
	int32_t l, r;
};

struct dataServo {
	int32_t pitch, yaw;
};

struct data {
	int32_t l, r, pitch, yaw;
};

/*
 * Communication queues
 */

QueueHandle_t queue_e, queue_s, queue_d, queue_m = NULL;

/*
 ********************************
 * ENGINE BLOCK
 ********************************
 */

/*
 * Connection pins
 */

#define ELP 23
#define ELCP 22
#define ELCN 21
#define EL 19

#define ERP 18
#define ERCP 5
#define ERCN 17
#define ER 16

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

uint64_t cur_t, prev_t, inter;
int32_t l, r;
double ctrl_l, ctrl_r, ctrl_l_p, ctrl_r_p;
boolean engaged_l, engaged_r, front_l, front_r;
uint64_t eng_t_l, eng_t_r, idl_t_l, idl_t_r, zero_l, zero_r, zero_countdown;
/*
 * Power controllers
 */

void turnOff(int32_t x) {
	gpio_set_level(x, 0);
}

void turnOn(int32_t x) {
	gpio_set_level(x, 1);
}

/*
 * Engine director
 */

void direct(uint32_t side, uint32_t hdg) {
	if (side == LEFT) {
		gpio_set_level(ELP, 0);
		vTaskDelay(IMPULSE);
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
		vTaskDelay(IMPULSE);
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

void engine_init() {
	printf("ENGINE_INIT: setting up L GPIO... \n");
	gpio_pad_select_gpio(ELP);
	gpio_set_direction(ELP, GPIO_MODE_OUTPUT);
	gpio_pad_select_gpio(ELCP);
	gpio_set_direction(ELCP, GPIO_MODE_OUTPUT);
	gpio_pad_select_gpio(ELCN);
	gpio_set_direction(ELCN, GPIO_MODE_OUTPUT);
	gpio_pad_select_gpio(EL);
	gpio_set_direction(EL, GPIO_MODE_OUTPUT);

	printf("ENGINE_INIT: setting up R GPIO... \n");
	gpio_pad_select_gpio(ERP);
	gpio_set_direction(ERP, GPIO_MODE_OUTPUT);
	gpio_pad_select_gpio(ERCP);
	gpio_set_direction(ERCP, GPIO_MODE_OUTPUT);
	gpio_pad_select_gpio(ERCN);
	gpio_set_direction(ERCN, GPIO_MODE_OUTPUT);
	gpio_pad_select_gpio(ER);
	gpio_set_direction(ER, GPIO_MODE_OUTPUT);

	printf("ENGINE_INIT: disabling engines... \n");
	turnOff(EL);
	turnOff(ER);

	printf("ENGINE_INIT: directing engines forward... \n");
	direct(LEFT, FWD);
	direct(RIGHT, FWD);

	front_l = true;
	front_r = true;
}

/*
 * Engine operator
 */

void app_engine() {
	printf("APP_ENGINE: Entry... \n");
	while (1) {
		//printf("APP_ENGINE: Scanning... \n");
		zero_countdown++;
		struct dataEngine dat;
		if (pdTRUE == xQueueReceive(queue_e, &dat, (TickType_t ) 0)) {
			l = dat.l;
			r = dat.r;
			printf("APP_ENGINE: Data received: l = %d, r = %d \n", l, r);
		} else {
			if (zero_countdown == IDLE_LIMIT) {
				l = 0;
				r = 0;
				printf("APP_ENGINE: Idleness limit exceeded. \n");
			}
			//printf("APP_ENGINE: No data received. \n");
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
			printf("APP_ENG: Engine R redirected backward. \n");
		}
		if ((!front_r) && (ctrl_r > 0)) {
			direct(RIGHT, BCK);
			front_r = true;
			printf("APP_ENG: Engine R redirected forward. \n");
		}
		if ((front_l) && (ctrl_l < 0)) {
			direct(LEFT, FWD);
			front_l = false;
			printf("APP_ENG: Engine L redirected backward. \n");
		}
		if ((!front_l) && (ctrl_l > 0)) {
			direct(LEFT, BCK);
			front_l = true;
			printf("APP_ENG: Engine L redirected forward. \n");
		}

		//printf("APP_ENG: Changing engine power... \n");
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

		vTaskDelay(IMPULSE);
	}
}

/*
 ********************************
 * SERVO BLOCK
 ********************************
 */

/*
 * Connections pins
 */

#define EP 2
#define EY 15

/*
 * System constants
 */

#define SERVO_MIN_PULSEWIDTH 1000
#define SERVO_MAX_PULSEWIDTH 2000
#define SERVO_MAX_DEGREE 90

/*
 * Global variables
 */

uint32_t zero_countdown2, angle_y, angle_p;
int32_t pitch, yaw;

/*
 * Servo initiator
 */

void servo_init() {
	printf("SERVO_INIT: setting up GPIO... \n");
	mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, EP);
	mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0B, EY);

	printf("SERVO_INIT: making MCPWM config... \n");
	mcpwm_config_t pwm_config;
	pwm_config.frequency = 50;
	pwm_config.cmpr_a = 0;
	pwm_config.cmpr_b = 0;
	pwm_config.counter_mode = MCPWM_UP_COUNTER;
	pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

	printf("SERVO_INIT: setting up MCPWM config... \n");
	mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
	mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_0, &pwm_config);
}

/*
 * Transfer function
 */

static uint32_t servo_per_degree_init(uint32_t degree_of_rotation) {
	uint32_t cal_pulsewidth = 0;
	cal_pulsewidth = (SERVO_MIN_PULSEWIDTH
			+ (((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH)
					* (degree_of_rotation)) / (SERVO_MAX_DEGREE)));
	return cal_pulsewidth;
}

/*
 * Servo operator
 */

void app_servo() {
	printf("APP_SERVO: Entry... \n");
	while (1) {
		zero_countdown2++;
		struct dataServo dat;
		if (pdTRUE == xQueueReceive(queue_s, &dat, (TickType_t ) 0)) {
			pitch = dat.pitch;
			yaw = dat.yaw;
			printf("APP_SERVO: Data received: pitch = %d, yaw = %d \n", pitch,
					yaw);
		} else {
			if (zero_countdown == IDLE_LIMIT) {
				pitch = 0;
				yaw = 0;
				printf("APP_SERVO: Idleness limit exceeded. \n");
			}
			//printf("APP_SERVO: No data received. \n");
		}

		angle_p = servo_per_degree_init(pitch);
		angle_y = servo_per_degree_init(yaw);

		mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, angle_p);
		mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, angle_y);

		vTaskDelay(1);
	}
}

/*
 ********************************
 * DATA OVERRIDE BLOCK
 ********************************
 */

/*
 * Receive operator
 */

void app_override() {
	printf("APP_OVERRIDE: Entry... \n");
	while (1) {
		struct data dat;
		struct dataEngine dat_e;
		struct dataServo dat_s;
		if (pdTRUE == xQueueReceive(queue_d, &dat, (TickType_t ) 0)) {
			dat_e.l = dat.l;
			dat_e.r = dat.r;
			dat_s.pitch = dat.pitch;
			dat_s.yaw = dat.yaw;

			printf(
					"APP_OVERRIDE: Spreading data. Received: l = %d, r = %d, pitch = %d, yaw = %d \n",
					dat.l, dat.r, dat.pitch, dat.yaw);

			xQueueSend(queue_e, (void * ) &dat_e, (TickType_t ) 0);
			xQueueSend(queue_s, (void * ) &dat_s, (TickType_t ) 0);
		}
		vTaskDelay(IMPULSE);
	}
}

/*
 ********************************
 * DATA GENERATION BLOCK
 ********************************
 */

/*
 * Generator
 */

void app_gen() {
	printf("APP_GEN: Entry... \n");
	while (1) {

		/*
		 * Generator
		 */

//		char ret[] =
//				"{\"l\" : 350, \"r\" : -30, \"pitch\" : 300, \"yaw\" : -340}";
//		xQueueSend(queue_m, (void * ) &ret, (TickType_t ) 0);
//		printf("APP_GEN: Sending data - %s \n", ret);
//		vTaskDelay(2000 / portTICK_PERIOD_MS);
		/*
		 * Serial converter
		 */

		char s[200], sp[200];
		while (1) {
			scanf("%s \n", s);
			int same = 1;
			for (int i = 0; i < 200; i++) {
				if (s[i] != sp[i]) {
					same = 0;
				}
			}
			if (same == 0) {
				xQueueSend(queue_m, (void * ) &s, (TickType_t ) 0);
				printf("APP_GEN: Sending data - %s \n", s);
			}
			for (int i = 0; i < 200; i++) {
				sp[i] = s[i];
			}

			vTaskDelay(1000 / portTICK_PERIOD_MS);
		}
	}

}

/*
 ********************************
 * DATA UNPACK BLOCK
 ********************************
 */

/*
 * JSON operator
 */

void app_receive() {
	char cmd[200];
	printf("APP_RECEIVE: Entry... \n");
	while (1) {
		if (pdTRUE == xQueueReceive(queue_m, &cmd, (TickType_t ) 0)) {
			printf("APP_RECEIVE: Received string - %s \n", cmd);

			cJSON *j = cJSON_Parse(cmd);
			if (j) {
				printf("APP_RECEIVE: JSON parsed. \n");

				struct data dat;
				dat.l = (int32_t) cJSON_GetObjectItem(j, "l")->valueint;
				dat.r = (int32_t) cJSON_GetObjectItem(j, "r")->valueint;
				dat.pitch = (int32_t) cJSON_GetObjectItem(j, "pitch")->valueint;
				dat.yaw = (int32_t) cJSON_GetObjectItem(j, "yaw")->valueint;

				printf("APP_RECEIVE: Sending data... \n");
				xQueueSend(queue_d, (void * ) &dat, (TickType_t ) 0);
			} else {
				printf("APP_RECEIVE: Invalid string received. \n");
			}
		} else {
			//printf("APP_RECEIVEL: No data received. \n");
		}
		vTaskDelay(IMPULSE);
	}
}

/*
 ********************************
 * MAIN BLOCK
 ********************************
 */

/*
 * System constants
 */

#define STACK_SIZE 4096

/*
 * Queue initiator
 */

void queue_init() {
	queue_e = xQueueCreate(20, sizeof(struct dataEngine));
	printf("QUEUE INIT: Engine data flow queue created. \n");
	queue_s = xQueueCreate(20, sizeof(struct dataServo));
	printf("QUEUE INIT: Servo data flow queue created. \n");
	queue_d = xQueueCreate(20, sizeof(struct data));
	printf("QUEUE INIT: Unpacked data flow queue created. \n");
	queue_m = xQueueCreate(20, sizeof(char[200]));
	printf("QUEUE INIT: Raw data flow queue created. \n");
}

/*
 * System initiator
 */

void sys_init() {
	engine_init();
	printf("SYS_INIT: Engine system is set. \n");
	servo_init();
	printf("SYS_INIT: Servo system is set. \n");
}

/*
 * Task initiator
 */

void task_init() {
	BaseType_t app_engine_up;
	BaseType_t app_servo_up;
	BaseType_t app_gen_up;
	BaseType_t app_receive_up;
	BaseType_t app_override_up;

	app_engine_up = xTaskCreate(app_engine, "engine control task", STACK_SIZE,
	NULL, 5, NULL);
	if (app_engine_up == pdPASS) {
		printf("TASK_INIT: Engine control task is on. \n");
	} else {
		printf("TASK_INIT: Engine control task failed to launch. \n");
	}

	app_servo_up = xTaskCreate(app_servo, "servo control task", STACK_SIZE,
			NULL, 5, NULL);
	if (app_servo_up == pdPASS) {
		printf("TASK_INIT: Servo control task is on. \n");
	} else {
		printf("TASK_INIT: Servo control task failed to launch. \n");
	}

	app_gen_up = xTaskCreate(app_gen, "generator task", STACK_SIZE, NULL, 5,
			NULL);
	if (app_gen_up == pdPASS) {
		printf("TASK_INIT: Generator task is on. \n");
	} else {
		printf("TASK_INIT: Generator task failed to launch. \n");
	}

	app_receive_up = xTaskCreate(app_receive, "receiver task", STACK_SIZE, NULL,
			5, NULL);
	if (app_receive_up == pdPASS) {
		printf("TASK_INIT: Receiver task is on. \n");
	} else {
		printf("TASK_INIT: Receiver task failed to launch. \n");
	}

	app_override_up = xTaskCreate(app_override, "overrider task", STACK_SIZE,
			NULL, 5, NULL);
	if (app_override_up == pdPASS) {
		printf("TASK_INIT: Overrider task is on. \n");
	} else {
		printf("TASK_INIT: Overrider task failed to launch. \n");
	}
}

/*
 * Main applicato
 */

void app_main() {
	printf("APP_MAIN: Launching kernel... \n");

	printf("APP_MAIN: Initiating queues... \n");
	queue_init();
	printf("APP_MAIN: Queues initiation complete. \n");

	printf("APP_MAIN: Initiating systems... \n");
	sys_init();
	printf("APP_MAIN: Systems initiation complete. \n");

	printf("APP_MAIN: Initiating tasks... \n");
	task_init();
	printf("APP_MAIN: Tasks initiation complete. \n");

	printf("APP_MAIN: System is up. \n");
}
