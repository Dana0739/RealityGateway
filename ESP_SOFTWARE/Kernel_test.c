/* RGW-P1 prototype kernel
 *
 * Inclusions.
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "driver/gpio.h"
#include "driver/mcpwm.h"

#include "sdkconfig.h"

#include "nvs_flash.h"

#include "esp_attr.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"

#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"

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

/*
 * Communication queues
 */

QueueHandle_t queue_e, queue_s = NULL;

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
#define IDLE_LIMIT 50
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
	static const char *TAG = "ENGINE INIT";

	ESP_LOGW(TAG, "Setting up L GPIO...");
	gpio_pad_select_gpio(ELP);
	gpio_set_direction(ELP, GPIO_MODE_OUTPUT);
	gpio_pad_select_gpio(ELCP);
	gpio_set_direction(ELCP, GPIO_MODE_OUTPUT);
	gpio_pad_select_gpio(ELCN);
	gpio_set_direction(ELCN, GPIO_MODE_OUTPUT);
	gpio_pad_select_gpio(EL);
	gpio_set_direction(EL, GPIO_MODE_OUTPUT);

	ESP_LOGW(TAG, "Setting up R GPIO...");
	gpio_pad_select_gpio(ERP);
	gpio_set_direction(ERP, GPIO_MODE_OUTPUT);
	gpio_pad_select_gpio(ERCP);
	gpio_set_direction(ERCP, GPIO_MODE_OUTPUT);
	gpio_pad_select_gpio(ERCN);
	gpio_set_direction(ERCN, GPIO_MODE_OUTPUT);
	gpio_pad_select_gpio(ER);
	gpio_set_direction(ER, GPIO_MODE_OUTPUT);

	ESP_LOGW(TAG, "Disabling engines...");
	turnOff(EL);
	turnOff(ER);

	ESP_LOGW(TAG, "Directing engines forward...");
	direct(LEFT, FWD);
	direct(RIGHT, FWD);

	front_l = true;
	front_r = true;
}

/*
 * Engine operator
 */

void app_engine() {
	static const char *TAG = "APP ENGINE";

	ESP_LOGW(TAG, "Entry...");

	while (1) {
		zero_countdown++;
		struct dataEngine dat;
		if (pdTRUE == xQueueReceive(queue_e, &dat, (TickType_t ) 0)) {
			l = dat.l;
			r = dat.r;
			zero_countdown = 0;
			ESP_LOGI(TAG, "Data received: l = %d, r = %d", l, r);
		} else {
			if (zero_countdown == IDLE_LIMIT) {
				l = 0;
				r = 0;
				ESP_LOGW(TAG, "Idleness limit exceeded.");
			}
		}

		cur_t = xTaskGetTickCount() * 100 / (portTICK_RATE_MS);
		printf("%ld \n", cur_t);
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
			ESP_LOGI(TAG, "Engine R is redirected backward.")
		}
		if ((!front_r) && (ctrl_r > 0)) {
			direct(RIGHT, BCK);
			front_r = true;
			ESP_LOGI(TAG, "Engine R is redirected forward.")
		}
		if ((front_l) && (ctrl_l < 0)) {
			direct(LEFT, FWD);
			front_l = false;
			ESP_LOGI(TAG, "Engine L is redirected backward.")
		}
		if ((!front_l) && (ctrl_l > 0)) {
			direct(LEFT, BCK);
			front_l = true;
			ESP_LOGI(TAG, "Engine L is redirected forward.")
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
	static const char *TAG = "SERVO INIT";

	ESP_LOGW(TAG, "Setting up GPIO...");
	mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, EP);
	mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0B, EY);

	ESP_LOGW(TAG, "Making MCPWM config...");
	mcpwm_config_t pwm_config;
	pwm_config.frequency = 50;
	pwm_config.cmpr_a = 0;
	pwm_config.cmpr_b = 0;
	pwm_config.counter_mode = MCPWM_UP_COUNTER;
	pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

	ESP_LOGW(TAG, "Setting up MCPWM config...");
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
	static const char *TAG = "APP SERVO";

	ESP_LOGW(TAG, "Entry...");

	while (1) {
		zero_countdown2++;
		struct dataServo dat;
		if (pdTRUE == xQueueReceive(queue_s, &dat, (TickType_t ) 0)) {
			pitch = dat.pitch;
			yaw = dat.yaw;
			zero_countdown2 = 0;
			ESP_LOGI(TAG, "APP_SERVO: Data received: pitch = %d, yaw = %d",
					pitch, yaw);
		} else {
			if (zero_countdown2 == IDLE_LIMIT) {
				pitch = 0;
				yaw = 0;
				ESP_LOGW(TAG, "Idleness limit exceeded.");
			}
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
 * WI-FI RECIEVER BLOCK
 ********************************
 */

/*
 * Local definitions
 */

//#define EXAMPLE_WIFI_SSID "Poot"
//#define EXAMPLE_WIFI_PASS "dispencer"
//
//#define WEB_SERVER "192.168.43.248"
//#define WEB_PORT 8000
//#define WEB_URL "/JSON.html"
#define EXAMPLE_WIFI_SSID "TP-LINK_D5D03C"
#define EXAMPLE_WIFI_PASS "dol99999"

#define WEB_SERVER "192.168.0.104"
#define WEB_PORT 8000
#define WEB_URL "/JSON.html"

/*
 * Local constants
 */

static EventGroupHandle_t wifi_event_group;
const int CONNECTED_BIT = BIT0;

static const char *REQUEST = "GET " WEB_URL " HTTP/1.1\r\n"
"Host: "WEB_SERVER"\r\n"
"User-Agent: esp-idf/1.0 esp32\r\n"
"\r\n";

/*
 * Event handler
 */

static esp_err_t event_handler(void *ctx, system_event_t *event) {
	switch (event->event_id) {
	case SYSTEM_EVENT_STA_START:
		esp_wifi_connect();
		break;
	case SYSTEM_EVENT_STA_GOT_IP:
		xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
		break;
	case SYSTEM_EVENT_STA_DISCONNECTED:
		/* This is a workaround as ESP32 WiFi libs don't currently
		 auto-reassociate. */
		esp_wifi_connect();
		xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
		break;
	default:
		break;
	}
	return ESP_OK;
}

/*
 * WI-FI initiator
 */

static void wifi_init(void) {
	static const char *TAG = "WIFI INIT";
	tcpip_adapter_init();
	wifi_event_group = xEventGroupCreate();
	ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT()
	;
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));
	ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
	wifi_config_t wifi_config = { .sta = { .ssid = EXAMPLE_WIFI_SSID,
			.password = EXAMPLE_WIFI_PASS, }, };
	ESP_LOGI(TAG, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
	ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
	ESP_ERROR_CHECK(esp_wifi_start());
}

/*
 * Generator
 */

void app_http() {
	static const char *TAG = "APP HTTP";

	ESP_LOGW(TAG, "Entry...");

	const struct addrinfo hints = { .ai_family = AF_INET, .ai_socktype =
	SOCK_STREAM, };
	struct addrinfo *res;
	int s, r, bal, lim;
	char recv_buf[256];
	char to_json[256];

	while (1) {
		xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT,
		false, true, portMAX_DELAY);

		int err = getaddrinfo(WEB_SERVER, "8000", &hints, &res);

		if (err != 0 || res == NULL) {
			ESP_LOGE(TAG, "DNS lookup failed err=%d res=%p", err, res);
			vTaskDelay(1000 / portTICK_PERIOD_MS);
			continue;
		}

		s = socket(res->ai_family, res->ai_socktype, 0);
		if (s < 0) {
			ESP_LOGE(TAG, "... Failed to allocate socket.");
			freeaddrinfo(res);
			vTaskDelay(1000 / portTICK_PERIOD_MS);
			continue;
		}

		if (connect(s, res->ai_addr, res->ai_addrlen) != 0) {
			ESP_LOGE(TAG, "... socket connect failed errno=%d", errno);
			close(s);
			freeaddrinfo(res);
			vTaskDelay(4000 / portTICK_PERIOD_MS);
			continue;
		}

		freeaddrinfo(res);

		if (write(s, REQUEST, strlen(REQUEST)) < 0) {
			ESP_LOGE(TAG, "... socket send failed");
			close(s);
			vTaskDelay(4000 / portTICK_PERIOD_MS);
			continue;
		}

		struct timeval receiving_timeout;
		receiving_timeout.tv_sec = 5;
		receiving_timeout.tv_usec = 0;
		if (setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &receiving_timeout,
				sizeof(receiving_timeout)) < 0) {
			ESP_LOGE(TAG, "... failed to set socket receiving timeout");
			close(s);
			vTaskDelay(4000 / portTICK_PERIOD_MS);
			continue;
		}

		bzero(to_json, sizeof(to_json));
		lim = 0;
		bal = 0;
		do {
			bzero(recv_buf, sizeof(recv_buf));
			r = read(s, recv_buf, sizeof(recv_buf) - 1);
			for (int i = 0; i < r; i++) {
				if (recv_buf[i] == '{') {
					bal++;
				}
				if (bal > 0) {
					to_json[lim++] = recv_buf[i];
				}
				if (recv_buf[i] == '}') {
					bal--;
				}
			}
		} while (r > 0);

		ESP_LOGI(TAG, "Located string: %s", to_json);

		cJSON *j = cJSON_Parse(to_json);

		if (j) {
			ESP_LOGI(TAG, "JSON parsed.");

			struct dataEngine dat_e;
			struct dataServo dat_s;

			dat_e.l = (int32_t) cJSON_GetObjectItem(j, "l")->valueint;
			dat_e.r = (int32_t) cJSON_GetObjectItem(j, "r")->valueint;
			dat_s.pitch = (int32_t) cJSON_GetObjectItem(j, "pitch")->valueint;
			dat_s.yaw = (int32_t) cJSON_GetObjectItem(j, "yaw")->valueint;

			ESP_LOGI(TAG,
					"APP_HTTP: Spreading data. Received: l = %d, r = %d, pitch = %d, yaw = %d \n",
					dat_e.l, dat_e.r, dat_s.pitch, dat_s.yaw)

			xQueueSend(queue_e, (void * ) &dat_e, (TickType_t ) 0);
			xQueueSend(queue_s, (void * ) &dat_s, (TickType_t ) 0);
		} else {
			ESP_LOGE(TAG, "Invalid string received.");
		}

		close(s);

		cJSON_Delete(j);

		vTaskDelay(50 / portTICK_PERIOD_MS);
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
	static const char *TAG = "QUEUE INIT";

	queue_e = xQueueCreate(20, sizeof(struct dataEngine));
	if (queue_e != 0) {
		ESP_LOGI(TAG, "Engine data flow queue created.");
	} else {
		ESP_LOGE(TAG, "Engine data flow queue failed to be created.")
	}

	queue_s = xQueueCreate(20, sizeof(struct dataServo));
	if (queue_s != 0) {
		ESP_LOGI(TAG, "Servo data flow queue created.");
	} else {
		ESP_LOGE(TAG, "Servo data flow queue failed to be created.")
	}
}

/*
 * System initiator
 */

void l_sys_init() {
	static const char *TAG = "SYS INIT";

	engine_init();
	ESP_LOGW(TAG, "Engine initiation passed.");

	servo_init();
	ESP_LOGW(TAG, "Servo initiation passed.")

	ESP_ERROR_CHECK(nvs_flash_init());
	ESP_LOGW(TAG, "Errors check passed.")

	wifi_init();
	ESP_LOGW(TAG, "Wi-Fi module started.")
}

/*
 * Task initiator
 */

void task_init() {
	static const char *TAG = "TASK INIT";

	BaseType_t app_engine_up;
	BaseType_t app_servo_up;
	BaseType_t app_http_up;

	app_engine_up = xTaskCreate(app_engine, "engine control task", STACK_SIZE,
	NULL, 5, NULL);
	if (app_engine_up == pdPASS) {
		ESP_LOGI(TAG, "Engine control task is on.");
	} else {
		ESP_LOGE(TAG, "Engine control task failed to launch.");
	}

	app_servo_up = xTaskCreate(app_servo, "servo control task", STACK_SIZE,
	NULL, 5, NULL);
	if (app_servo_up == pdPASS) {
		ESP_LOGI(TAG, "Servo control task is on.");
	} else {
		ESP_LOGE(TAG, "Servo control task failed to launch.");
	}

	app_http_up = xTaskCreate(&app_http, "HTTP task", STACK_SIZE, NULL, 5,
	NULL);
	if (app_http_up == pdPASS) {
		ESP_LOGI(TAG, "HTTP task is on.");
	} else {
		ESP_LOGE(TAG, "TASK_INIT: HTTP task failed to launch.");
	}
}

/*
 * Main application
 */

void app_main() {
	static const char *TAG = "APP MAIN";

	ESP_LOGW(TAG, "Launching kernel...");

	ESP_LOGI(TAG, "Initiating queues...");
	queue_init();
	ESP_LOGI(TAG, "Queues initiation passed.");

	ESP_LOGI(TAG, "Initiating systems...");
	l_sys_init();
	ESP_LOGI(TAG, "Systems initiation complete.");

	ESP_LOGI(TAG, "APP_MAIN: Initiating tasks...");
	task_init();
	ESP_LOGI(TAG, "Tasks initiation complete.");

	ESP_LOGW(TAG, "System is up. \n");
}
