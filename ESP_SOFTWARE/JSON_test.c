#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "cJSON.h"

int app_main(void) {
	printf("Version: %s\n", cJSON_Version());

	char test_str[] =
			"{\n\"name\": \"Jack (\\\"Bee\\\") Nimble\", \n\"format\": {\"type\":       \"rect\", \n\"width\":      1920, \n\"height\":     1080, \n\"interlace\":  false,\"frame rate\": 24\n}\n}";

	cJSON * test = cJSON_Parse(test_str);

	char *res = cJSON_Print(test);

	printf("%s\n", res);
	printf("%d\n", cJSON_GetObjectItem(cJSON_GetObjectItem(test, "format"), "width")->valueint);

	free(res);
	free(test);

	return 0;
}
