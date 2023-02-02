#include "aht21/aht21.h"

int main(void)
{
	tadragon::i2c i2cdevice{"/dev/i2c-1"};
	tadragon::PiAHT21 aht21{i2cdevice};

	auto status = aht21.read_status();

	if(!status) {
		printf("Error...\n");
		return 0;
	}
	if((status.value() & AHT21_STATUS_BIT_CAL_ENABLE) != AHT21_STATUS_BIT_CAL_ENABLE) {
		return 0;
	}

	while (true)
	{
		float humi = 0;
		float tmp = 0;
		aht21.read_measurement(&humi,&tmp);
		printf("Temprature : %.2f\n",tmp);
		printf("humidity   : %.2f\n",humi);
		sleep(1);
	}

	return 0;
}