#include <iostream>
#include <stdio.h>
#include <unistd.h>

using namespace std;

int main()
{
	cout << "Blinky..." << endl;
	FILE *LEDHandle = NULL;
	const char *LEDBrightness="/sys/class/leds/beaglebone:green:usr1/brightness";

	for(int i=0; i<10; i++){
		if((LEDHandle = fopen(LEDBrightness, "r+")) != NULL){
			fwrite("1", sizeof(char), 1, LEDHandle);
			fclose(LEDHandle);
		}
		usleep(1000000);

		if((LEDHandle = fopen(LEDBrightness, "r+")) != NULL){
			fwrite("0", sizeof(char), 1, LEDHandle);
			fclose(LEDHandle);
		}
		usleep(1000000);
	}
	cout << "End" << endl;

}

