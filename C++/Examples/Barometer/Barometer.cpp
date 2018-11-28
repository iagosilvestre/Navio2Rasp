/*
Provided to you by Emlid Ltd (c) 2014.
twitter.com/emlidtech || www.emlid.com || info@emlid.com

Example: Get pressure from MS5611 barometer onboard of Navio shield for Raspberry Pi

To run this example navigate to the directory containing it and run following commands:
make
./Barometer
*/

#include <Common/MS5611.h>
#include "Common/I2Cdev.h"
#include <Common/Util.h>
#include <Common/Util.cpp>
#include <unistd.h>
#include <stdio.h>
#include <string>
#include <memory>
#include <stdint.h>
#include <sys/time.h>
#include <iostream>
#include <vector>
#define _GNU_SOURCE
#include <sched.h>

unsigned long int dtlong=0,auxCount=0,ledCount=0,count=0,countMax=25000;
float temperatura,pressao;
std::vector<int> baroData;

int main()
{
    MS5611 barometer;
    struct timeval t0, t1, dt;
    barometer.initialize();

    cpu_set_t my_set;        /* Define your cpu_set bit mask. */
   	CPU_ZERO(&my_set);       /* Initialize it all to 0, i.e. no CPUs selected. */
   	CPU_SET(0, &my_set);     /* set the bit that represents core 7. */
   	sched_setaffinity(0, sizeof(cpu_set_t), &my_set);
   	printf("sched_getcpu = %d\n", sched_getcpu());

   	ProfilerStart("baro.log");

    while (count<500) {
    	gettimeofday(&t0, NULL);
        barometer.refreshPressure();
        usleep(10000); // Waiting for pressure data ready
        barometer.readPressure();

        barometer.refreshTemperature();
        usleep(10000); // Waiting for temperature data ready
        barometer.readTemperature();

        barometer.calculatePressureAndTemperature();

        temperatura=barometer.getTemperature();
		pressao=barometer.getPressure();

        gettimeofday(&t1, NULL);
        timersub(&t1, &t0, &dt);
		baroData.push_back(dt.tv_usec-20000);
        //printf("Temperature(C): %f Pressure(millibar): %f\n",barometer.getTemperature(), barometer.getPressure());
        count++;
        usleep(10000);
    }
	FILE *fBaro = fopen("barometerS.txt", "w");
	fprintf(fBaro, "count;dtBaro\n");
	fclose(fBaro);
	for (std::vector<int>::iterator it = baroData.begin() ; it != baroData.end(); ++it){
		auxCount++;
		FILE *fBaro = fopen("barometerS.txt", "a");
		fprintf(fBaro, "%d;%d\n",auxCount,*it);
		fclose(fBaro);
	}


	ProfilerStop();


    return 0;
}
