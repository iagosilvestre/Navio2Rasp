/*
Provided to you by Emlid Ltd (c) 2015.
twitter.com/emlidtech || www.emlid.com || info@emlid.com

Example: Read accelerometer, gyroscope and magnetometer values from
inertial measurement unit: MPU9250 or LSM9DS1 over SPI on Raspberry Pi + Navio.

Navio's onboard sensors are connected to the SPI bus on Raspberry Pi
and can be read through /dev/spidev0.1 (MPU9250), /dev/spidev0.3 (acc/gyro LSM9DS1)
and /dev/spidev0.2 (mag LSM9DS1).

To run this example navigate to the directory containing it and run following commands:
make
./AccelGyroMag -i [sensor name]
Sensors names: mpu is MPU9250, lsm is LSM9DS1.
For print help:
./AccelGyroMag -h
*/
#define _GNU_SOURCE
#include <sched.h>
#include <Navio2/Led_Navio2.h>
#include <Common/Ublox.h>
#include <Common/MS5611.h>
#include <string>
#include <stdio.h>
#include <memory>
#include <stdint.h>
#include <unistd.h>
#include <sys/time.h>
#include <Common/MPU9250.h>
#include <Navio2/LSM9DS1.h>
#include <Common/Util.h>
#include <pthread.h>
#include <iostream>
#include <vector>
#include <Common/profiler.h>

#define G_SI 9.80665
#define PI   3.14159

using namespace std;
	
	std::vector<int> baroData;
	std::vector<int> mpuData;
	std::vector<int> lsmData;
	std::vector<int> ledData;
	
	struct timeval t0,t1,dtBARO,dtMPU,dtLED,dtLSM;
	float dt;
	unsigned long int dtlong=0,auxCount=0,ledCount=0,count=0,countMax=5000;
	
	float ax, ay, az;
	float gx, gy, gz;
	float mx, my, mz;
	
	float ax2, ay2, az2;
	float gx2, gy2, gz2;
	float mx2, my2, mz2;
	
	float temperatura,pressao;
	
	
std::unique_ptr <InertialSensor> get_inertial_sensor( std::string sensor_name)
{
    if (sensor_name == "mpu") {
        printf("Selected: MPU9250\n");
        auto ptr = std::unique_ptr <InertialSensor>{ new MPU9250() };
        return ptr;
    }
    else if (sensor_name == "lsm") {
        printf("Selected: LSM9DS1\n");
        auto ptr = std::unique_ptr <InertialSensor>{ new LSM9DS1() };
        return ptr;
    }
    else {
        return NULL;
    }
}
void print_help()
{
    printf("Possible parameters:\nSensor selection: -i [sensor name]\n");
    printf("Sensors names: mpu is MPU9250, lsm is LSM9DS1\nFor help: -h\n");
}

std::string get_sensor_name(int argc, char *argv[])
{
    if (get_navio_version() == NAVIO2) {

        if (argc < 2) {
            printf("Enter parameter\n");
            print_help();
            return std::string();
        }

        // prevent the error message
        opterr = 0;
        int parameter;

        while ((parameter = getopt(argc, argv, "i:h")) != -1) {
            switch (parameter) {
            case 'i': if (!strcmp(optarg,"mpu") ) return "mpu";
                            else return "lsm";
            case 'h': print_help(); return "-1";
            case '?': printf("Wrong parameter.\n");
                      print_help();
                      return std::string();
            }
        }

    } else { //sensor on NAVIO+

        return "mpu";
    }

}
//=============================================================================
int main(int argc, char *argv[])
{
	MS5611 barometer;
	barometer.initialize();

    /*auto sensor_name = get_sensor_name(argc, argv);
    if (sensor_name.empty())
        return EXIT_FAILURE;*/

    auto sensor = get_inertial_sensor("mpu");
    auto sensor2 = get_inertial_sensor("lsm");

    if (!sensor->probe()) {
        printf("Sensor not enabled\n");
        return EXIT_FAILURE;
    }
    sensor->initialize();
    sensor2->initialize();

	
//-------------------------------------------------------------------------
    while(count<countMax) {
    	count++;
//----------------Obtencao do tempo antes da leitura dos sensores---------------------------------//

//----------------Escrita no PWM  ---------------------------------//
//		gettimeofday(&t0,NULL);
//
//		led->setColor(Colors::Green);
//
//		gettimeofday(&t1, NULL);
//		timersub(&t1, &t0, &dtLED);
//		ledData.push_back(dtLED.tv_usec);
//----------------Leitura da IMU MPU ---------------------------------//
		gettimeofday(&t0,NULL);

        sensor->update();
        sensor->read_accelerometer(&ax, &ay, &az);
        sensor->read_gyroscope(&gx, &gy, &gz);
        sensor->read_magnetometer(&mx, &my, &mz);

        gettimeofday(&t1,NULL);
        timersub(&t1, &t0, &dtMPU);
		mpuData.push_back(dtMPU.tv_usec);
//----------------Leitura da IMU LSM---------------------------------//
		gettimeofday(&t0,NULL);

        sensor2->update();
        sensor2->read_accelerometer(&ax2, &ay2, &az2);
        sensor2->read_gyroscope(&gx2, &gy2, &gz2);
        sensor2->read_magnetometer(&mx2, &my2, &mz2);

		gettimeofday(&t1,NULL);
		timersub(&t1, &t0, &dtLSM);
		lsmData.push_back(dtLSM.tv_usec);
//----------------Leitura do barometro ---------------------------------//


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
        timersub(&t1, &t0, &dtBARO);
		baroData.push_back(dtBARO.tv_usec-20000);



       }
    //----------------Print dos tempos em um log txt ---------------------------------//
    FILE *fBaro = fopen("barometerOO.txt", "w");
    	fprintf(fBaro, "count;dtBaro\n");
    	fclose(fBaro);
    	for (std::vector<int>::iterator it = baroData.begin() ; it != baroData.end(); ++it){
    		auxCount++;
    		FILE *fBaro = fopen("barometerOO.txt", "a");
    		fprintf(fBaro, "%d;%d\n",auxCount,*it);
    		fclose(fBaro);
    	}


    	auxCount=0;
    	FILE *fMPU = fopen("mpuOO.txt", "w");
    	fprintf(fMPU, "count;dtMPU\n");
    	fclose(fMPU);
    	for (std::vector<int>::iterator it = mpuData.begin() ; it != mpuData.end(); ++it){
    		auxCount++;
    		FILE *fMPU = fopen("mpuOO.txt", "a");
    		fprintf(fMPU, "%d;%lu\n",auxCount,*it);
    		fclose(fMPU);
    	}


    	auxCount=0;
    	FILE *fLSM = fopen("lsmOO.txt", "w");
    	fprintf(fLSM, "count;dtLSM\n");
    	fclose(fLSM);
    	for (std::vector<int>::iterator it = lsmData.begin() ; it != lsmData.end(); ++it){
    		auxCount++;
    		FILE *fLSM = fopen("lsmOO.txt", "a");
    		fprintf(fLSM, "%d;%lu\n",auxCount,*it);
    		fclose(fLSM);
    	}


    	auxCount=0;
    	FILE *fLed = fopen("ledOO.txt", "w");
    	fprintf(fLed, "count;dtLed\n");
    	fclose(fLed);
    	for (std::vector<int>::iterator it = ledData.begin() ; it != ledData.end(); ++it){
    		auxCount++;
    		FILE *fLed = fopen("ledOO.txt", "a");
    		fprintf(fLed, "%d;%lu\n",auxCount,*it);
    		fclose(fLed);
    	}
}

