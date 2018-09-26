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
#include <Navio2/Led_Navio2.h>
#include <Navio+/Led_Navio.h>
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
#include <iostream>       // std::cout
#include <thread>         // std::thread, std::this_thread::sleep_for
#include <chrono>

#define G_SI 9.80665
#define PI   3.14159

	    float ax, ay, az;
	    float gx, gy, gz;
	    float mx, my, mz;
	    float ax2, ay2, az2;
	    float gx2, gy2, gz2;
	    float mx2, my2, mz2;

		int swBaro=0,swMPU=0,swLSM=0,swLed=0;
	    struct timeval baro1,baro2,mpu1,mpu2,lsm1,lsm2,led1,led2,tot1,tot2;
		float dt;
		unsigned long int dtlong=0,count=0,dtMPU=0,dtLSM=0,dtLED=0,dtBaro=0,dtTot=0,countMax=500;

	    float temperatura,pressao;

using namespace std;

std::unique_ptr <Led> get_led()
{
    if (get_navio_version() == NAVIO2)
    {
        auto ptr = std::unique_ptr <Led>{ new Led_Navio2() };
        return ptr;
    } else
    {
        auto ptr = std::unique_ptr <Led>{ new Led_Navio() };
        return ptr;
    }
}


void * acquireBarometerData(void * barom)
{
	//unsigned long int previoustime=0, currenttime=0;
	unsigned long int baroCount=0;
    MS5611* barometer = (MS5611*)barom;
    while (count<countMax) {
    	if(swBaro==0){
    	baroCount++;
    	gettimeofday(&baro1,NULL);
        barometer->refreshPressure();
        usleep(10000); // Waiting for pressure data ready
        barometer->readPressure();

        barometer->refreshTemperature();
        usleep(10000); // Waiting for temperature data ready
        barometer->readTemperature();

        barometer->calculatePressureAndTemperature();

        temperatura=barometer->getTemperature();

        pressao=barometer->getPressure();
        gettimeofday(&baro2,NULL);
        dtBaro=(1000000 * baro2.tv_sec + baro2.tv_usec)-1000000 * baro1.tv_sec - baro1.tv_usec-20000;
        swBaro=1;
        if(baroCount==1){
        	FILE *f = fopen("barometer.txt", "w");
        	fprintf(f, "count;dtBaro\n");
        	fprintf(f, "%d;%lu\n",baroCount,dtBaro);
        	fclose(f);
        }
        else if(baroCount>1){
        	FILE *f = fopen("barometer.txt", "a");
        	fprintf(f, "%d;%lu\n",baroCount,dtBaro);
        	fclose(f);
        }
        usleep(5000);
    }
    }

    pthread_exit(NULL);
}
void * acquireMPUData(void * imuMPU)
{
	unsigned long int mpuCount=0;
	MPU9250* mpu=(MPU9250*)imuMPU;
	while(count<countMax){
		if(swMPU==0){
		mpuCount++;
    	gettimeofday(&mpu1,NULL);
		mpu->update();
		mpu->read_accelerometer(&ax, &ay, &az);
		mpu->read_gyroscope(&gx, &gy, &gz);
		mpu->read_magnetometer(&mx, &my, &mz);
		gettimeofday(&mpu2,NULL);
		dtMPU=(1000000 * mpu2.tv_sec + mpu2.tv_usec)-1000000 * mpu1.tv_sec - mpu1.tv_usec ;
		swMPU=1;
		if(mpuCount==1){
		        	FILE *f = fopen("mpu.txt", "w");
		        	fprintf(f, "count;dtMPU\n");
		        	fprintf(f, "%d;%lu\n",mpuCount,dtMPU);
		        	fclose(f);
		        }
		        else if(mpuCount>1){
		        	FILE *f = fopen("mpu.txt", "a");
		        	fprintf(f, "%d;%lu\n",mpuCount,dtMPU);
		        	fclose(f);
		        }
		usleep(5000);
	}
	}
	pthread_exit(NULL);
}
void * acquireLSMData(void * imuLSM)
{
	unsigned long int lsmCount=0;
	LSM9DS1* lsm=(LSM9DS1*)imuLSM;
	while(count<countMax){
		if(swLSM==0){
		lsmCount++;
		gettimeofday(&lsm1,NULL);
		lsm->update();
		lsm->read_accelerometer(&ax2, &ay2, &az2);
		lsm->read_gyroscope(&gx2, &gy2, &gz2);
		lsm->read_magnetometer(&mx2, &my2, &mz2);
		gettimeofday(&lsm2,NULL);
		dtLSM=(1000000 * lsm2.tv_sec + lsm2.tv_usec)-1000000 * lsm1.tv_sec - lsm1.tv_usec ;
		swLSM=1;
		if(lsmCount==1){
				FILE *f = fopen("lsm.txt", "w");
				fprintf(f, "count;dtLSM\n");
				fprintf(f, "%d;%lu\n",lsmCount,dtLSM);
				fclose(f);
			}
		else if(lsmCount>1){
			FILE *f = fopen("lsm.txt", "a");
			fprintf(f, "%d;%lu\n",lsmCount,dtLSM);
			fclose(f);
		}
		usleep(5000);
	}
	}
	pthread_exit(NULL);
}

void * acquireLedData(void * led)
{
	unsigned long int ledCount=0;
	Led_Navio2* diode=(Led_Navio2*)led;
	while(count<countMax){
		if(swLed==0){
		ledCount++;
		gettimeofday(&led1,NULL);
    	if((ledCount%2)==0){
    		diode->setColor(Colors::Red);
    	}
    	else{
    		diode->setColor(Colors::Green);
    	}
    	gettimeofday(&led2,NULL);
		dtLED=(1000000 * led2.tv_sec + led2.tv_usec)-1000000 * led1.tv_sec - led1.tv_usec ;
		swLed=1;
		if(ledCount==1){
			  FILE *f = fopen("led.txt", "w");
			  fprintf(f, "count;dtLED\n");
			  fprintf(f, "%d;%lu\n",ledCount,dtLED);
			  fclose(f);
				}
		else if(ledCount>1 & ledCount<5000){
			FILE *f = fopen("led.txt", "a");
			fprintf(f, "%d;%lu\n",ledCount,dtLED);
			fclose(f);
		}
		usleep(200000);
	}
	}

	pthread_exit(NULL);
}
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
	unsigned long int min=0,max=0,mem=0,media=0,sum=0;
	if (check_apm()) {
	        return 1;
	    }

	Led_Navio2 led;
	MS5611 baro;
	MPU9250 imuMPU;
	LSM9DS1 imuLSM;

	/*std::cout << "Spawning 4 threads...\n";
	std::thread t1 (acquireBarometerData,(void *)&baro);
	std::thread t2 (acquireLSMData,(void *)&imuLSM);
	std::thread t3 (acquireMPUData,(void *)&imuMPU);
	//std::thread t4 (acquireLedData,&led);
	std::cout << "Done spawning threads. Now waiting for them to join:\n";
	t1.join();
	t2.join();
	t3.join();
	//t4.join();*/
	pthread_t baro_thread;
	pthread_t MPU_thread;
	pthread_t LSM_thread;
	pthread_t led_thread;

	baro.initialize();
	    if(pthread_create(&baro_thread, NULL, acquireBarometerData, (void *)&baro))
	    {
	        printf("Error: Failed to create barometer thread\n");
	        return 0;
	    }
	imuLSM.initialize();
			if(pthread_create(&LSM_thread, NULL, acquireLSMData, (void *)&imuLSM))
				{
					printf("Error: Failed to create lsm thread\n");
						return 0;
			}
	imuMPU.initialize();
		if(pthread_create(&MPU_thread, NULL, acquireMPUData, (void *)&imuMPU))
		    {
		        printf("Error: Failed to create mpu thread\n");
		        return 0;
		    }
	led.initialize();
	if(pthread_create(&led_thread, NULL, acquireLedData, (void *)&led))
				{
					printf("Error: Failed to create led thread\n");
						return 0;
			}

    while(count<countMax) {
    	count++;
    	swBaro=0;
    	swMPU=0;
    	swLSM=0;
    	swLed=0;
    	gettimeofday(&tot1,NULL);
    	while((swBaro & swMPU & swLSM & swLed)!=1){
    		printf("Numero da leitura: %d \n", swBaro);
    		printf("Numero da leitura: %d \n", swMPU);
    		printf("Numero da leitura: %d \n", swLSM);
    		printf("Numero da leitura: %d \n", swLed);
    		usleep(1000000);
    	}
    	gettimeofday(&tot2,NULL);
    	dtTot=(1000000 * tot2.tv_sec + tot2.tv_usec)-1000000 * tot1.tv_sec - tot1.tv_usec ;

//----------------Obtencao do tempo antes da leitura dos sensores---------------------------------//

//----------------Escrita no PWM  ---------------------------------//

//----------------Leitura da IMU MPU ---------------------------------//

//----------------Leitura da IMU LSM---------------------------------//

//----------------Leitura do barometro ---------------------------------//



//----------------Obtencao do tempo apos leitura dos dados ---------------------------------//
        if(count!=1){
        	 mem=dtlong;
        }
    	dtlong= dtMPU + dtLSM + dtLED;
    	if(count==2){
    	    		min=dtlong;
    	    		max=dtlong;
    	    		mem=dtlong;
    	    		media=dtlong;
    	    		sum=dtlong;
    	    	}
    	else{
    		sum=sum+dtlong;
    		media=sum/count;
    		if(dtlong<min){
    			min=dtlong;
    		}
    		if(dtlong>max){
    			max=dtlong;
    		}
    	}
    	printf("Numero da leitura: %lu \n", count);
    	printf("Duracao media em microsegundos da leitura dos sensores: %lu \n", media);
    	printf("Duracao atual em microsegundos da leitura dos sensores: %lu \n", dtTot);

    	usleep(1000000);

           }

            printf("--------------------------------------------------------------------------------------------------\n");
                    	printf("Numero da leitura: %lu \n", count);
                    	//printf("Duracao minima microsegundos da leitura dos sensores: %lu \n", min);
                    	printf("Duracao em microsegundos da leitura atual do barometro: %lu \n", dtBaro);
                    	printf("Duracao em microsegundos da leitura atual da IMU MPU: %lu \n", dtMPU);
                    	printf("Duracao em microsegundos da leitura atual da IMU LSM: %lu \n", dtLSM);
                    	printf("Duracao em microsegundos da escrita PWM no LED: %lu \n", dtLED);
                    	printf("Duracao em microsegundos da leitura atual MPU LSM e escrita LED: %lu \n", dtlong);
                    	printf("Duracao media em microsegundos da leitura dos sensores: %lu \n", media);
                    	printf("Duracao minima microsegundos da leitura dos sensores: %lu \n", min);
                    	printf("Duracao maxima microsegundos da leitura dos sensores: %lu \n", max);
                    	time_t rawtime;
                    	struct tm * timeinfo;

                    	time ( &rawtime );
                    	timeinfo = localtime ( &rawtime );
                    	printf ( "Data e tempo local atual: %s", asctime (timeinfo) );
                    	printf("-----------------------------------Leitura da IMU MPU9250-----------------------------------------");
                    			printf("\n\nAcc: %+7.3f %+7.3f %+7.3f  ", ax, ay, az);
                    	        printf("Gyr: %+8.3f %+8.3f %+8.3f  ", gx, gy, gz);
                    	        printf("Mag: %+7.3f %+7.3f %+7.3f\n", mx, my, mz);
                    	printf("-----------------------------------Leitura da IMU LSM9DS1-----------------------------------------");
                    	        printf("\n\nAcc: %+7.3f %+7.3f %+7.3f  ", ax2, ay2, az2);
                    	        printf("Gyr: %+8.3f %+8.3f %+8.3f  ", gx2, gy2, gz2);
                    	        printf("Mag: %+7.3f %+7.3f %+7.3f\n", mx2, my2, mz2);
                    	printf("-----------------------------------Leitura do barometro-------------------------------------------");
                    	printf("\nTemperatura(C): %f Pressao (milibar): %f\n",
                    	                        temperatura, pressao);
    pthread_exit(NULL);
    pthread_exit(NULL);
    pthread_exit(NULL);
    pthread_exit(NULL);
           return 0;
       }

