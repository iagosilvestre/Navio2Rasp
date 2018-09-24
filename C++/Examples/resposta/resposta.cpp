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

#define G_SI 9.80665
#define PI   3.14159

	    float ax, ay, az;
	    float gx, gy, gz;
	    float mx, my, mz;
	    float ax2, ay2, az2;
	    float gx2, gy2, gz2;
	    float mx2, my2, mz2;

		struct timeval baro1,baro2,mpu1,mpu2,lsm1,lsm2,led1,led2;
		float dt;
		unsigned long int dtlong=0,count=0,min=0,max=0,mem=0,media=0,sum=0,dtMPU=0,dtLSM=0,dtLED=0,dtBaro=0;

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
    MS5611* barometer = (MS5611*)barom;
    while (true) {
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
        //sleep(0.5);
    }

    pthread_exit(NULL);
}
void * acquireMPUData(void * imuMPU)
{
	MPU9250* mpu=(MPU9250*)imuMPU;
	while(true){
    	gettimeofday(&mpu1,NULL);
		mpu->update();
		mpu->read_accelerometer(&ax, &ay, &az);
		mpu->read_gyroscope(&gx, &gy, &gz);
		mpu->read_magnetometer(&mx, &my, &mz);
		gettimeofday(&mpu2,NULL);
		dtMPU=(1000000 * mpu2.tv_sec + mpu2.tv_usec)-1000000 * mpu1.tv_sec - mpu1.tv_usec ;
	}
	pthread_exit(NULL);
}
void * acquireLSMData(void * imuLSM)
{
	LSM9DS1* lsm=(LSM9DS1*)imuLSM;
	while(true){
		gettimeofday(&lsm1,NULL);
		lsm->update();
		lsm->read_accelerometer(&ax2, &ay2, &az2);
		lsm->read_gyroscope(&gx2, &gy2, &gz2);
		lsm->read_magnetometer(&mx2, &my2, &mz2);
		gettimeofday(&lsm2,NULL);
		dtLSM=(1000000 * lsm2.tv_sec + lsm2.tv_usec)-1000000 * lsm1.tv_sec - lsm1.tv_usec ;
	}
	pthread_exit(NULL);
}

void * acquireLedData(void * led)
{
	Led_Navio2* diode=(Led_Navio2*)led;
	while(true){
		gettimeofday(&led1,NULL);
    	if((count%2)==0){
    		diode->setColor(Colors::Red);
    	}
    	else
    		diode->setColor(Colors::Green);
		gettimeofday(&led2,NULL);
		dtLED=(1000000 * led2.tv_sec + led2.tv_usec)-1000000 * led1.tv_sec - led1.tv_usec ;
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

	if (check_apm()) {
	        return 1;
	    }
	/*auto led = get_led();
	if (!led->initialize())
	        return EXIT_FAILURE;*/
	Led_Navio2 led;
	MS5611 baro;
	MPU9250 imuMPU;
	LSM9DS1 imuLSM;

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
	imuMPU.initialize();
		if(pthread_create(&MPU_thread, NULL, acquireMPUData, (void *)&imuMPU))
		    {
		        printf("Error: Failed to create mpu thread\n");
		        return 0;
		    }

	imuLSM.initialize();
		if(pthread_create(&LSM_thread, NULL, acquireLSMData, (void *)&imuLSM))
			{
				printf("Error: Failed to create lsm thread\n");
					return 0;
		}
	led.initialize();
	if(pthread_create(&led_thread, NULL, acquireLedData, (void *)&led))
				{
					printf("Error: Failed to create led thread\n");
						return 0;
			}
	std::vector<double> pos_data;
	Ublox gps;


//-------------------------------------------------------------------------


    if(gps.testConnection())
        {
            printf("Ublox test OK\n");
            if (!gps.configureSolutionRate(1000))
            {
                printf("Setting new rate: FAILED\n");
            }
    while(1) {
    	count++;
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
    	/*if(count==1){
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
    	}*/


        /*if (gps.decodeSingleMessage(Ublox::NAV_POSLLH, pos_data) == 1)
                   {*/
                       // after desired message is successfully decoded, we can use the information stored in pos_data vector
                       // right here, or we can do something with it from inside decodeSingleMessage() function(see ublox.h).
                       // the way, data is stored in pos_data vector is specified in decodeMessage() function of class UBXParser(see ublox.h)
        	printf("--------------------------------------------------------------------------------------------------\n");
        	printf("Numero da leitura: %lu \n", count);
        	//printf("Duracao minima microsegundos da leitura dos sensores: %lu \n", min);
        	printf("Duracao em microsegundos da leitura dos sensores: %lu \n", dtMPU);
        	printf("Duracao em microsegundos da leitura dos sensores: %lu \n", dtLSM);
        	printf("Duracao em microsegundos da leitura dos sensores: %lu \n", dtLED);
        	//printf("Duracao media em microsegundos da leitura dos sensores: %lu \n", media);
        	//printf("Duracao maxima microsegundos da leitura dos sensores: %lu \n", max);
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
        	/*printf("-----------------------------------Leitura do GPS-------------------------------------------------");
        	        printf("\nGPS Millisecond Time of Week: %.0lf s\n", pos_data[0]/1000);
                    printf("Longitude: %lf\n", pos_data[1]/10000000);
                    printf("Latitude: %lf\n", pos_data[2]/10000000);
                    printf("Height above Ellipsoid: %.3lf m\n", pos_data[3]/1000);
                    printf("Height above mean sea level: %.3lf m\n", pos_data[4]/1000);
                    printf("Horizontal Accuracy Estateimate: %.3lf m\n", pos_data[5]/1000);
                    printf("Vertical Accuracy Estateimate: %.3lf m\n", pos_data[6]/1000);
                    printf("--------------------------------------------------------------------------------------------------\n");
                   } else {
                	   printf("Numero da leitura: %lu \n", count);
                	  // printf("Duracao minima microsegundos da leitura dos sensores: %lu \n", min);
                	   printf("Duracao em microsegundos da leitura dos sensores: %lu \n", dtMPU);
                	   printf("Duracao em microsegundos da leitura dos sensores: %lu \n", dtLSM);
                	   printf("Duracao em microsegundos da leitura dos sensores: %lu \n", dtLED);
                	   //printf("Duracao media em microsegundos da leitura dos sensores: %lu \n", media);
                	   //printf("Duracao maxima microsegundos da leitura dos sensores: %lu \n", max);
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
                       // printf("Message not captured\n");
                       // use this to see, how often you get the right messages
                       // to increase the frequency you can turn off the undesired messages or tweak ublox settings
                       // to increase internal receiver frequency
                   }*/

                   /*if (gps.decodeSingleMessage(Ublox::NAV_STATUS, pos_data) == 1)
                   {
                       printf("Current GPS status:\n");
                       printf("gpsFixOk: %d\n", ((int)pos_data[1] & 0x01));

                       printf("gps Fix status: ");
                       switch((int)pos_data[0]){
                           case 0x00:
                               printf("no fix\n");
                               break;

                           case 0x01:
                               printf("dead reckoning only\n");
                               break;

                           case 0x02:
                               printf("2D-fix\n");
                               break;

                           case 0x03:
                               printf("3D-fix\n");
                               break;

                           case 0x04:
                               printf("GPS + dead reckoning combined\n");
                               break;

                           case 0x05:
                               printf("Time only fix\n");
                               break;

                           default:
                               printf("Reserved value. Current state unknown\n");
                               break;

                       }

                       printf("\n");

                   } else {
                       // printf("Status Message not captured\n");
                   }*/


                   usleep(100000);
               //}

           }
    pthread_exit(NULL);
           return 0;
       }
}

