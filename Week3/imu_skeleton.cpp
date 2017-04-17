#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <time.h>
#include <math.h>

int main (int argc, char *argv[])
{
    //open file for writing to if needed
    FILE *f_data;
    f_data=fopen("data.csv","w+");

    struct timespec te;
    int address;
    int mag,imu;
    int data;
    int hex_value;

    wiringPiSetup();

    //setup imu on I2C
    imu=wiringPiI2CSetup(0x6B) ; //accel/gyro address
    hex_value = wiringPiI2CReadReg8(imu,0x0F); //read who am i register
    printf(" 0x%x \r\n",hex_value);

    if(imu==-1)
    {
        printf("cant connect to I2C device\n");
        return -1;
    }
    else
    {
        //turn on imu by writing 0xC0 to address 0x10 and 0x20
        wiringPiI2CWriteReg8(imu,0x20,0xC0); //turn on and set accell output
        wiringPiI2CWriteReg8(imu,0x10,0xC0); //turn on and set gyro output

        float pry_angle[3]={0};
        float delta_pry_angle[3]={0};

        float gyro_angle[2]={0};
        float acc_angle[2]={0};

        long time_curr;
        long time_prev;

        int add_gyro[3]={0x18,0x1A,0x1C};
        int add_acc[3]={0x28,0x2A,0x2C};

        int gyro_calibration[3]={116,-85,-294};

/*
 *        // get iniital pitch and roll angle
 *        float pr_ini[2]={0};
 *        for(int i=0;i<10;i++)
 *        {
 *          int acc_data[3]={0};
 *          for(int i=0;i<10;i++)
 *          {
 *            acc_data[i]=wiringPiI2CReadReg16(imu,add_acc[i]);
 *            if(acc_data[i]>0x8000)
 *            {
 *                acc_data[i]=acc_data[i] ^ 0xffff;
 *                acc_data[i]=-acc_data[i]-1;
 *            }
 *
 *            pr_ini[0]+=atan2(float(-acc_data[1]),float(acc_data[2]))*180.0/3.1415926;
 *            pr_ini[1]=atan2(float(acc_data[0]),float(acc_data[2]))*180.0/3.1415926;
 *          }
 *        }
 *        pry_angle[0]=pr_ini[0]/10;
 *        pry_angle[1]=pr_ini[1]/10;
 */

        while(1)
        {
            //read in gyro z rate
            //address=0x1C;
            int gyro_data[3]={0};
            int acc_data[3]={0};

            for(int i=0;i<3;i++)
            {
              gyro_data[i]=wiringPiI2CReadReg16(imu,add_gyro[i]);
              if(gyro_data[i]>0x8000)
              {
                  gyro_data[i]=gyro_data[i] ^ 0xffff;
                  gyro_data[i]=-gyro_data[i]-1;
              }
              gyro_data[i]+=gyro_calibration[i];

              acc_data[i]=wiringPiI2CReadReg16(imu,add_acc[i]);
              if(acc_data[i]>0x8000)
              {
                  acc_data[i]=acc_data[i] ^ 0xffff;
                  acc_data[i]=-acc_data[i]-1;
              }
            }

            //get current time in nanoseconds
            timespec_get(&te,TIME_UTC);

            //remember the current time
            time_curr=te.tv_nsec;

            //compute time since last execution
            float diff=time_curr-time_prev;

            //update time_prev
            time_prev=time_curr;

            //check for rollover
            if(diff<=0){
                diff+=1000000000;
            }

            //convert to seconds
            diff=diff/1000000000;

            //convert to dps
            float angle_dps[3]={0};

            for(int i=0;i<3;i++)
            {
              angle_dps[i]=gyro_data[i]*245.0/32767.0;
              delta_pry_angle[i]=angle_dps[i]*diff;
              pry_angle[i]+=delta_pry_angle[i];
            }

            //update roll and pitch from gyro
            gyro_angle[0]+=delta_pry_angle[0];
            gyro_angle[1]+=delta_pry_angle[1];

            //update roll and pitch from acc
            acc_angle[0]=atan2(float(-acc_data[1]),float(acc_data[2]))*180.0/3.1415926;
            acc_angle[1]=atan2(float(acc_data[0]),float(acc_data[2]))*180.0/3.1415926;

            float A=0.02;
            pry_angle[0]= A*acc_angle[0]+(1-A)*pry_angle[0];
            pry_angle[1]= A*acc_angle[1]+(1-A)*pry_angle[1];

            //print out the value
            printf("========================================================================\r\n");
            printf("roll angle acc %f \r\n",acc_angle[0]);
            printf("pitch angle acc %f \r\n",acc_angle[1]);
            printf("------------------------------------------------------------------------\r\n");
            printf("roll angle gyro %f \r\n",gyro_angle[0]);
            printf("pitch angle gyro %f \r\n",gyro_angle[1]);
            printf("------------------------------------------------------------------------\r\n");
            printf("delta x = %f , roll angle %f \r\n", delta_pry_angle[0], pry_angle[0]);
            printf("delta y = %f , pitch angle %f \r\n",delta_pry_angle[1], pry_angle[1]);
            printf("delta z = %f , yaw angle %f \r\n",  delta_pry_angle[2], pry_angle[2]);
            printf("========================================================================\r\n");

            //print to file if desired
            fprintf(f_data,"%f,%f,%f,%f,%f,%f\n",pry_angle[0],gyro_angle[0],acc_angle[0],pry_angle[1],gyro_angle[1],acc_angle[1]);
        }

  	 }
    return 0;
}
