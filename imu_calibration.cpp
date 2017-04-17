#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <time.h>
#include <math.h>

int main (int argc, char *argv[])
{

  //open file for writing to if needed
	FILE *f;
	f=fopen("data.txt","a+");



  struct timespec te;
  int address;
  int mag,imu;
  int data;


  //initialize wringPI
  wiringPiSetup () ;


  //setup imu on I2C, the accel/gyro address is 0x68
  imu=wiringPiI2CSetup (0x6B) ;
  //read who am i register to test if I2C works
  int who_am_I;
  //who_am_I should be the accel/gyro address, i.e. 0x68
  who_am_I = wiringPiI2CReadReg8(imu,0x0f);
  printf("who am I is 0x%x\n",who_am_I);

  if(imu==-1)
  {
    printf("cant connect to I2C device\n");
    return -1;
  }
  else
  {
    //turn on imu by writing 0xC0 to address 0x10 and 0x20
    //turn on and set accell output
    wiringPiI2CWriteReg8(imu,0x20,0xc0);
    //turn on and set gyro output
    wiringPiI2CWriteReg8(imu,0x10,0xc0);

    //count records the number of readings for gyro z rate
    int count = 0;
    //sum records the total sum of all readings for gyro z rate
    int gyro_sum[3] = {0};
    int add_gyro[3]={0x18,0x1A,0x1C};
    //int add_x_acc=0x28;
    //int add_y_acc=0x2A;
    //int add_z_acc=0x2C;
    while(1)
    {
      //read in gyro z rate
      //the address to read gyro z rate is ox1c
      address=0x1c;
      int gyro_rate[3]={0};
      for(int i=0;i<3;i++)
      {
        gyro_rate[i]=wiringPiI2CReadReg16(imu,add_gyro[i]);
        if(gyro_rate[i]>0x8000)
        {
            gyro_rate[i]=gyro_rate[i] ^ 0xffff;
            gyro_rate[i]=-gyro_rate[i]-1;
        }
        gyro_sum[i]+=gyro_rate[i];
      }
      count++;

      //print out the guess for the drift of gyro z rate by taking average of all readings
      printf("===================================================\n");
      printf("%d   x_rate: %f\n", count, gyro_sum[0]/double(count));
      printf("%d   y_rate: %f\n", count, gyro_sum[1]/double(count));
      printf("%d   z_rate: %f\n", count, gyro_sum[2]/double(count));

      if(count>10000)
        break;
    }
  }
  return 0;
}
