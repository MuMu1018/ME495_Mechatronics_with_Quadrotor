#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <time.h>
#include <math.h>

int main (int argc, char *argv[])
{
    //open file for writing to if needed
    FILE *f_roll;
    FILE *f_pitch;
    FILE *f_roll_acc;
    FILE *f_pitch_acc;
    FILE *f_roll_gyro;
    FILE *f_pitch_gyro;
    f_pitch=fopen("data_pitch.txt","a+");
    f_pitch_acc=fopen("data_pitch_acc.txt","a+");
    f_pitch_gyro=fopen("data_pitch_gyro.txt","a+");
    f_roll=fopen("data_roll.txt","a+");
    f_roll_acc=fopen("data_roll_acc.txt","a+");
    f_roll_gyro=fopen("data_roll_gyro.txt","a+");

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
        float pitch_angle=0;
        float roll_angle=0;
        float yaw_angle=0;
        long time_curr;
        long time_prev;
        float delta_x_rotate=0;
        float delta_y_rotate=0;
        float delta_z_rotate=0;

        int add_x_rate=0x18;
        int add_y_rate=0x1A;
        int add_z_rate=0x1C;
        int add_x_acc=0x28;
        int add_y_acc=0x2A;
        int add_z_acc=0x2C;

        float gyro_angle[2]={0};
        float acc_angle[2]={0};
        while(1)
        {
            //read in gyro x,y,z rate and acceleration
            int x_rate,y_rate,z_rate;
            int x_acc,y_acc,z_acc;
            
            x_rate=wiringPiI2CReadReg16(imu,add_x_rate);
            y_rate=wiringPiI2CReadReg16(imu,add_y_rate);
            z_rate=wiringPiI2CReadReg16(imu,add_z_rate);
            x_acc=wiringPiI2CReadReg16(imu,add_x_acc);
            y_acc=wiringPiI2CReadReg16(imu,add_y_acc);
            z_acc=wiringPiI2CReadReg16(imu,add_z_acc);

            // convert to 2's complement
            if(x_rate>0x8000){
                x_rate=x_rate ^ 0xffff;
                x_rate=-x_rate-1;
            }
            if(y_rate>0x8000){
                y_rate=y_rate ^ 0xffff;
                y_rate=-y_rate-1;
            }
            if(z_rate>0x8000){
                z_rate=z_rate ^ 0xffff;
                z_rate=-z_rate-1;
            }
            if(x_acc>0x8000){
                x_acc=x_acc ^ 0xffff;
                x_acc=-x_acc-1;
            }
            if(y_acc>0x8000){
                y_acc=y_acc ^ 0xffff;
                y_acc=-y_acc-1;
            }
            if(z_acc>0x8000){
                z_acc=z_acc ^ 0xffff;
                z_acc=-z_acc-1;
            }
            //calibration value
            x_rate+=93; // -78
            y_rate-=79;  // +78
            z_rate-=321; // +312

            //convert to dps
            float x_angle_dps=x_rate*245.0/32767.0;
            float y_angle_dps=y_rate*245.0/32767.0;
            float z_angle_dps=z_rate*245.0/32767.0;

            //get current time in nanoseconds
            timespec_get(&te,TIME_UTC);
            time_curr=te.tv_nsec;

            //compute time since last execution
            float diff=time_curr-time_prev;

            //check for rollover
            if(diff<=0){
                diff+=1000000000;
            }

            //convert to seconds
            diff=diff/1000000000;

            //compute amount of rotation since last execution
            delta_x_rotate=x_angle_dps*diff;
            delta_y_rotate=y_angle_dps*diff;
            delta_z_rotate=z_angle_dps*diff;

            //update roll and pitch from gyro
            gyro_angle[0]+=delta_x_rotate;
            gyro_angle[1]+=delta_y_rotate;

            //update roll and pitch from acc
            acc_angle[0]=atan2(float(-y_acc),float(z_acc))*180.0/3.1415926;
            acc_angle[1]=atan2(float(x_acc),float(z_acc))*180.0/3.1415926;
            //compute yaw
            //roll_angle+= delta_x_rotate;
            //pitch_angle+= delta_y_rotate;
            float A=0.8;
            roll_angle = A*acc_angle[0]+(1-A)*gyro_angle[0];
            pitch_angle = A*acc_angle[1]+(1-A)*gyro_angle[1];
            yaw_angle+= delta_z_rotate;

            //print out the value
            printf("========================================================================\r\n");
            printf("x_acc = %d\r\n", x_acc);
            printf("y_acc = %d\r\n", y_acc);
            printf("z_acc = %d\r\n", z_acc);
            printf("------------------------------------------------------------------------\r\n");
            printf("roll angle acc %f \r\n",acc_angle[0]);
            printf("pitch angle acc %f \r\n",acc_angle[1]);
            printf("------------------------------------------------------------------------\r\n");
            printf("delta x = %f , roll angle %f \r\n",delta_x_rotate, roll_angle);
            printf("delta y = %f , pitch angle %f \r\n",delta_y_rotate, pitch_angle);
            printf("delta z = %f , yaw angle %f \r\n",delta_z_rotate, yaw_angle);
            printf("========================================================================\r\n");

            // //print to file if desired
            fprintf(f_roll,"%f ",roll_angle);
            fprintf(f_roll_acc,"%f ",acc_angle[0]);
            fprintf(f_roll_gyro,"%f ",gyro_angle[0]);
            fprintf(f_pitch,"%f ",pitch_angle);
            fprintf(f_pitch_acc,"%f ",acc_angle[1]);
            fprintf(f_pitch_gyro,"%f ",gyro_angle[1]);
            //fprintf(f, "%f %f %f %f %f %f\n\r",roll_angle,pitch_angle,acc_angle[0],acc_angle[1],gyro_angle[0],gyro_angle[1]);
            //fprintf(f, "yaw %f\n\r",yaw_angle);

            //remember the current time
            time_prev=time_curr;
        }

  	 }
    return 0;
}
