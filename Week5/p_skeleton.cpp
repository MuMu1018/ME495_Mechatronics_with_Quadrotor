#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <time.h>
#include <math.h>
#include <stdint.h>
#include <signal.h>

#define A 0.002 // complementary filter constant
#define NEUTRAL 1300
#define KP 3.6				// 0 to 15	1.8
#define KD 200				// 10 to 100	100
#define KI 0.005				// less than 0.05
#define frequency 25000000.0
#define LED0 0x6			//LED0 start register
#define LED0_ON_L 0x6		//LED0 output and brightness control byte 0
#define LED0_ON_H 0x7		//LED0 output and brightness control byte 1
#define LED0_OFF_L 0x8		//LED0 output and brightness control byte 2
#define LED0_OFF_H 0x9		//LED0 output and brightness control byte 3
#define LED_MULTIPLYER 4	// For the other 15 channels
int execute,pwm;

//init the pwm board
void init_pwm(int pwm)
{
    float freq =400.0*.95;
    float prescaleval = 25000000;
    prescaleval /= 4096;
    prescaleval /= freq;
    prescaleval -= 1;
    uint8_t prescale = floor(prescaleval+0.5);
    int settings = wiringPiI2CReadReg8(pwm, 0x00) & 0x7F;
    int sleep	= settings | 0x10;
    int wake 	= settings & 0xef;
    int restart = wake | 0x80;
    wiringPiI2CWriteReg8(pwm, 0x00, sleep);
    wiringPiI2CWriteReg8(pwm, 0xfe, prescale);
    wiringPiI2CWriteReg8(pwm, 0x00, wake);
    delay(10);
    wiringPiI2CWriteReg8(pwm, 0x00, restart|0x20);
}

//turn on the motor
void init_motor(int pwm,uint8_t channel)
{
    int on_value=0;
    
    int time_on_us=900;
    uint16_t off_value=round((time_on_us*4096.f)/(1000000.f/400.0));
    
    wiringPiI2CWriteReg8(pwm, LED0_ON_L + LED_MULTIPLYER * channel, on_value & 0xFF);
    wiringPiI2CWriteReg8(pwm, LED0_ON_H + LED_MULTIPLYER * channel, on_value >> 8);
    wiringPiI2CWriteReg8(pwm, LED0_OFF_L + LED_MULTIPLYER * channel, off_value & 0xFF);
    wiringPiI2CWriteReg8(pwm, LED0_OFF_H + LED_MULTIPLYER * channel, off_value >> 8);
    delay(100);
    
    time_on_us=1200;
    off_value=round((time_on_us*4096.f)/(1000000.f/400.0));
    
    wiringPiI2CWriteReg8(pwm, LED0_ON_L + LED_MULTIPLYER * channel, on_value & 0xFF);
    wiringPiI2CWriteReg8(pwm, LED0_ON_H + LED_MULTIPLYER * channel, on_value >> 8);
    wiringPiI2CWriteReg8(pwm, LED0_OFF_L + LED_MULTIPLYER * channel, off_value & 0xFF);
    wiringPiI2CWriteReg8(pwm, LED0_OFF_H + LED_MULTIPLYER * channel, off_value >> 8);
    delay(100);
    
    time_on_us=1000;
    off_value=round((time_on_us*4096.f)/(1000000.f/400.0));
    
    wiringPiI2CWriteReg8(pwm, LED0_ON_L + LED_MULTIPLYER * channel, on_value & 0xFF);
    wiringPiI2CWriteReg8(pwm, LED0_ON_H + LED_MULTIPLYER * channel, on_value >> 8);
    wiringPiI2CWriteReg8(pwm, LED0_OFF_L + LED_MULTIPLYER * channel, off_value & 0xFF);
    wiringPiI2CWriteReg8(pwm, LED0_OFF_H + LED_MULTIPLYER * channel, off_value >> 8);
    delay(100);
    
}

//set the pwm value of the motor
void set_PWM(int pwm, uint8_t channel, float time_on_us)
{
    uint16_t off_value=round((time_on_us*4096.f)/(1000000.f/400.0));
    wiringPiI2CWriteReg16(pwm, LED0_OFF_L + LED_MULTIPLYER * channel,off_value);
    
}

//when cntrl+c pressed, kill motors
void kill_motor(int pwm,uint8_t channel)
{
    int on_value=0;
    
    int time_on_us=1000;
    uint16_t off_value=round((time_on_us*4096.f)/(1000000.f/400.0));
    
    wiringPiI2CWriteReg8(pwm, LED0_ON_L + LED_MULTIPLYER * channel, on_value & 0xFF);
    wiringPiI2CWriteReg8(pwm, LED0_ON_H + LED_MULTIPLYER * channel, on_value >> 8);
    wiringPiI2CWriteReg8(pwm, LED0_OFF_L + LED_MULTIPLYER * channel, off_value & 0xFF);
    wiringPiI2CWriteReg8(pwm, LED0_OFF_H + LED_MULTIPLYER * channel, off_value >> 8);
    
}

void trap(int signal)
{
    delay(10);
    set_PWM(pwm,0,1000);
    delay(10);
    set_PWM(pwm,1,1000);
    delay(10);
    set_PWM(pwm,2,1000);
    delay(10);
    set_PWM(pwm,3,1000);
    delay(100);
    delay(10);
    set_PWM(pwm,0,1000);
    delay(100);
    set_PWM(pwm,1,1000);
    delay(100);
    set_PWM(pwm,2,1000);
    delay(100);
    set_PWM(pwm,3,1000);
    delay(100);
    
    kill_motor(pwm,0);
    delay(100);
    kill_motor(pwm,1);
    delay(100);
    kill_motor(pwm,2);
    delay(100);
    kill_motor(pwm,3);
    delay(100);
    
    execute=0;
    printf("ending program\n\r");
}

int main (int argc, char *argv[])
{
    //open file for writing to if needed
    FILE *f;
    f=fopen("data.csv","a+");
    execute=1;
    
    signal(SIGINT, &trap);
    struct timespec te;
    int address;
    int mag,imu;
    int data;
    int display = 0;
    
    wiringPiSetup();
    
    //setup for pwm
    pwm=wiringPiI2CSetup(0x40);  //connect pwm board to imu
    
    imu=wiringPiI2CSetup(0x6B) ; //accel/gyro address
    
    if(imu==-1||pwm==-1)
    {
        printf("cant connect to I2C device %d %d\n",imu,pwm);
        return -1;
    }
    else
    {
        //turn on imu by writing 0xC0 to address 0x10 and 0x20
        wiringPiI2CWriteReg8(imu,0x20,0xC0); //turn on and set accell output
        wiringPiI2CWriteReg8(imu,0x10,0xC0);  //turn on and set gyro output
        
        //start the pwm
        init_pwm(pwm);
        
        //init motor 0 and 1
        init_motor(pwm,0);
        init_motor(pwm,1);
        
        //declarations and initializations for imu
        float pitch_angle=0;
        float roll_angle=0;
        float yaw_angle=0;
        float roll_angle_accel=0;
        float pitch_angle_accel=0;
        long time_curr;
        long time_prev;
        float delta_x_rotate=0;
        float delta_y_rotate=0;
        float delta_z_rotate=0;
        float z_rate_average = 0;
        float roll_filtered = 0;
        float pitch_filtered = 0;
        
        float current_roll = 0;
        float previous_roll = 0;
        float roll_velocity = 0;
        float I_term = 0;
        float error = 0;
        
    /*   //yaw calibration
         int num_samples = 0;
         while (num_samples < 10000) {
         address=0x1C;
         int z_rate;
         z_rate=wiringPiI2CReadReg16(imu,address);
         //convert to 2's complement
         if(z_rate>0x8000)
         {
         z_rate=z_rate ^ 0xffff;
         z_rate=-z_rate-1;
         }
         //increment number of samples
         num_samples++;
         //moving average
         z_rate_average = ((num_samples-1.0)/num_samples)*z_rate_average + (1.0/num_samples)*z_rate;
         }
         printf("yaw_rate average is %f\n\r",z_rate_average); 
    */
        
        while(execute==1)
        {
            //read in gyro
            int x_rate=wiringPiI2CReadReg16(imu,0x18);
            int y_rate=wiringPiI2CReadReg16(imu,0x1A);
            int z_rate=wiringPiI2CReadReg16(imu,0x1C);
            //read in accelerometer
            int x_accel=wiringPiI2CReadReg16(imu,0x28);
            int y_accel=wiringPiI2CReadReg16(imu,0x2A);
            int z_accel=wiringPiI2CReadReg16(imu,0x2C);
            //convert all to 2's complement
            if(x_rate>0x8000)
            {
                x_rate=x_rate ^ 0xffff;
                x_rate=-x_rate-1;
                
            }
            if(y_rate>0x8000)
            {
                y_rate=y_rate ^ 0xffff;
                y_rate=-y_rate-1;
                
            }
            if(z_rate>0x8000)
            {
                z_rate=z_rate ^ 0xffff;
                z_rate=-z_rate-1;
                
            }
            if(x_accel>0x8000)
            {
                x_accel=x_accel ^ 0xffff;
                x_accel=-x_accel-1;
                
            }
            if(y_accel>0x8000)
            {
                y_accel=y_accel ^ 0xffff;
                y_accel=-y_accel-1;
                
            }
            if(z_accel>0x8000)
            {
                z_accel=z_accel ^ 0xffff;
                z_accel=-z_accel-1;
                
            }
            // convert accelerometer to g's
            // this was done for debugging purposes, but not necessary for atan2 input
            float x = x_accel*(2.0/32757.0);
            float y = y_accel*(2.0/32757.0);
            float z = z_accel*(2.0/32757.0);
            
            // yaw calibration value
            z_rate-=z_rate_average;
            // printf("%d\n",z_rate);
            // convert gyro data to dps
            float x_angle_dps = (float) x_rate*(245.0/32767.0);
            float y_angle_dps = (float) y_rate*(245.0/32767.0);
            float z_angle_dps = (float) z_rate*(245.0/32767.0);
            // convert linear accel data to roll and pitch angles in degrees
            roll_angle_accel = -180.0/3.14*atan2(y_accel,z_accel); // negative so in same direction as gyro
            pitch_angle_accel = 180.0/3.14*atan2(x_accel,z_accel);
            
            //get current time in nanoseconds
            timespec_get(&te,TIME_UTC);
            time_curr=te.tv_nsec;
            
            //compute time since last execution
            float diff=time_curr-time_prev;
            //check for rollover
            if(diff<=0)
            {
                diff+=1000000000;
            }
            //convert to seconds
            diff=diff/1000000000;
            
            //compute amount of rotation since last execution
            delta_x_rotate=x_angle_dps*diff;
            delta_y_rotate=y_angle_dps*diff;
            delta_z_rotate=z_angle_dps*diff;
            
            //compute cumulative yaw, pitch, roll
            roll_angle += delta_x_rotate;
            pitch_angle += delta_y_rotate;
            yaw_angle += delta_z_rotate;
            
            // complementary filter for roll and pitch
            roll_filtered = roll_angle_accel*A + (1-A)*(delta_x_rotate + roll_filtered);
            pitch_filtered = pitch_angle_accel*A + (1-A)*(delta_y_rotate + pitch_filtered);
            
            // turn of motors if cntrl+c pressed
            signal(SIGINT, &trap);
            
            if(execute==1)//if cntrl+c has not been pressed
            {
                // safety checks for roll and pitch angle, loop timing
                if ((roll_filtered > 45) || (roll_filtered < -45) || (pitch_filtered > 45) || (pitch_filtered < -45) || (diff > 5E9))
                    execute = 0;
                else {
                    current_roll = roll_filtered;
                    roll_velocity = current_roll - previous_roll;
                    
                    error = current_roll;
                    I_term = I_term + error*KI;
                    if (I_term > 150)
                        I_term = 150;
                    if (I_term < -150)
                        I_term = -150;
                    // PD control
                    int prop_PWM0 = NEUTRAL + KP*roll_filtered + KD*roll_velocity + I_term;
                    int prop_PWM1 = NEUTRAL - KP*roll_filtered - KD*roll_velocity - I_term;
                    // limits on motor duty cycle
                    if (prop_PWM0 > 1400)
                        prop_PWM0 = 1400;
                    if (prop_PWM0 < 1000)
                        prop_PWM0 = 1000;
                    if (prop_PWM1 > 1400)
                        prop_PWM1 = 1400;
                    if (prop_PWM1 < 1000)
                        prop_PWM1 = 1000;
                    // set duty cycle
                    //printf("motor 0: %d		motor 1: %d\r\n", prop_PWM0, prop_PWM1);
                    set_PWM(pwm,0,prop_PWM0);
                    set_PWM(pwm,1,prop_PWM1);
                    
                    previous_roll = current_roll;
                }
                
                //set motors
                //set_PWM(pwm,0,1200);
                //set_PWM(pwm,1,1200);
                
            }
            else if(execute==0)//put at end of main loop
            {
                set_PWM(pwm,0,1000);
                delay(100);
                set_PWM(pwm,1,1000);
                delay(100);
                set_PWM(pwm,2,1000);
                delay(100);
                set_PWM(pwm,3,1000);
                delay(100);
                printf("motors set to 1000 for exit\n\r");
            }
            
            if(display%200==0)
            {
                fprintf(f, "%f\n", current_roll);
                //printf("%f		%f\r\n", roll_filtered, I_term);
            }
            display++;
            //remember the current time
            time_prev=time_curr;
        }
        
  	 }
    trap(0);
    return 0;
}
