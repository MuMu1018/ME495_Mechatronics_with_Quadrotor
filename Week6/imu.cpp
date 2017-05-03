#include <sys/shm.h>
#include <sys/stat.h>
#include <stdlib.h>
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
#define KI 0.02				// less than 0.05
#define frequency 25000000.0
#define LED0 0x6			//LED0 start register
#define LED0_ON_L 0x6		//LED0 output and brightness control byte 0
#define LED0_ON_H 0x7		//LED0 output and brightness control byte 1
#define LED0_OFF_L 0x8		//LED0 output and brightness control byte 2
#define LED0_OFF_H 0x9		//LED0 output and brightness control byte 3
#define LED_MULTIPLYER 4	// For the other 15 channels

struct data
{
    int a;
    int sequence_num;
    int alive_check;
};

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

int main ()
{
    //open file for writing to if needed
    FILE *f;
    f=fopen("data.csv","a+");
    execute=1;
    
    signal(SIGINT, &trap);
    struct timespec te;
    int address;
    int mag,imu;
    int display = 0;
    
    int segment_id;
    data* shared_memory;
    struct shmid_ds shmbuffer;
    int segment_size;
    const int shared_segment_size = 0x6400;
    int smhkey=33222;
    
    /* Allocate a shared memory segment.  */
    segment_id = shmget (smhkey, shared_segment_size,IPC_CREAT | 0666);
    /* Attach the shared memory segment.  */
    shared_memory = (data*) shmat (segment_id, 0, 0);
    printf ("shared memory attached at address %p\n", shared_memory);
    /* Determine the segment's size. */
    shmctl (segment_id, IPC_STAT, &shmbuffer);
    segment_size  =               shmbuffer.shm_segsz;
    printf ("segment size: %d\n", segment_size);
    /* Write a string to the shared memory segment.  */
    //sprintf (shared_memory, "test!!!!.");
    
    /* Print out the string from shared memory.  */
    int frame=0, Total_PWM = 255, i =0, PWM_value = 100, a_c = -1, count = 0;
    // i = 105; d = 100;
    //wiringPiSetupGpio();
    //pinMode(16,OUTPUT);
    
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
        float desired_roll = 0;
        float roll_velocity = 0;
        
        float P_term_roll = 0;
        float I_term_roll = 0;
        float error_roll = 0;
        
        float current_pitch = 0;
        float previous_pitch = 0;
        float desired_pitch = 0;
        float pitch_velocity = 0;

        float P_term_pitch = 0;
        float I_term_pitch = 0;
        float error_pitch = 0;
        
        int neutral_power = NEUTRAL;
        
        while(execute==1)
        {
            data s=*shared_memory;
            
            // check whether another program is running
            if (a_c == s.alive_check){
                if (count<100){
                    count++;
                }
                else{
                    break;
                }
            }
            else{
                a_c = s.alive_check;
                count = 0;
            }
            
            // read command and set PWM value
            if(s.sequence_num!=frame){
                printf("data received %d\n\r",s.a);
                if (s.a == 'a'){
                    desired_roll += 2.5;
                }
                else if (s.a == 'd'){
                    desired_roll -= 2.5;
                }
                else if (s.a == 'h'){
                    neutral_power += 25;
                }
                else if (s.a == 'n'){
                    neutral_power -= 25;
                }
                else if (s.a == 'x'){
                    desired_roll = 0;
                    desired_pitch = 0;
                }
                else if (s.a == 'z'){
                    neutral_power = NEUTRAL;
                }
                else if (s.a == 'w'){
                    desired_pitch += 2.5;
                }
                else if (s.a == 's'){
                    desired_pitch -= 2.5;
                }
                else if (s.a == ' '){
                    execute = 0;
                    printf("Quit!!!\r\n");
                    s.a = '-';
                    break;
                }
                frame=s.sequence_num;
            }
            
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
                if ((roll_filtered > 50) || (roll_filtered < -50) || (pitch_filtered > 50) || (pitch_filtered < -50) || (diff > 5E9))
                    execute = 0;
                else {
                    //roll
                    current_roll = roll_filtered;
                    roll_velocity = current_roll - previous_roll;
                    
                    error_roll = current_roll - desired_roll;
                    P_term_roll = error_roll*KP;
                    I_term_roll = I_term_roll + error_roll*KI;
                    
                    //pitch
                    current_pitch = pitch_filtered;
                    pitch_velocity= current_pitch - previous_pitch;
                    
                    error_pitch = current_pitch - desired_pitch;
                    P_term_pitch = error_pitch*KP;
                    I_term_pitch = I_term_pitch + error_pitch*KI;
                    
                    //printf("P_term = %f  I_term = %f\r\n",P_term,I_term);
                    
                    // I_term limitation
                    if (I_term_roll > 100)
                        I_term_roll = 100;
                    if (I_term_roll < -100)
                        I_term_roll = -100;
                    if (I_term_pitch > 100)
                        I_term_pitch = 100;
                    if (I_term_pitch < -100)
                        I_term_pitch = -100;
                    
                    // PID control
                    //int prop_PWM0 = neutral_power - 50 + P_term_roll + KD*roll_velocity + I_term_roll;
                    //int prop_PWM1 = neutral_power + 150 - P_term_roll - KD*roll_velocity - I_term_roll;
                    int prop_PWM0 = 1000;
                    int prop_PWM1 = 1000;
                    //int prop_PWM2 = 1300;
                    //int prop_PWM3 = 1300;
                    int prop_PWM2 = neutral_power - P_term_pitch - KD*pitch_velocity - I_term_pitch;
                    int prop_PWM3 = neutral_power + 50 + P_term_pitch + KD*pitch_velocity + I_term_pitch;
                    
                    // limits on motor duty cycle
                    if (prop_PWM0 > 1500)
                        prop_PWM0 = 1500;
                    if (prop_PWM0 < 1000)
                        prop_PWM0 = 1000;
                    if (prop_PWM1 > 1500)
                        prop_PWM1 = 1500;
                    if (prop_PWM1 < 1000)
                        prop_PWM1 = 1000;
                    if (prop_PWM2 > 1500)
                        prop_PWM2 = 1500;
                    if (prop_PWM2 < 1000)
                        prop_PWM2 = 1000;
                    if (prop_PWM3 > 1500)
                        prop_PWM3 = 1500;
                    if (prop_PWM3 < 1000)
                        prop_PWM3 = 1000;
                    
                    // set duty cycle
                    //printf("motor 0: %d		motor 1: %d\r\n", prop_PWM0, prop_PWM1);
                    set_PWM(pwm,0,prop_PWM0);
                    set_PWM(pwm,1,prop_PWM1);
                    set_PWM(pwm,2,prop_PWM2);
                    set_PWM(pwm,3,prop_PWM3);
                    
                    previous_roll = current_roll;
                    previous_pitch = current_pitch;
                }
                
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
            
            if(display%1==0)
            {
                fprintf(f,"%f,%f\n",current_roll,desired_roll);
                //printf("%f		%f\r\n", roll_filtered, I_term);
            }
            display++;
            //remember the current time
            time_prev=time_curr;
            
        }
        /* Detach the shared memory segment.  */
        shmdt (shared_memory);
        
        /* Deallocate the shared memory segment.  */
        // shmctl (segment_id, IPC_RMID, 0);
        
  	 }
    trap(0);
    return 0;
}
