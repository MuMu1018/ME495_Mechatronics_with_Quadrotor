#include <stdio.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <wiringPi.h>
struct data
{
    int a;
    int sequence_num;
    int alive_check;
};

int main ()
{
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
    wiringPiSetupGpio();
    pinMode(16,OUTPUT);
    while(1)
    {
        data s=*shared_memory;
        
        // check whether another program is running
        if (a_c == s.alive_check){
            if (count<1000){
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
        
        // set PWM_value limitation
        if (PWM_value > 255){
            PWM_value = 255;
            printf("Max LED brightness!\n\r");
        }
        else if (PWM_value < 0){
            PWM_value = 0;
            printf("LED turn off!\n\r");
        }
        
        // use PWM control LED brightness
        for(i = 0; i < PWM_value; i++){
            digitalWrite(16,HIGH);
        }
        //delay(500);
        for (i = PWM_value; i< Total_PWM; i++){
            digitalWrite(16,LOW);
        }
        //delay(500);
        
        // read command and set PWM value
        if(s.sequence_num!=frame){
            printf("data received %d\n\r",s.a);
            if (s.a == 'i'){
                PWM_value = PWM_value + 25;
            }
            else if (s.a == 'd'){
                PWM_value = PWM_value - 25;
            }
            else if (s.a == 'q'){
                digitalWrite(16,LOW);
                printf("Quit!\r\n");
                break;
            }
            frame=s.sequence_num;
        }
        
    }
    /* Detach the shared memory segment.  */
    shmdt (shared_memory);
    
    /* Deallocate the shared memory segment.  */
    // shmctl (segment_id, IPC_RMID, 0);
    
    return 0;
}
