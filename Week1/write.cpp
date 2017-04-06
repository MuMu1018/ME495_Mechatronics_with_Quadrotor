#include <curses.h>
#include <stdio.h>
#include <sys/shm.h>
#include <sys/stat.h>


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
    
    int ch = 0;
    
    /* Basic initialization of curses lib */
    initscr();
    cbreak();
    noecho(); /* Set this for interactive programs. */
    nonl();
    intrflush(stdscr, FALSE);
    keypad(stdscr, TRUE);
    
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
    int i=0, b = 0;
    
    nodelay(stdscr,TRUE);
    while(1)
    {
        /* Ready to rumble! */
        printf("Type a key: (Use 'q' to quit)\r\n");
        printf("Type 'i' to increase. Type 'd' to decrease.\r\n");
        while (ch != 'q') {
            ch = getch();
            if(ch!=-1){
                printf("You typed: '%c' which is ASCII %d\n\r", (char) ch, ch);
                shared_memory->a=ch;
                shared_memory->sequence_num=i++;
            }
            shared_memory->alive_check = b++;
        }
        break;
    }
    
    
    /* Detach the shared memory segment.  */
    shmdt (shared_memory);
    
    /* Deallocate the shared memory segment.  */ 
    // shmctl (segment_id, IPC_RMID, 0); 
    endwin(); 
    
    return 0; 
} 
