#include <sys/types.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <err.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <libserialport.h>

#include <libsbp/sbp.h>
#include <libsbp/system.h>
#include <libsbp/observation.h>
#include <libsbp/navigation.h>
#include <libsbp/imu.h>
#include <modbus/modbus.h>
#include <modbus/modbus-tcp.h>

#include <errno.h>
#include "sbp_callback_functions.h"

#ifndef MODBUS_SET_INT32_TO_INT16
# define MODBUS_SET_INT32_TO_INT16(tab_int16, index, value) \
    do { \
        tab_int16[(index)    ] = (value) >> 16; \
        tab_int16[(index) + 1] = (value); \
    } while (0)
#endif

typedef struct {
  // see definition of SBP protocol at https://github.com/swift-nav/libsbp/blob/master/docs/sbp.pdf

  // populated from SBP message MSG_GPS_TIME
  uint16_t weeks; // # of weeks since unix epoch MSG_GPS_TIME
  uint32_t tow; // time of week. 
  int ns_residual; // residual nanoseconds for precise time

  // populated from SBP message MSG_BASELINE_NED
  int x; // baseline North coordinate
  int y; // baseline East coordinate
  int z; // baseline Down coordinate
  unsigned short h_accuracy; // horizontal position accuracy estimate
  unsigned short v_accuracy; // vertical position accuracy estimate
  unsigned char n_sats;   // Number of satellites used in solution
  unsigned char position_flags; // status flags from MSG_BASELINE_NED. 
} piksi_ned_data;


int runModBusTcpServer(piksi_ned_data *piksi_struct)
{
  int socket;
    modbus_t *ctx;
    modbus_mapping_t *mb_mapping;
    int rc;
    int nPort = 502; // port number for modbus
    
    // to emulate a large MODBUS device we need at least 10000 input and holding registers
    mb_mapping = modbus_mapping_new(0, 0, 10000, 0);
    if (mb_mapping == NULL) {
        fprintf(stderr, "Failed to allocate the mapping: %s\n", modbus_strerror(errno));
        modbus_free(ctx);
        return -1;
    }

    printf("Waiting for TCP connection on Port %i \n",nPort);
    ctx = modbus_new_tcp("0.0.0.0", nPort);
    socket = modbus_tcp_listen(ctx, 1);
    modbus_tcp_accept(ctx, &socket);
    printf("TCP connection started!\n");
    
    for(;;) 
    {
      uint8_t query[MODBUS_TCP_MAX_ADU_LENGTH];

      rc = modbus_receive(ctx, query);
      if (rc >= 0) 
      {
        int nToShow = 10;
        int i=0;

        // request received. populate registers from structure
        //fprintf(stdout, "sizeof  ", sizeof(mb_mapping->tab_registers));
        mb_mapping->tab_registers[0] = piksi_struct->weeks;
        MODBUS_SET_INT32_TO_INT16( mb_mapping -> tab_registers, 1, piksi_struct -> tow);
        
        fprintf(stdout, "retrieving data from struct: weeks/tow ==> %d/%d\n", mb_mapping->tab_registers[0], MODBUS_GET_INT32_FROM_INT16(mb_mapping -> tab_registers, 1));
        
        //updateAllSurfaceCards(mb_mapping); // every request we want a new set of cards

        printf("Replying to request num bytes=%i (",rc);
        for(i=0;i<rc;i++)
        {
          printf("%i, ",query[i]);
        }
        printf(")\n");

        modbus_reply(ctx, query, rc, mb_mapping);

        // after each communication, show the first ? ModBus registers so you can see what is happening
        printf("tab_bits = ");
        for( i=0;i<nToShow;i++)
        {
          printf("%i, ",mb_mapping->tab_bits[i]);
        }
        printf("\n");

        printf("tab_input_bits = ");
        for( i=0;i<nToShow;i++)
        {
          printf("%i, ",mb_mapping->tab_input_bits[i]);
        }
        printf("\n");

        printf("tab_input_registers = ");
        for( i=0;i<nToShow;i++)
        {
          printf("%i, ",mb_mapping->tab_input_registers[i]);
        }
        printf("\n");

        printf("tab_registers = ");
        for( i=0;i<nToShow;i++)
        {
          printf("%i, ",mb_mapping->tab_registers[i]);
        }
        printf("\n");

        // every time we do a communication, update a bunch of the registers so we have something interesting to plot on the graphs
        mb_mapping->tab_registers[0]++; // increment the holding reg 0 for each read
        mb_mapping->tab_registers[1] = rand(); // this register is a full scale random number 0 - 0xffff
        mb_mapping->tab_input_registers[0] = 2; // version number
        for( i=1;i<nToShow;i++)
        {
          // randomly increase or decrease the register, but do not allow wrapping
          if( rand() > RAND_MAX/2 )
          {		
            if ( mb_mapping->tab_input_registers[i] < 0xfffe )
            {
              mb_mapping->tab_input_registers[i] += 1;
            }
            
            if( mb_mapping->tab_registers[i+1] < 0xfffe )
            {
              mb_mapping->tab_registers[i+1] += 1;
            }
          }
          else
          {
            if( mb_mapping->tab_input_registers[i] > 0 )
            {
              mb_mapping->tab_input_registers[i] -= 1;
            }
            if( mb_mapping->tab_registers[i+1] > 0 )
            {
              mb_mapping->tab_registers[i+1] -= 1;
            }
          } 
        }
      } 
      else 
      {
        /* Connection closed by the client or server */
        printf("Con Closed.\n");
        modbus_close(ctx); // close
        // immediately start waiting for another request again
        modbus_tcp_accept(ctx, &socket);
      }
    }


    printf("Quit the loop: %s\n", modbus_strerror(errno));

    modbus_mapping_free(mb_mapping);
    close(socket);
    modbus_free(ctx);

  
  
}


// Does not work on OS X, as you can't mmap over /dev/zero 
int main(void)
{
  int parpid = getpid(), childpid;
  piksi_ned_data *anon;
  anon = (piksi_ned_data *)mmap(NULL, sizeof(piksi_ned_data), PROT_READ|PROT_WRITE, MAP_ANONYMOUS|MAP_SHARED, -1, 0);

  if (anon == MAP_FAILED)
  {
    errx(1, "either mmap");
  }

  anon -> weeks = 1960;
  anon -> tow = 439816400;
  anon -> ns_residual = 0;
  anon -> x = 1;
  anon -> y = 1;
  anon -> z = 1;
  anon -> h_accuracy = 1;
  anon -> v_accuracy = 1;
  anon -> n_sats = 5;
  anon -> position_flags = 0x04;
  
  
  fprintf(stdout, "initial readings: #weeks/tow/x/y/z = %d/%d/%d/%d/%d\n", anon -> weeks, anon -> tow, anon -> x, anon -> y, anon -> z);
  

  switch ((childpid = fork())) {
  case -1:
    err(1, "fork");
    // NOTREACHED
  case 0: // ******************************** in the child process ********************************
    childpid = getpid();
    fprintf(stdout, "child PID %d\n", childpid);

    runModBusTcpServer(anon);
    
        
    /*
    // ************************************************************************************************
    sleep(1 );
    
    for (i=0;i<20;i++)
    {
      sleep(1);
      printf("#weeks/tow/x/y/z = %d/%d/%d/%d/%d\n", anon -> weeks, anon -> tow, anon -> x, anon -> y, anon -> z);
    }
    // ************************************************************************************************
    */
    fprintf(stdout, "terminating child process PID %d\n", childpid);
    
    munmap(anon, sizeof(piksi_ned_data));
    return EXIT_SUCCESS;
  }

  // ******************************** start of parent process code ********************************
  
  sleep(2);

  wait(NULL); // make parent process stay open until all child processes have terminated
  
  /*for (i=0;i<50000;i++)
  {
    usleep(1000);
    //fprintf(stdout, "#weeks/tow/x/y/z = %d/%d/%d/%d/%d\n", anon -> weeks, anon -> tow, anon -> x, anon -> y, anon -> z);
    (anon -> tow)++;
  }
*/

  
  fprintf(stdout, "terminating parent process PID %d\n", parpid);
  munmap(anon, sizeof(piksi_ned_data));
  return EXIT_SUCCESS;
}