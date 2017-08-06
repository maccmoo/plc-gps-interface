// this macro is defined in libmodbus 3.14, but not in 3.06. as it is very useful, 
// I will define it here with an ifndef which "should" not break if we upgrade to 3.14
#ifndef MODBUS_SET_INT32_TO_INT16 
# define MODBUS_SET_INT32_TO_INT16(tab_int16, index, value) \
    do { \
        tab_int16[(index)    ] = (value) >> 16; \
        tab_int16[(index) + 1] = (value); \
    } while (0)
#endif

#include <stdio.h>
//#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/mman.h>
#include <libserialport.h>

#include <libsbp/sbp.h>
#include <libsbp/system.h>
#include <libsbp/observation.h>
#include <libsbp/navigation.h>
#include <libsbp/imu.h>
#include <modbus/modbus.h>
#include <modbus/modbus-tcp.h>

#include <errno.h>
#include <err.h>
#include "sbp_callback_functions.h"

char strDummyInput[100];
char *serial_port_name = NULL;
struct sp_port *piksi_port = NULL;

static sbp_msg_callbacks_node_t heartbeat_callback_node;
static sbp_msg_callbacks_node_t base_pos_llh_callback_node;
static sbp_msg_callbacks_node_t pos_llh_callback_node;
static sbp_msg_callbacks_node_t baseline_ned_callback_node;
static sbp_msg_callbacks_node_t vel_ned_callback_node;
static sbp_msg_callbacks_node_t gps_time_callback_node;
static sbp_msg_callbacks_node_t utc_time_callback_node;
static sbp_msg_callbacks_node_t imu_raw_callback_node;



int runModBusTcpServer(piksi_data_t *piksi_struct)
{
  int socket;
  modbus_t *ctx;
  modbus_mapping_t *mb_mapping;
  int rc;
  int nPort = 502; // port number for modbus
  //fprintf(stdout, "c-1\n");
  // we only need holding registers. will define a few of each other type just in case
  mb_mapping = modbus_mapping_new(100, 100, 10000, 100);
  //fprintf(stdout, "c-2\n");
  if (mb_mapping == NULL) {
      fprintf(stderr, "Failed to allocate the mapping: %s\n", modbus_strerror(errno));
      //modbus_free(ctx);
      return -1;
  }
  //fprintf(stdout, "c-3\n");

  printf("Waiting for TCP connection on Port %i \n",nPort);
  ctx = modbus_new_tcp("0.0.0.0", nPort); // 0.0.0.0 means to listen on all IP addresses
  socket = modbus_tcp_listen(ctx, 1);
  modbus_tcp_accept(ctx, &socket);
  printf("TCP connection started!\n");
  
  for(;;) 
  {
    uint8_t query[MODBUS_TCP_MAX_ADU_LENGTH];

    rc = modbus_receive(ctx, query);
    if (rc >= 0) 
    {
      int nToShow = 20;
      int i=0;

      // request received. populate registers from structure
      //fprintf(stdout, "sizeof  ", sizeof(mb_mapping->tab_registers));
      mb_mapping->tab_registers[0] = piksi_struct->GPS_time_data->wn;
      MODBUS_SET_INT32_TO_INT16( mb_mapping -> tab_registers, 1, piksi_struct->GPS_time_data->tow);
      fprintf(stdout, "from modbus registers: week number/tow => %d/%d\n", mb_mapping->tab_registers[0], MODBUS_GET_INT32_FROM_INT16(mb_mapping -> tab_registers, 1) );
      fprintf(stdout, "struct: week number/tow => %d/%d\n", piksi_struct->GPS_time_data->wn, piksi_struct->GPS_time_data->tow);
      
      //fprintf(stdout, "retrieving data from struct: weeks/tow ==> %d/%d\n", mb_mapping->tab_registers[0], MODBUS_GET_INT32_FROM_INT16(mb_mapping -> tab_registers, 1));
      

      printf("Replying to request num bytes=%i (",rc);
      for(i=0;i<rc;i++)
      {
        printf("%i, ",query[i]);
      }
      printf(")\n");

      modbus_reply(ctx, query, rc, mb_mapping);

      /*// after each communication, show the first ? ModBus registers so you can see what is happening
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
      */
      
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
  return 0;
}



void usage(char *prog_name) {
  fprintf(stderr, "usage: %s [-p serial port] [ -b baud rate] \n", prog_name);
}

void setup_port()
{
  int result;

  result = sp_set_baudrate(piksi_port, 115200);
  if (result != SP_OK) {
    fprintf(stderr, "Cannot set port baud rate!\n");
    exit(EXIT_FAILURE);
  }

  result = sp_set_flowcontrol(piksi_port, SP_FLOWCONTROL_NONE);
  if (result != SP_OK) {
    fprintf(stderr, "Cannot set flow control!\n");
    exit(EXIT_FAILURE);
  }

  result = sp_set_bits(piksi_port, 8);
  if (result != SP_OK) {
    fprintf(stderr, "Cannot set data bits!\n");
    exit(EXIT_FAILURE);
  }

  result = sp_set_parity(piksi_port, SP_PARITY_NONE);
  if (result != SP_OK) {
    fprintf(stderr, "Cannot set parity!\n");
    exit(EXIT_FAILURE);
  }

  result = sp_set_stopbits(piksi_port, 1);
  if (result != SP_OK) {
    fprintf(stderr, "Cannot set stop bits!\n");
    exit(EXIT_FAILURE);
  }
}


u32 piksi_port_read(u8 *buff, u32 n, void *context)
{
  (void)context;
  u32 result;

  result = sp_blocking_read(piksi_port, buff, n, 0);

  return result;
}


int main(int argc, char **argv)
{
  //fprintf(stdout, ".1\n");
  int opt;
  int result = 0;
  int intBaudRate = 115200;
  sbp_state_t s;

  char blnGPSTimeEnabled = 1;
  char blnIMUEnabled = 1;
  char blnUTCTimeEnabled = 1;
  char blnBasePosEnabled = 1;
  char blnLLHPosEnabled = 1;
  char blnBaseNEDEnabled = 1;
  char blnVelNEDEnabled = 1;
  char blnPiksiOutputEnabled=1;
  char blnHeartbeatEnabled = 1;
  
  int parpid = getpid(), childpid;
  fprintf(stdout, "parent processID=%d\n", parpid);
  piksi_data_t *CurrentData = (piksi_data_t *)mmap(NULL, sizeof(*CurrentData), PROT_READ|PROT_WRITE, MAP_ANONYMOUS|MAP_SHARED, -1, 0);
  //piksi_data_t *CurrentData = calloc(1, sizeof(*CurrentData));
  piksi_data_setup(CurrentData);
  
  if (argc <= 1) {
    usage(argv[0]);
    fprintf(stdout, "a\n");
    exit(EXIT_FAILURE);
  }

  while ((opt = getopt(argc, argv, "p:b:gufinvle")) != -1) 
  {
    switch (opt) {
      case 'p':
        serial_port_name = (char *)calloc(strlen(optarg) + 1, sizeof(char));
        if (!serial_port_name) {
          fprintf(stderr, "Cannot allocate memory!\n");
          exit(EXIT_FAILURE);
        }
        strcpy(serial_port_name, optarg);
        fprintf(stdout, "serial port set to \"%s\"\n", serial_port_name);
        break;
      case 'b':
        {
           long l = -1;
           l=strtol(optarg, 0, 10);

           if ((!optarg) ||  (l <= 0))
             { 
                fprintf(stderr, "invalid baud rate under option -b  %s - expecting a number\n", 
                optarg?optarg:"");
                fprintf(stdout, "b\n");
                usage(argv[0]);
                exit(EXIT_FAILURE);
             };
           intBaudRate = (int) l;
        }
        fprintf(stdout, "baud rate set to %d\n\n", intBaudRate);
        break;
      case 'g': // if parameter is in existence, then disable GPS Time callback function
        blnGPSTimeEnabled = 0; // disable GPS Time callback function
        fprintf(stdout, "GPS Time disabled\n");
        break;
      case 'u': // if parameter is in existence, then disable UTC Time callback function
        blnUTCTimeEnabled = 0; // disable UTC Time callback function
        fprintf(stdout, "UTC Time disabled\n");
        break;
      case 'l': // if parameter is in existence, then disable LLH position callback function
        blnLLHPosEnabled = 0; // disable LLH position callback function
        fprintf(stdout, "LLH position data collection disabled\n");
        break;
      case 'f': // if parameter is in existence, then disable base position callback function
        blnBasePosEnabled = 0; // disable base position callback function
        fprintf(stdout, "base position data collection disabled\n");
        break;
      case 'i': // if parameter is in existence, then disable IMU callback function
        blnIMUEnabled = 0; // disable IMU callback function
        fprintf(stdout, "IMU data collection disabled\n");
        break;
      case 'n': // if parameter is in existence, then disable baseline rover NED callback function
        blnBaseNEDEnabled = 0; // disable baseline rover NED callback function
        fprintf(stdout, "Baseline NED Rover coordinate collection disabled\n");
        break;
      case 'v': // if parameter is in existence, then disable velocity rover NED callback function
        blnVelNEDEnabled = 0; // disable velocity rover NED callback function
        fprintf(stdout, "NED velocity collection disabled\n");
        break;
      case 'e': // if parameter is in existence, then disable heartbeat callback function
        blnHeartbeatEnabled = 0; // disable heartbeat callback function
        fprintf(stdout, "heartbeat collection disabled\n");
        break;
      /*case 'h':
        usage(argv[0]);
        exit(EXIT_FAILURE);*/
    }
  }

  fprintf(stdout, "Type any character and press enter to continue\n");
  scanf("%s", strDummyInput); // wait so we can read output

  
  switch ((childpid = fork())) 
  {
  case -1:
    err(1, "fork");
    // NOTREACHED
  case 0: // ******************************** in the child process ********************************
    childpid = getpid();
    fprintf(stdout, "child PID %d\n", childpid);

    
    
    
    runModBusTcpServer(CurrentData);
    /*while(1)
    {
      usleep(100000);
      fprintf(stdout, ".\n");
    }*/
    
    
    fprintf(stdout, "terminating child process PID %d\n", childpid);
    
    //munmap(anon, sizeof(piksi_ned_data));
    piksi_data_close(CurrentData);
    return EXIT_SUCCESS;
  }


  
  
  
  if (!serial_port_name) {
    fprintf(stderr, "Please supply the serial port path where the Piksi is " \
                    "connected!\n");
    exit(EXIT_FAILURE);
  }

  result = sp_get_port_by_name(serial_port_name, &piksi_port);
  if (result != SP_OK) {
    fprintf(stderr, "Cannot find provided serial port!\n");
    exit(EXIT_FAILURE);
  }

  result = sp_open(piksi_port, SP_MODE_READ);
  if (result != SP_OK) {
    fprintf(stderr, "Cannot open %s for reading!\n", serial_port_name);
    exit(EXIT_FAILURE);
  }

  setup_port();

  sbp_state_init(&s);

  if ((blnPiksiOutputEnabled) && (blnHeartbeatEnabled))  {
  sbp_register_callback(&s, SBP_MSG_HEARTBEAT, &heartbeat_callback, (void*)CurrentData,
                        &heartbeat_callback_node);
  }

  if ((blnPiksiOutputEnabled) && (blnBasePosEnabled)) {
  sbp_register_callback(&s, SBP_MSG_BASE_POS_LLH, &base_pos_llh_callback, (void*)CurrentData,
                        &base_pos_llh_callback_node);
  }

  if ((blnPiksiOutputEnabled) && (blnLLHPosEnabled)) {
  sbp_register_callback(&s, SBP_MSG_POS_LLH, &pos_llh_callback, (void*)CurrentData,
                        &pos_llh_callback_node);
  }

  if ((blnPiksiOutputEnabled) && (blnBaseNEDEnabled)) {
    sbp_register_callback(&s, SBP_MSG_BASELINE_NED, &baseline_ned_callback, (void*)CurrentData,
                          &baseline_ned_callback_node);
  }
  if ((blnPiksiOutputEnabled) && (blnVelNEDEnabled)) {
    sbp_register_callback(&s, SBP_MSG_VEL_NED, &vel_ned_callback, (void*)CurrentData,
                          &vel_ned_callback_node);
  }

  if ((blnPiksiOutputEnabled) && (blnGPSTimeEnabled)) {
  sbp_register_callback(&s, SBP_MSG_GPS_TIME, &gps_time_callback, (void*)CurrentData,
                        &gps_time_callback_node);
  }
						
  if ((blnPiksiOutputEnabled) && (blnUTCTimeEnabled)) {
  sbp_register_callback(&s, SBP_MSG_UTC_TIME, &utc_time_callback, (void*)CurrentData,
                        &utc_time_callback_node);
  }

  if ((blnPiksiOutputEnabled) && (blnIMUEnabled)) {
    sbp_register_callback(&s, SBP_MSG_IMU_RAW, &imu_raw_callback, (void*)CurrentData,
                          &imu_raw_callback_node);
  }


  while(1) {
    sbp_process(&s, &piksi_port_read); // process requests from Piksi Multi
  }

  
  piksi_data_close(CurrentData);
  
  return 0;
  }
