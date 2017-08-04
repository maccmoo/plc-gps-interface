
#include <stdio.h>
//#include <stdint.h>
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


char *serial_port_name = NULL;
struct sp_port *piksi_port = NULL;
piksi_ned_data *CurrentData;

static sbp_msg_callbacks_node_t heartbeat_callback_node;
static sbp_msg_callbacks_node_t base_pos_llh_callback_node;
static sbp_msg_callbacks_node_t pos_llh_callback_node;
static sbp_msg_callbacks_node_t baseline_ned_callback_node;
static sbp_msg_callbacks_node_t gps_time_callback_node;
static sbp_msg_callbacks_node_t utc_time_callback_node;
static sbp_msg_callbacks_node_t imu_raw_callback_node;


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
  char blnIMUEnabled = 1;
  char blnUTCTimeEnabled = 1;
  char blnBasePosEnabled = 1;
  char blnBaseNEDEnabled = 1;
  char blnPiksiOutputEnabled=1;
  
  
  //int intSocket = -1;                   // for modbus testing
  //modbus_t *ctx;                        // for modbus testing
  //modbus_mapping_t *mb_mapping;         // for modbus testing

  
  //fprintf(stdout, ".2\n");
  if (argc <= 1) {
    usage(argv[0]);
    fprintf(stdout, "a\n");
    exit(EXIT_FAILURE);
  }
  //fprintf(stdout, ".3\n");

  while ((opt = getopt(argc, argv, "p:b:ufin")) != -1) 
  {
    //fprintf(stdout, ".4\n");
    switch (opt) {
      case 'p':
        //fprintf(stdout, ".5\n");
        serial_port_name = (char *)calloc(strlen(optarg) + 1, sizeof(char));
        //fprintf(stdout, ".6\n");
        if (!serial_port_name) {
          fprintf(stderr, "Cannot allocate memory!\n");
          exit(EXIT_FAILURE);
        }
        //fprintf(stdout, ".7\n");
        strcpy(serial_port_name, optarg);
        fprintf(stdout, "serial port set to \"%s\"\n", serial_port_name);
        break;
      case 'b':
        //fprintf(stdout, "1\n");
        {
           long l = -1;
           l=strtol(optarg, 0, 10);

           //fprintf(stdout, "2\n");
           if ((!optarg) ||  (l <= 0))
             { 
                fprintf(stderr, "invalid baud rate under option -b  %s - expecting a number\n", 
                optarg?optarg:"");
                fprintf(stdout, "b\n");
                usage(argv[0]);
                exit(EXIT_FAILURE);
             };
           //fprintf(stdout, "3\n");
           intBaudRate = (int) l;
        }
        //fprintf(stdout, "4\n");
        fprintf(stdout, "baud rate set to %d\n\n", intBaudRate);
        break;
      case 'u': // if parameter is in existence, then disable UTC Time callback function
        blnUTCTimeEnabled = 0; // disable UTC Time callback function
        break;
      case 'f': // if parameter is in existence, then disable base position callback function
        blnBasePosEnabled = 0; // disable base position callback function
        break;
      case 'i': // if parameter is in existence, then disable IMU callback function
        blnIMUEnabled = 0; // disable IMU callback function
        break;
      case 'n': // if parameter is in existence, then disable baseline rover NED callback function
        blnBaseNEDEnabled = 0; // disable disable baseline rover NED callback function
        break;
      /*case 'h':
        usage(argv[0]);
        exit(EXIT_FAILURE);*/
    }
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

  if (blnPiksiOutputEnabled) {
  sbp_register_callback(&s, SBP_MSG_HEARTBEAT, &heartbeat_callback, NULL,
                        &heartbeat_callback_node);
  }

  if ((blnPiksiOutputEnabled) || (blnBasePosEnabled)) {
  sbp_register_callback(&s, SBP_MSG_BASE_POS_LLH, &base_pos_llh_callback, NULL,
                        &base_pos_llh_callback_node);
  }

  if (blnPiksiOutputEnabled) {
  sbp_register_callback(&s, SBP_MSG_POS_LLH, &pos_llh_callback, NULL,
                        &pos_llh_callback_node);
  }

  if ((blnPiksiOutputEnabled) || (blnBaseNEDEnabled)) {
    sbp_register_callback(&s, SBP_MSG_BASELINE_NED, &baseline_ned_callback, NULL,
                          &baseline_ned_callback_node);
  }

  if (blnPiksiOutputEnabled) {
  sbp_register_callback(&s, SBP_MSG_GPS_TIME, &gps_time_callback, NULL,
                        &gps_time_callback_node);
  }
						
  if ((blnPiksiOutputEnabled) || (blnUTCTimeEnabled)) {
  sbp_register_callback(&s, SBP_MSG_UTC_TIME, &utc_time_callback, NULL,
                        &utc_time_callback_node);
  }

  if ((blnPiksiOutputEnabled) || (blnIMUEnabled)) {
    sbp_register_callback(&s, SBP_MSG_IMU_RAW, &imu_raw_callback, NULL,
                          &imu_raw_callback_node);
  }
/*
  //  ***************************** initialise modbus ***************************** 
  ctx = modbus_new_tcp("0.0.0.0", 1502);
  // modbus_set_debug(ctx, TRUE); 

  mb_mapping = modbus_mapping_new(500, 500, 500, 500);
  if (mb_mapping == NULL) {
      fprintf(stderr, "Failed to allocate the mapping: %s\n",
              modbus_strerror(errno));
      modbus_free(ctx);
      return -1;
  }

  intSocket = modbus_tcp_listen(ctx, 1);
  modbus_tcp_accept(ctx, &intSocket);
  //  *****************************************************************************
*/
  while(1) {
    sbp_process(&s, &piksi_port_read);
  //  modbus_process_incoming_request();
  }

  /*
  //  ***************************** close modbus ***************************** 
  
      printf("Quit the loop: %s\n", modbus_strerror(errno));

    if (intSocket != -1) {
        close(intSocket);
    }
    modbus_mapping_free(mb_mapping);
    modbus_close(ctx);
    modbus_free(ctx);


  //  *****************************************************************************
  */
  
  
  
  return 0;
  }
