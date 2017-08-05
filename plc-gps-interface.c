
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
  // logical grouping of NED coordinate information

  // see definition of SBP protocol at https://github.com/swift-nav/libsbp/blob/master/docs/sbp.pdf
  // populated from SBP message MSG_BASELINE_NED
  uint32_t tow; // time of week. 
  int32_t n; // baseline North coordinate
  int32_t e; // baseline East coordinate
  int32_t d; // baseline Down coordinate
  uint16_t h_accuracy; // horizontal position accuracy estimate
  uint16_t v_accuracy; // vertical position accuracy estimate
  uint8_t n_sats;   // Number of satellites used in solution
  uint8_t position_flags; // status flags from MSG_BASELINE_NED. 
} piksi_NED_data_t;

typedef struct {
  // logical grouping of absolute geodetic coordinate information

  // see definition of SBP protocol at https://github.com/swift-nav/libsbp/blob/master/docs/sbp.pdf
  // populated from SBP message MSG_POS_LLH
  uint32_t tow; // time of week. 
  double latitude; // geodetic latitude
  double longitude; // geodetic longitude
  double height; // height above WGS84 ellipsoid
  uint16_t h_accuracy; // horizontal position accuracy estimate
  uint16_t v_accuracy; // vertical position accuracy estimate
  uint8_t n_sats;   // Number of satellites used in solution
  uint8_t position_flags; // status flags from MSG_POS_LLH. 
} piksi_LLH_data_t;

typedef struct {
  // logical grouping of NED velocity calculations as supplied by the Piksi multi device
  
  // see definition of SBP protocol at https://github.com/swift-nav/libsbp/blob/master/docs/sbp.pdf
  // populated from SBP message MSG_VEL_NED
  uint32_t tow; // time of week. 
  int32_t n_velocity; // baseline North velocity mm/s
  int32_t e_velocity; // baseline East velocity mm/s
  int32_t d_velocity; // baseline Down velocity mm/s
  // accuracy estimate information is stated as "not implemented. 
  // Defaults to 0" as of protocol specification 2.2.1, but in 2.2.8 there is no such qualifier.
  uint16_t h_accuracy; // horizontal velocity accuracy estimate
  uint16_t v_accuracy; // vertical velocity accuracy estimate
  uint8_t n_sats;   // Number of satellites used in solution
  uint8_t position_flags; // status flags from MSG_BASELINE_NED. 
} piksi_NED_velocity_t;

typedef struct {
  // logical grouping of IMU data 
  
  // see definition of SBP protocol at https://github.com/swift-nav/libsbp/blob/master/docs/sbp.pdf
  // populated from SBP message MSG_IMU_RAW
  uint32_t tow; // time of week. 
  uint8_t tow_f; // fractional part of time of week. allows greater accuracy than ms as required by high sampling rates
  int16_t acc_x; // acceleration in the body frame x axis
  int16_t acc_y; // acceleration in the body frame y axis
  int16_t acc_z; // acceleration in the body frame z axis
  int16_t gyro_x; // angular rate around the body frame x axis
  int16_t gyro_y; // angular rate around the body frame y axis
  int16_t gyro_z; // angular rate around the body frame z axis
} piksi_IMU_data_t;

typedef struct {
  // see definition of SBP protocol at https://github.com/swift-nav/libsbp/blob/master/docs/sbp.pdf

  // populated from SBP message MSG_GPS_TIME
  uint16_t weeks; // # of weeks since unix epoch MSG_GPS_TIME
  uint32_t tow; // time of week. 
  int ns_residual; // residual nanoseconds for precise time
  piksi_NED_data_t *NED_data;
  piksi_LLH_data_t *LLH_data;
  piksi_NED_velocity_t *NED_velocity_data;
  piksi_IMU_data_t *IMU_data;
} piksi_data_t;

char strDummyInput[100];
char *serial_port_name = NULL;
struct sp_port *piksi_port = NULL;
piksi_data_t *CurrentData;

static sbp_msg_callbacks_node_t heartbeat_callback_node;
static sbp_msg_callbacks_node_t base_pos_llh_callback_node;
static sbp_msg_callbacks_node_t pos_llh_callback_node;
static sbp_msg_callbacks_node_t baseline_ned_callback_node;
static sbp_msg_callbacks_node_t vel_ned_callback_node;
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

  char blnGPSTimeEnabled = 1;
  char blnIMUEnabled = 1;
  char blnUTCTimeEnabled = 1;
  char blnBasePosEnabled = 1;
  char blnLLHPosEnabled = 1;
  char blnBaseNEDEnabled = 1;
  char blnVelNEDEnabled = 1;
  char blnPiksiOutputEnabled=1;
  char blnHeartbeatEnabled = 1;
  
  //int intSocket = -1;                   // for modbus testing
  //modbus_t *ctx;                        // for modbus testing
  //modbus_mapping_t *mb_mapping;         // for modbus testing

  
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
  sbp_register_callback(&s, SBP_MSG_HEARTBEAT, &heartbeat_callback, NULL,
                        &heartbeat_callback_node);
  }

  if ((blnPiksiOutputEnabled) && (blnBasePosEnabled)) {
  sbp_register_callback(&s, SBP_MSG_BASE_POS_LLH, &base_pos_llh_callback, NULL,
                        &base_pos_llh_callback_node);
  }

  if ((blnPiksiOutputEnabled) && (blnLLHPosEnabled)) {
  sbp_register_callback(&s, SBP_MSG_POS_LLH, &pos_llh_callback, NULL,
                        &pos_llh_callback_node);
  }

  if ((blnPiksiOutputEnabled) && (blnBaseNEDEnabled)) {
    sbp_register_callback(&s, SBP_MSG_BASELINE_NED, &baseline_ned_callback, NULL,
                          &baseline_ned_callback_node);
  }
  if ((blnPiksiOutputEnabled) && (blnVelNEDEnabled)) {
    sbp_register_callback(&s, SBP_MSG_VEL_NED, &vel_ned_callback, NULL,
                          &vel_ned_callback_node);
  }

  if ((blnPiksiOutputEnabled) && (blnGPSTimeEnabled)) {
  sbp_register_callback(&s, SBP_MSG_GPS_TIME, &gps_time_callback, NULL,
                        &gps_time_callback_node);
  }
						
  if ((blnPiksiOutputEnabled) && (blnUTCTimeEnabled)) {
  sbp_register_callback(&s, SBP_MSG_UTC_TIME, &utc_time_callback, NULL,
                        &utc_time_callback_node);
  }

  if ((blnPiksiOutputEnabled) && (blnIMUEnabled)) {
    sbp_register_callback(&s, SBP_MSG_IMU_RAW, &imu_raw_callback, NULL,
                          &imu_raw_callback_node);
  }


  while(1) {
    sbp_process(&s, &piksi_port_read); // process requests from Piksi Multi
  }

  
  return 0;
  }
