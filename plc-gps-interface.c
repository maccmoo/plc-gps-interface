#define HeartBeatTimeout 3 // if a timeout isn't received in this number of seconds, then attempt to reestablish connection.
#define OKMessageInterval 4 // send a message every x seconds when we're receiving messages from the GPS device
#define reg_MSG_GPS_TIME_wn 100
#define reg_MSG_GPS_TIME_tow 101
#define reg_MSG_GPS_TIME_ns_residual 103
#define reg_MSG_GPS_TIME_flags 105

#define reg_MSG_BASELINE_NED_tow 106
#define reg_MSG_BASELINE_NED_n 108
#define reg_MSG_BASELINE_NED_e 110
#define reg_MSG_BASELINE_NED_d 112
#define reg_MSG_BASELINE_NED_h_accuracy 114
#define reg_MSG_BASELINE_NED_v_accuracy 115
#define reg_MSG_BASELINE_NED_n_sats 116
#define reg_MSG_BASELINE_NED_flags 117

#define reg_MSG_POS_LLH_tow 118
#define reg_MSG_POS_LLH_lat 120
#define reg_MSG_POS_LLH_lat_decimal 122
#define reg_MSG_POS_LLH_lon 124
#define reg_MSG_POS_LLH_lon_decimal 126
#define reg_MSG_POS_LLH_height 128
#define reg_MSG_POS_LLH_height_decimal 130
#define reg_MSG_POS_LLH_h_accuracy 132
#define reg_MSG_POS_LLH_v_accuracy 133
#define reg_MSG_POS_LLH_n_sats 134
#define reg_MSG_POS_LLH_flags 135




#define reg_MSG_VEL_NED_tow 136
#define reg_MSG_VEL_NED_n 138
#define reg_MSG_VEL_NED_e 140
#define reg_MSG_VEL_NED_d 142
#define reg_MSG_VEL_NED_h_accuracy 144
#define reg_MSG_VEL_NED_v_accuracy 145
#define reg_MSG_VEL_NED_n_sats 146
#define reg_MSG_VEL_NED_flags 147

#define reg_MSG_IMU_RAW_tow 148
#define reg_MSG_IMU_RAW_tow_f 150
#define reg_MSG_IMU_RAW_acc_x 151
#define reg_MSG_IMU_RAW_acc_y 152
#define reg_MSG_IMU_RAW_acc_z 153
#define reg_MSG_IMU_RAW_gyr_x 154
#define reg_MSG_IMU_RAW_gyr_y 155
#define reg_MSG_IMU_RAW_gyr_z 156

#define reg_MSG_POS_ECEF_tow 157
#define reg_MSG_POS_ECEF_x 159
#define reg_MSG_POS_ECEF_x_decimal 161
#define reg_MSG_POS_ECEF_y 163
#define reg_MSG_POS_ECEF_y_decimal 165
#define reg_MSG_POS_ECEF_z 167
#define reg_MSG_POS_ECEF_z_decimal 169
#define reg_MSG_POS_ECEF_accuracy 171
#define reg_MSG_POS_ECEF_n_sats 172
#define reg_MSG_POS_ECEF_flags 173

#define reg_MSG_BASELINE_ECEF_tow 174
#define reg_MSG_BASELINE_ECEF_x 176
#define reg_MSG_BASELINE_ECEF_y 178
#define reg_MSG_BASELINE_ECEF_z 180
#define reg_MSG_BASELINE_ECEF_accuracy 182
#define reg_MSG_BASELINE_ECEF_n_sats 183
#define reg_MSG_BASELINE_ECEF_flags 184

#define reg_UNIX_EPOCH_TIME 185

#define reg_MSG_UTC_TIME_flags 187
#define reg_MSG_UTC_TIME_tow 188
#define reg_MSG_UTC_TIME_year 190
#define reg_MSG_UTC_TIME_month 191
#define reg_MSG_UTC_TIME_day 192
#define reg_MSG_UTC_TIME_hours 193
#define reg_MSG_UTC_TIME_minutes 194
#define reg_MSG_UTC_TIME_seconds 195
#define reg_MSG_UTC_TIME_ns 196

#define reg_MSG_DEVICE_MONITOR_dev_vin 198
#define reg_MSG_DEVICE_MONITOR_cpu_vint 199
#define reg_MSG_DEVICE_MONITOR_cpu_vaux 200
#define reg_MSG_DEVICE_MONITOR_cpu_temperature 201
#define reg_MSG_DEVICE_MONITOR_fe_temperature 202

#define reg_MSG_LINUX_SYS_STATE_mem_total 203
#define reg_MSG_LINUX_SYS_STATE_pcpu 204
#define reg_MSG_LINUX_SYS_STATE_pmem 205
#define reg_MSG_LINUX_SYS_STATE_procs_starting 206
#define reg_MSG_LINUX_SYS_STATE_procs_stopping 207
#define reg_MSG_LINUX_SYS_STATE_pid_count 208

#define reg_SBP_MSG_AGE_CORRECTIONS_tow 209
#define reg_SBP_MSG_AGE_CORRECTIONS_age 211

#define reg_SBP_MSG_IMU_AUX_imu_type 212
#define reg_SBP_MSG_IMU_AUX_temp 213
#define reg_SBP_MSG_IMU_AUX_imu_conf 214




// this macro is defined in libmodbus 3.14, but not in 3.06. 
// As it is very useful, I will define it here with an ifndef which "should" not break if we upgrade to 3.14
#ifndef MODBUS_SET_INT32_TO_INT16 
# define MODBUS_SET_INT32_TO_INT16(tab_int16, index, value) \
    do { \
        tab_int16[(index)    ] = (value) >> 16; \
        tab_int16[(index) + 1] = (value); \
    } while (0)
#endif

#ifndef MODBUS_SET_INT64_TO_INT16 
# define MODBUS_SET_INT64_TO_INT16(tab_int16, index, value) \
    do { \
        tab_int16[(index)    ] = (value) >> 48; \
        tab_int16[(index) + 1] = (value) >> 32; \
        tab_int16[(index) + 2] = (value) >> 16; \
        tab_int16[(index) + 3] = (value); \
    } while (0)
#endif

#include <stdio.h>
//#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/mman.h>
//#include <libserialport.h>

#include <libsbp/sbp.h>
#include <libsbp/system.h>
#include <libsbp/observation.h>
#include <libsbp/navigation.h>
#include <libsbp/imu.h>
#include <libsbp/piksi.h>
#include <libsbp/linux.h>
#include <modbus/modbus.h>
#include <modbus/modbus-tcp.h>

#include <errno.h>
#include <err.h>
#include "sbp_callback_functions.h"
#include <slog.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <time.h>
//#include <libconfig.h>

//char strDummyInput[100];
char *serial_port_name = NULL;
//struct sp_port *piksi_port = NULL;
piksi_data_t *CurrentDataNode1;
piksi_data_t *CurrentDataNode2;
piksi_data_t *CurrentDataNode3;
piksi_data_t *CurrentDataNode4;
piksi_data_t *CurrentDataNode5;

// objects for TCP/IP connection to Piksi Multi
char *tcp_ip_addr = NULL;
char *tcp_ip_port=NULL;
int socketDescNode1 =- 1;
int socketDescNode2 =- 1;
int socketDescNode3 =- 1;
int socketDescNode4 =- 1;
int socketDescNode5 =- 1;
const char *strNode1IPAddress = "192.168.1.11";
const char *strNode2IPAddress = "192.168.1.12";
const char *strNode3IPAddress = "192.168.1.7";
const char *strNode4IPAddress = "192.168.1.15";
const char *strNode5IPAddress = "192.168.1.16";
const int intNode1Port = 55555;
const int intNode2Port = 55555;
const int intNode3Port = 55555;
const int intNode4Port = 55555;
const int intNode5Port = 55555;
const int intNodeQty = 5; // number of RTK GNSS nodes we wish to read from. 1 x 2 stackers, 2 x 1 reclaimer, 1 x 1 basestation

// GNSS node definitions
sbp_state_t sbp_stateNode1;
sbp_state_t sbp_stateNode2;
sbp_state_t sbp_stateNode3;
sbp_state_t sbp_stateNode4;
sbp_state_t sbp_stateNode5;

typedef struct sbp_callbacks {
	 sbp_msg_callbacks_node_t heartbeat_callback_node;
	 sbp_msg_callbacks_node_t base_pos_llh_callback_node;
	 sbp_msg_callbacks_node_t pos_llh_callback_node;
	 sbp_msg_callbacks_node_t baseline_ned_callback_node;
	 sbp_msg_callbacks_node_t vel_ned_callback_node;
	 sbp_msg_callbacks_node_t gps_time_callback_node;
	 sbp_msg_callbacks_node_t utc_time_callback_node;
	 sbp_msg_callbacks_node_t imu_raw_callback_node;
	 sbp_msg_callbacks_node_t baseline_ecef_callback_node;
	 sbp_msg_callbacks_node_t pos_ecef_callback_node;
	 sbp_msg_callbacks_node_t log_callback_node;
	 sbp_msg_callbacks_node_t device_monitor_callback_node;
	 sbp_msg_callbacks_node_t linux_sys_callback_node;
	 sbp_msg_callbacks_node_t correction_age_callback_node;
	 sbp_msg_callbacks_node_t imu_aux_callback_node;
} sbp_callbacks;

static sbp_callbacks CallbacksNode1;
static sbp_callbacks CallbacksNode2;
static sbp_callbacks CallbacksNode3;
static sbp_callbacks CallbacksNode4;
static sbp_callbacks CallbacksNode5;


char blnGPSTimeEnabled;
char blnIMUEnabled;
char blnUTCTimeEnabled;
char blnBasePosEnabled;
char blnLLHPosEnabled;
char blnBaseNEDEnabled;
char blnECEFEnabled;
char blnVelNEDEnabled;
char blnHeartbeatEnabled;


// These are for runModBusTcpServerMulti. not required if we don't complete the multithreaded version.
static modbus_t *ctx = NULL;
static modbus_mapping_t *mb_mapping;
static int server_socket = -1;
// end of definitions for runModBusTcpServerMulti. Remove to here if its removed.




int updateRegistersFromStruct(piksi_data_t *piksi_struct, modbus_mapping_t *mb_mapping, int intNodeID)
{
  time_t tmCurrentTime = time(NULL);

  mb_mapping->tab_registers[(intNodeID * 1000) + reg_MSG_GPS_TIME_wn] = piksi_struct->GPS_time_data->wn;
  
  
  // gps time data
  MODBUS_SET_INT32_TO_INT16( mb_mapping -> tab_registers, (intNodeID * 1000) + reg_MSG_GPS_TIME_tow, piksi_struct->GPS_time_data->tow);
  MODBUS_SET_INT32_TO_INT16( mb_mapping -> tab_registers, (intNodeID * 1000) + reg_MSG_GPS_TIME_ns_residual, piksi_struct->GPS_time_data->ns_residual);
  mb_mapping->tab_registers[(intNodeID * 1000) + reg_MSG_GPS_TIME_flags] = piksi_struct->GPS_time_data->flags;
  
  // baseline NED data
  MODBUS_SET_INT32_TO_INT16( mb_mapping -> tab_registers, (intNodeID * 1000) + reg_MSG_BASELINE_NED_tow, piksi_struct->baseline_NED_data->tow);
  MODBUS_SET_INT32_TO_INT16( mb_mapping -> tab_registers, (intNodeID * 1000) + reg_MSG_BASELINE_NED_n, piksi_struct->baseline_NED_data->n);
  MODBUS_SET_INT32_TO_INT16( mb_mapping -> tab_registers, (intNodeID * 1000) + reg_MSG_BASELINE_NED_e, piksi_struct->baseline_NED_data->e);
  MODBUS_SET_INT32_TO_INT16( mb_mapping -> tab_registers, (intNodeID * 1000) + reg_MSG_BASELINE_NED_d, piksi_struct->baseline_NED_data->d);
  mb_mapping->tab_registers[(intNodeID * 1000) + reg_MSG_BASELINE_NED_h_accuracy] = piksi_struct->baseline_NED_data->h_accuracy;
  mb_mapping->tab_registers[(intNodeID * 1000) + reg_MSG_BASELINE_NED_v_accuracy] = piksi_struct->baseline_NED_data->v_accuracy;
  mb_mapping->tab_registers[(intNodeID * 1000) + reg_MSG_BASELINE_NED_n_sats] = piksi_struct->baseline_NED_data->n_sats;
  mb_mapping->tab_registers[(intNodeID * 1000) + reg_MSG_BASELINE_NED_flags] = piksi_struct->baseline_NED_data->flags;

  // LLH data
  MODBUS_SET_INT32_TO_INT16( mb_mapping -> tab_registers, (intNodeID * 1000) + reg_MSG_POS_LLH_tow, piksi_struct->LLH_data->tow);
  //mb_mapping->tab_registers[(intNodeID * 1000) + reg_MSG_POS_LLH_lat]=0;
  //mb_mapping->tab_registers[(intNodeID * 1000) + reg_MSG_POS_LLH_lat + 1]=0;
  MODBUS_SET_INT32_TO_INT16( mb_mapping -> tab_registers, (intNodeID * 1000) + reg_MSG_POS_LLH_lat, (int)(piksi_struct->LLH_data->lat)); // output whole number as 32 bit integer 
  MODBUS_SET_INT32_TO_INT16( mb_mapping -> tab_registers, (intNodeID * 1000) + reg_MSG_POS_LLH_lat_decimal, abs((int)(((piksi_struct->LLH_data->lat)- (int)(piksi_struct->LLH_data->lat)) *10000000)) ); // output remainder as integer to 4dp. accuracy
  
  MODBUS_SET_INT32_TO_INT16( mb_mapping -> tab_registers, (intNodeID * 1000) + reg_MSG_POS_LLH_lon, (int)(piksi_struct->LLH_data->lon)); // output whole number as 32 bit integer 
  MODBUS_SET_INT32_TO_INT16( mb_mapping -> tab_registers, (intNodeID * 1000) + reg_MSG_POS_LLH_lon_decimal, abs((int)(((piksi_struct->LLH_data->lon)- (int)(piksi_struct->LLH_data->lon)) *10000000)) ); // output remainder as integer to 4dp. accuracy
  
  MODBUS_SET_INT32_TO_INT16( mb_mapping -> tab_registers, (intNodeID * 1000) + reg_MSG_POS_LLH_height, (int)(piksi_struct->LLH_data->height)); // output whole number as 32 bit integer 
  MODBUS_SET_INT32_TO_INT16( mb_mapping -> tab_registers, (intNodeID * 1000) + reg_MSG_POS_LLH_height_decimal,  abs((int)(((piksi_struct->LLH_data->height)- (int)(piksi_struct->LLH_data->height)) * 10000000)) ); // output remainder as integer to 4dp. accuracy
  mb_mapping->tab_registers[(intNodeID * 1000) + reg_MSG_POS_LLH_h_accuracy] = piksi_struct->LLH_data->h_accuracy;
  mb_mapping->tab_registers[(intNodeID * 1000) + reg_MSG_POS_LLH_v_accuracy] = piksi_struct->LLH_data->v_accuracy;
  mb_mapping->tab_registers[(intNodeID * 1000) + reg_MSG_POS_LLH_n_sats] = piksi_struct->LLH_data->n_sats;
  mb_mapping->tab_registers[(intNodeID * 1000) + reg_MSG_POS_LLH_flags] = piksi_struct->LLH_data->flags;
  
  // baseline NED VELOCITY data
  MODBUS_SET_INT32_TO_INT16( mb_mapping -> tab_registers, (intNodeID * 1000) + reg_MSG_VEL_NED_tow, piksi_struct->NED_velocity_data->tow);
  MODBUS_SET_INT32_TO_INT16( mb_mapping -> tab_registers, (intNodeID * 1000) + reg_MSG_VEL_NED_n, piksi_struct->NED_velocity_data->n);
  MODBUS_SET_INT32_TO_INT16( mb_mapping -> tab_registers, (intNodeID * 1000) + reg_MSG_VEL_NED_e, piksi_struct->NED_velocity_data->e);
  MODBUS_SET_INT32_TO_INT16( mb_mapping -> tab_registers, (intNodeID * 1000) + reg_MSG_VEL_NED_d, piksi_struct->NED_velocity_data->d);
  mb_mapping->tab_registers[(intNodeID * 1000) + reg_MSG_VEL_NED_h_accuracy] = piksi_struct->NED_velocity_data->h_accuracy;
  mb_mapping->tab_registers[(intNodeID * 1000) + reg_MSG_VEL_NED_v_accuracy] = piksi_struct->NED_velocity_data->v_accuracy;
  mb_mapping->tab_registers[(intNodeID * 1000) + reg_MSG_VEL_NED_n_sats] = piksi_struct->NED_velocity_data->n_sats;
  mb_mapping->tab_registers[(intNodeID * 1000) + reg_MSG_VEL_NED_flags] = piksi_struct->NED_velocity_data->flags;
  
  // raw IMU data
  MODBUS_SET_INT32_TO_INT16( mb_mapping -> tab_registers, (intNodeID * 1000) + reg_MSG_IMU_RAW_tow, piksi_struct->IMU_data->tow);
  mb_mapping->tab_registers[(intNodeID * 1000) + reg_MSG_IMU_RAW_tow_f] = piksi_struct->IMU_data->tow_f;
  mb_mapping->tab_registers[(intNodeID * 1000) + reg_MSG_IMU_RAW_acc_x] = piksi_struct->IMU_data->acc_x;
  mb_mapping->tab_registers[(intNodeID * 1000) + reg_MSG_IMU_RAW_acc_y] = piksi_struct->IMU_data->acc_y;
  mb_mapping->tab_registers[(intNodeID * 1000) + reg_MSG_IMU_RAW_acc_z] = piksi_struct->IMU_data->acc_z;
  mb_mapping->tab_registers[(intNodeID * 1000) + reg_MSG_IMU_RAW_gyr_x] = piksi_struct->IMU_data->gyr_x;
  mb_mapping->tab_registers[(intNodeID * 1000) + reg_MSG_IMU_RAW_gyr_y] = piksi_struct->IMU_data->gyr_y;
  mb_mapping->tab_registers[(intNodeID * 1000) + reg_MSG_IMU_RAW_gyr_z] = piksi_struct->IMU_data->gyr_z;

  // pos ECEF data
  MODBUS_SET_INT32_TO_INT16( mb_mapping -> tab_registers, (intNodeID * 1000) + reg_MSG_POS_ECEF_tow, piksi_struct->ECEF_data->tow);
  MODBUS_SET_INT32_TO_INT16( mb_mapping -> tab_registers, (intNodeID * 1000) + reg_MSG_POS_ECEF_x, (int)(piksi_struct->ECEF_data->x)); // output whole number as 32 bit integer 
  mb_mapping->tab_registers[(intNodeID * 1000) + reg_MSG_POS_ECEF_x_decimal] = abs((int)(((piksi_struct->ECEF_data->x)- (int)(piksi_struct->ECEF_data->x)) *10000)); // output remainder as integer to 4dp. accuracy
  MODBUS_SET_INT32_TO_INT16( mb_mapping -> tab_registers, (intNodeID * 1000) + reg_MSG_POS_ECEF_y, (int)(piksi_struct->ECEF_data->y)); // output whole number as 32 bit integer 
  mb_mapping->tab_registers[(intNodeID * 1000) + reg_MSG_POS_ECEF_y_decimal] = abs((int)(((piksi_struct->ECEF_data->y)- (int)(piksi_struct->ECEF_data->y)) *10000)); // output remainder as integer to 4dp. accuracy
  MODBUS_SET_INT32_TO_INT16( mb_mapping -> tab_registers, (intNodeID * 1000) + reg_MSG_POS_ECEF_z, (int)(piksi_struct->ECEF_data->z)); // output whole number as 32 bit integer 
  mb_mapping->tab_registers[(intNodeID * 1000) + reg_MSG_POS_ECEF_z_decimal] = abs((int)(((piksi_struct->ECEF_data->z)- (int)(piksi_struct->ECEF_data->z)) *10000)); // output remainder as integer to 4dp. accuracy

  mb_mapping->tab_registers[(intNodeID * 1000) + reg_MSG_POS_ECEF_accuracy] = piksi_struct->ECEF_data->accuracy;
  mb_mapping->tab_registers[(intNodeID * 1000) + reg_MSG_POS_ECEF_n_sats] = piksi_struct->ECEF_data->n_sats;
  mb_mapping->tab_registers[(intNodeID * 1000) + reg_MSG_POS_ECEF_flags] = piksi_struct->ECEF_data->flags;

  // baseline ECEF data
  MODBUS_SET_INT32_TO_INT16( mb_mapping -> tab_registers, (intNodeID * 1000) + reg_MSG_BASELINE_ECEF_tow, piksi_struct->baseline_ECEF_data->tow);
  MODBUS_SET_INT32_TO_INT16( mb_mapping -> tab_registers, (intNodeID * 1000) + reg_MSG_BASELINE_ECEF_x, piksi_struct->baseline_ECEF_data->x);
  MODBUS_SET_INT32_TO_INT16( mb_mapping -> tab_registers, (intNodeID * 1000) + reg_MSG_BASELINE_ECEF_y, piksi_struct->baseline_ECEF_data->y);
  MODBUS_SET_INT32_TO_INT16( mb_mapping -> tab_registers, (intNodeID * 1000) + reg_MSG_BASELINE_ECEF_z, piksi_struct->baseline_ECEF_data->z);
  mb_mapping->tab_registers[(intNodeID * 1000) + reg_MSG_BASELINE_ECEF_accuracy] = piksi_struct->baseline_ECEF_data->accuracy;
  mb_mapping->tab_registers[(intNodeID * 1000) + reg_MSG_BASELINE_ECEF_n_sats] = piksi_struct->baseline_ECEF_data->n_sats;
  mb_mapping->tab_registers[(intNodeID * 1000) + reg_MSG_BASELINE_ECEF_flags] = piksi_struct->baseline_ECEF_data->flags;
  
  // set current system time into 4 registers. can be used for heartbeat to prove comms. using GPS time 
  MODBUS_SET_INT32_TO_INT16( mb_mapping -> tab_registers, (intNodeID * 1000) + reg_UNIX_EPOCH_TIME, (unsigned long)(tmCurrentTime) );
  
  // UTC time data
  mb_mapping->tab_registers[(intNodeID * 1000) + reg_MSG_UTC_TIME_flags] = piksi_struct->UTC_data->flags;
  MODBUS_SET_INT32_TO_INT16( mb_mapping -> tab_registers, (intNodeID * 1000) + reg_MSG_UTC_TIME_tow, piksi_struct->UTC_data->tow);
  mb_mapping->tab_registers[(intNodeID * 1000) + reg_MSG_UTC_TIME_year] = piksi_struct->UTC_data->year;
  mb_mapping->tab_registers[(intNodeID * 1000) + reg_MSG_UTC_TIME_month] = piksi_struct->UTC_data->month;
  mb_mapping->tab_registers[(intNodeID * 1000) + reg_MSG_UTC_TIME_day] = piksi_struct->UTC_data->day;
  mb_mapping->tab_registers[(intNodeID * 1000) + reg_MSG_UTC_TIME_hours] = piksi_struct->UTC_data->hours;
  mb_mapping->tab_registers[(intNodeID * 1000) + reg_MSG_UTC_TIME_minutes] = piksi_struct->UTC_data->minutes;
  mb_mapping->tab_registers[(intNodeID * 1000) + reg_MSG_UTC_TIME_seconds] = piksi_struct->UTC_data->seconds;
  MODBUS_SET_INT32_TO_INT16( mb_mapping -> tab_registers, (intNodeID * 1000) + reg_MSG_UTC_TIME_ns, piksi_struct->UTC_data->ns);

  // device monitor data
  mb_mapping->tab_registers[(intNodeID * 1000) + reg_MSG_DEVICE_MONITOR_dev_vin] = piksi_struct->device_monitor_data->dev_vin;
  mb_mapping->tab_registers[(intNodeID * 1000) + reg_MSG_DEVICE_MONITOR_cpu_vint] = piksi_struct->device_monitor_data->cpu_vint;
  mb_mapping->tab_registers[(intNodeID * 1000) + reg_MSG_DEVICE_MONITOR_cpu_vaux] = piksi_struct->device_monitor_data->cpu_vaux;
  mb_mapping->tab_registers[(intNodeID * 1000) + reg_MSG_DEVICE_MONITOR_cpu_temperature] = piksi_struct->device_monitor_data->cpu_temperature;
  mb_mapping->tab_registers[(intNodeID * 1000) + reg_MSG_DEVICE_MONITOR_fe_temperature] = piksi_struct->device_monitor_data->fe_temperature;

  mb_mapping->tab_registers[(intNodeID * 1000) + reg_MSG_LINUX_SYS_STATE_mem_total] = piksi_struct->linux_sys_data->mem_total;
  mb_mapping->tab_registers[(intNodeID * 1000) + reg_MSG_LINUX_SYS_STATE_pcpu] = piksi_struct->linux_sys_data->pcpu;
  mb_mapping->tab_registers[(intNodeID * 1000) + reg_MSG_LINUX_SYS_STATE_pmem] = piksi_struct->linux_sys_data->pmem;
  mb_mapping->tab_registers[(intNodeID * 1000) + reg_MSG_LINUX_SYS_STATE_procs_starting] = piksi_struct->linux_sys_data->procs_starting;
  mb_mapping->tab_registers[(intNodeID * 1000) + reg_MSG_LINUX_SYS_STATE_procs_stopping] = piksi_struct->linux_sys_data->procs_stopping;
  mb_mapping->tab_registers[(intNodeID * 1000) + reg_MSG_LINUX_SYS_STATE_pid_count] = piksi_struct->linux_sys_data->pid_count;
  
  // correction age
  MODBUS_SET_INT32_TO_INT16( mb_mapping -> tab_registers, (intNodeID * 1000) + reg_SBP_MSG_AGE_CORRECTIONS_tow, piksi_struct->correction_age_data->tow);
  mb_mapping->tab_registers[(intNodeID * 1000) + reg_SBP_MSG_AGE_CORRECTIONS_age] = piksi_struct->correction_age_data->age;

  //imu aux data
  mb_mapping->tab_registers[(intNodeID * 1000) + reg_SBP_MSG_IMU_AUX_imu_type] = piksi_struct->IMU_AUX_data->imu_type;
  mb_mapping->tab_registers[(intNodeID * 1000) + reg_SBP_MSG_IMU_AUX_temp] = piksi_struct->IMU_AUX_data->temp;
  mb_mapping->tab_registers[(intNodeID * 1000) + reg_SBP_MSG_IMU_AUX_imu_conf] = piksi_struct->IMU_AUX_data->imu_conf;

  
  return 0;
}

 
static void close_sigint(int dummy)
{
    if (server_socket != -1) {
        close(server_socket);
    }
    //modbus_free(ctx);
    //modbus_mapping_free(mb_mapping);

    exit(dummy);
}

int runModBusTcpServerMulti()
{
	int nConnections = 5;
	int nPort = 502; // port number for modbus

	uint8_t query[MODBUS_TCP_MAX_ADU_LENGTH];
    int master_socket;
    int rc;
    fd_set refset;
    fd_set rdset;
    // Maximum file descriptor number 
    int fdmax;

	ctx = modbus_new_tcp("0.0.0.0", nPort); // 0.0.0.0 means to listen on all IP addresses

	// we only need holding registers. However the library doesn't like having 0 for the other values

	slog(0, SLOG_INFO, "ModbusTCP: creating modbus mapping");
	mb_mapping = modbus_mapping_new(1000,  1000, 10000, 1000);
	slog(0, SLOG_INFO, "ModbusTCP: modbus mapping complete");

	if (mb_mapping == NULL) {
	  slog(0, SLOG_ERROR, "ModbusTCP: Failed to allocate the mapping: %s", modbus_strerror(errno));
	  //modbus_free(ctx);
	  return -1;
	}

    server_socket = modbus_tcp_listen(ctx, nConnections);
    if (server_socket == -1) {
		slog(0, SLOG_ERROR, "ModbusTCP: Unable to listen TCP connection");
        modbus_free(ctx);
        return -1;
    }

    signal(SIGINT, close_sigint);

    // Clear the reference set of socket 
    FD_ZERO(&refset);
    // Add the server socket 
    FD_SET(server_socket, &refset);

    // Keep track of the max file descriptor 
    fdmax = server_socket;

    for (;;) {
        rdset = refset;
        if (select(fdmax+1, &rdset, NULL, NULL, NULL) == -1) {
            perror("Server select() failure.");
            close_sigint(1);
        }

        //  Run through the existing connections looking for data to be
        //  read 
        for (master_socket = 0; master_socket <= fdmax; master_socket++) {

            if (!FD_ISSET(master_socket, &rdset)) {
                continue;
            }

            if (master_socket == server_socket) {
                // A client is asking a new connection 
                socklen_t addrlen;
                struct sockaddr_in clientaddr;
                int newfd;

                // Handle new connections 
                addrlen = sizeof(clientaddr);
                memset(&clientaddr, 0, sizeof(clientaddr));
                newfd = accept(server_socket, (struct sockaddr *)&clientaddr, &addrlen);
                if (newfd == -1) {
                    perror("Server accept() error");
                } else {
                    FD_SET(newfd, &refset);

                    if (newfd > fdmax) {
                        // Keep track of the maximum 
                        fdmax = newfd;
                    }
					slog(0, SLOG_INFO, "ModbusTCP: New connection from %s:%d on socket %d", inet_ntoa(clientaddr.sin_addr), clientaddr.sin_port, newfd);
                }
            } else {
                modbus_set_socket(ctx, master_socket);
                rc = modbus_receive(ctx, query);
                if (rc > 0) {
					
					// nodeid2 is already configured in the stacker PLC on the first 1000 registers. writing to the first 1k as well as where it should be to not break it in the mean time
					//if (intNodeID == 2)
					//{
						updateRegistersFromStruct(CurrentDataNode2, mb_mapping, 0); 
						//updateRegistersFromStruct(piksi_struct, mb_mapping, 0); 
					//}
					
					// request received. populate registers from structure
					updateRegistersFromStruct(CurrentDataNode1, mb_mapping, 1); 
					updateRegistersFromStruct(CurrentDataNode2, mb_mapping, 2); 
					updateRegistersFromStruct(CurrentDataNode3, mb_mapping, 3); 
					updateRegistersFromStruct(CurrentDataNode4, mb_mapping, 4); 
					updateRegistersFromStruct(CurrentDataNode5, mb_mapping, 5); 
					
					
					
                    modbus_reply(ctx, query, rc, mb_mapping);
                } else if (rc == -1) {
                     // This example server in ended on connection closing or any errors. 
					slog(0, SLOG_INFO, "ModbusTCP: Connection closed on socket %d", master_socket);
                    close(master_socket);

                    // Remove from reference set 
                    FD_CLR(master_socket, &refset);

                    if (master_socket == fdmax) {
                        fdmax--;
                    }
                }
            }
        }
    }
    return 0;
}

void usage(char *prog_name) {
  slog(0, SLOG_ERROR, "usage: %s [-p serial port] [ -b baud rate] ", prog_name);
}


/*
void setup_socket(struct sockaddr_in server, const char* strIPAddress, int intPort)
{
  // struct sockaddr_in server;
  socketDescNode2 = socket(AF_INET , SOCK_STREAM | SOCK_NONBLOCK , 0); // changed it to be non-blocking sockets. unsure of the ramifications as things currently stand
  if (socketDescNode2 == -1)
  {
	slog(0, SLOG_ERROR, "Could not create socket");
  }

  memset(&server, '0', sizeof(server));
  server.sin_addr.s_addr = inet_addr(strIPAddress);
  server.sin_family = AF_INET;
  server.sin_port = htons(intPort);

  
  if (connect(socketDescNode2, (struct sockaddr *)&server , sizeof(server)) < 0)
  {
	slog(0, SLOG_ERROR, "Connection error");
  }
  
}
*/


void setup_socket(struct sockaddr_in server, const char* strIPAddress, int intPort, int *intSocketDesc)
{
  // struct sockaddr_in server;
  *intSocketDesc = socket(AF_INET , SOCK_STREAM | SOCK_NONBLOCK , 0); // non-blocking sockets
  if (*intSocketDesc == -1)
  {
	slog(0, SLOG_ERROR, "Could not create socket");
  }

  memset(&server, '0', sizeof(server));
  server.sin_addr.s_addr = inet_addr(strIPAddress);
  server.sin_family = AF_INET;
  server.sin_port = htons(intPort);

  
  if (connect(*intSocketDesc, (struct sockaddr *)&server , sizeof(server)) < 0)
  {
	slog(0, SLOG_ERROR, "Connection error to %s", strIPAddress);
  }
  
}

void close_socket( int *intSocketDesc)
{
  close(*intSocketDesc);
}


// passed as a function pointer to sbp_process in sbp.c library. therefore, can't add a parameter to make it generic. 
// therefore will likely need to define multiple versions for each device
s32 socket_readNode1(u8 *buff, u32 n, void *context)
{
  (void)context;
  s32 result;

  result = read(socketDescNode1, buff, n);
  return result;
}

s32 socket_readNode2(u8 *buff, u32 n, void *context)
{
  (void)context;
  s32 result;

  result = read(socketDescNode2, buff, n);
  return result;
}

s32 socket_readNode3(u8 *buff, u32 n, void *context)
{
  (void)context;
  s32 result;

  result = read(socketDescNode3, buff, n);
  return result;
}

s32 socket_readNode4(u8 *buff, u32 n, void *context)
{
  (void)context;
  s32 result;

  result = read(socketDescNode4, buff, n);
  return result;
}

s32 socket_readNode5(u8 *buff, u32 n, void *context)
{
  (void)context;
  s32 result;

  result = read(socketDescNode5, buff, n);
  return result;
}


/*
void sbp_setup_all(sbp_state_t *sbp_state, piksi_data_t *CurrentDataNode)
{

  sbp_state_init(sbp_state);

  // register callback functions for each sbp message we wish to process
  
  if (blnHeartbeatEnabled)  {
	sbp_register_callback(sbp_state, SBP_MSG_HEARTBEAT, &heartbeat_callback, (void*)CurrentDataNode,
						&heartbeat_callback_node);
	slog(0, SLOG_INFO, "registering heartbeat_callback");
  }

  if (blnBasePosEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_BASE_POS_LLH, &base_pos_llh_callback, (void*)CurrentDataNode,
						&base_pos_llh_callback_node);
	slog(0, SLOG_INFO, "registering base_pos_llh_callback");
  }

  if (blnLLHPosEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_POS_LLH, &pos_llh_callback, (void*)CurrentDataNode,
						&pos_llh_callback_node);
	slog(0, SLOG_INFO, "registering pos_llh_callback");
  }

  if (blnBaseNEDEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_BASELINE_NED, &baseline_ned_callback, (void*)CurrentDataNode,
						  &baseline_ned_callback_node);
	slog(0, SLOG_INFO, "registering baseline_ned_callback");
  }
  if (blnVelNEDEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_VEL_NED, &vel_ned_callback, (void*)CurrentDataNode,
						  &vel_ned_callback_node);
	slog(0, SLOG_INFO, "registering vel_ned_callback");
  }

  if (blnGPSTimeEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_GPS_TIME, &gps_time_callback, (void*)CurrentDataNode,
						&gps_time_callback_node);
	//fprintf(stdout, "registering gps_time_callback\n");
	slog(0, SLOG_INFO, "registering gps_time_callback");
  }
						
  if (blnUTCTimeEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_UTC_TIME, &utc_time_callback, (void*)CurrentDataNode,
						&utc_time_callback_node);
	slog(0, SLOG_INFO, "registering utc_time_callback");
  }

  if (blnIMUEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_IMU_RAW, &imu_raw_callback, (void*)CurrentDataNode,
						  &imu_raw_callback_node);
	slog(0, SLOG_INFO, "registering imu_raw_callback");
  }

  if (blnECEFEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_BASELINE_ECEF, &baseline_ecef_callback, (void*)CurrentDataNode,
						  &baseline_ecef_callback_node);
	//fprintf(stdout, "registering baseline_ecef_callback\n");
	slog(0, SLOG_INFO, "registering baseline_ecef_callback");
  }

  if (blnECEFEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_POS_ECEF, &pos_ecef_callback, (void*)CurrentDataNode,
						  &pos_ecef_callback_node);
	//fprintf(stdout, "registering pos_ecef_callback\n");
	slog(0, SLOG_INFO, "registering pos_ecef_callback");
  }

  if (blnECEFEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_LOG, &log_callback, (void*)CurrentDataNode,
						  &log_callback_node);
	slog(0, SLOG_INFO, "registering log_callback_node");
  }
  
  if (blnECEFEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_DEVICE_MONITOR, &device_monitor_callback, (void*)CurrentDataNode,
						  &device_monitor_callback_node);
	slog(0, SLOG_INFO, "registering device_monitor_callback_node");
  }
  
  if (blnECEFEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_LINUX_SYS_STATE, &linux_sys_callback, (void*)CurrentDataNode,
						  &linux_sys_callback_node);
	slog(0, SLOG_INFO, "registering linux_sys_callback_node");
  }
  
  if (blnECEFEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_AGE_CORRECTIONS, &correction_age_callback, (void*)CurrentDataNode,
						  &correction_age_callback_node);
	slog(0, SLOG_INFO, "registering correction_age_callback_node");
  }
  if (blnIMUEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_IMU_RAW, &imu_aux_callback, (void*)CurrentDataNode,
						  &imu_aux_callback_node);
	slog(0, SLOG_INFO, "registering imu_aux_callback");
  }
}
*/


void sbp_setup_all_struct1(sbp_state_t *sbp_state, piksi_data_t *CurrentDataNode)
{

  sbp_state_init(sbp_state);

  // register callback functions for each sbp message we wish to process
  
  if (blnHeartbeatEnabled)  {
	sbp_register_callback(sbp_state, SBP_MSG_HEARTBEAT, &heartbeat_callback1, (void*)CurrentDataNode, &(CallbacksNode1.heartbeat_callback_node) );
	// slog(0, SLOG_INFO, "registering heartbeat_callback 1");
  }
  if (blnBasePosEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_BASE_POS_LLH, &base_pos_llh_callback, (void*)CurrentDataNode, &(CallbacksNode1.base_pos_llh_callback_node) );
	// slog(0, SLOG_INFO, "registering base_pos_llh_callback 1");
  }
  if (blnLLHPosEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_POS_LLH, &pos_llh_callback, (void*)CurrentDataNode, &(CallbacksNode1.pos_llh_callback_node) );
	// slog(0, SLOG_INFO, "registering pos_llh_callback 1");
  }
  if (blnBaseNEDEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_BASELINE_NED, &baseline_ned_callback, (void*)CurrentDataNode, &(CallbacksNode1.baseline_ned_callback_node) );
	// slog(0, SLOG_INFO, "registering baseline_ned_callback 1");
  }
  if (blnVelNEDEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_VEL_NED, &vel_ned_callback, (void*)CurrentDataNode, &(CallbacksNode1.vel_ned_callback_node) );
	// slog(0, SLOG_INFO, "registering vel_ned_callback 1");
  }
  if (blnGPSTimeEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_GPS_TIME, &gps_time_callback, (void*)CurrentDataNode, &(CallbacksNode1.gps_time_callback_node) );
	// slog(0, SLOG_INFO, "registering gps_time_callback 1");
  }
  if (blnUTCTimeEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_UTC_TIME, &utc_time_callback, (void*)CurrentDataNode, &(CallbacksNode1.utc_time_callback_node) );
	// slog(0, SLOG_INFO, "registering utc_time_callback 1");
  }
  if (blnIMUEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_IMU_RAW, &imu_raw_callback, (void*)CurrentDataNode, &(CallbacksNode1.imu_raw_callback_node) );
	// slog(0, SLOG_INFO, "registering imu_raw_callback 1");
  }
  if (blnECEFEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_BASELINE_ECEF, &baseline_ecef_callback, (void*)CurrentDataNode, &(CallbacksNode1.baseline_ecef_callback_node) );
	// slog(0, SLOG_INFO, "registering baseline_ecef_callback 1");
  }
  if (blnECEFEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_POS_ECEF, &pos_ecef_callback, (void*)CurrentDataNode, &(CallbacksNode1.pos_ecef_callback_node) );
	// slog(0, SLOG_INFO, "registering pos_ecef_callback 1");
  }
  if (blnECEFEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_LOG, &log_callback, (void*)CurrentDataNode, &(CallbacksNode1.log_callback_node) );
	// slog(0, SLOG_INFO, "registering log_callback_node 1");
  }
  if (blnECEFEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_DEVICE_MONITOR, &device_monitor_callback, (void*)CurrentDataNode, &(CallbacksNode1.device_monitor_callback_node) );
	// slog(0, SLOG_INFO, "registering device_monitor_callback_node 1");
  }
  if (blnECEFEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_LINUX_SYS_STATE, &linux_sys_callback, (void*)CurrentDataNode, &(CallbacksNode1.linux_sys_callback_node) );
	// slog(0, SLOG_INFO, "registering linux_sys_callback_node 1");
  }
  if (blnECEFEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_AGE_CORRECTIONS, &correction_age_callback, (void*)CurrentDataNode, &(CallbacksNode1.correction_age_callback_node) );
	// slog(0, SLOG_INFO, "registering correction_age_callback_node 1");
  }
  if (blnIMUEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_IMU_RAW, &imu_aux_callback, (void*)CurrentDataNode, &(CallbacksNode1.imu_aux_callback_node) );
	// slog(0, SLOG_INFO, "registering imu_aux_callback 1");
  }
}

void sbp_setup_all_struct2(sbp_state_t *sbp_state, piksi_data_t *CurrentDataNode)
{

  sbp_state_init(sbp_state);

  // register callback functions for each sbp message we wish to process
  
  if (blnHeartbeatEnabled)  {
	sbp_register_callback(sbp_state, SBP_MSG_HEARTBEAT, &heartbeat_callback2, (void*)CurrentDataNode, &(CallbacksNode2.heartbeat_callback_node) );
	// slog(0, SLOG_INFO, "registering heartbeat_callback 2");
  }
  if (blnBasePosEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_BASE_POS_LLH, &base_pos_llh_callback, (void*)CurrentDataNode, &(CallbacksNode2.base_pos_llh_callback_node) );
	// slog(0, SLOG_INFO, "registering base_pos_llh_callback 2");
  }
  if (blnLLHPosEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_POS_LLH, &pos_llh_callback, (void*)CurrentDataNode, &(CallbacksNode2.pos_llh_callback_node) );
	// slog(0, SLOG_INFO, "registering pos_llh_callback 2");
  }
  if (blnBaseNEDEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_BASELINE_NED, &baseline_ned_callback, (void*)CurrentDataNode, &(CallbacksNode2.baseline_ned_callback_node) );
	// slog(0, SLOG_INFO, "registering baseline_ned_callback 2");
  }
  if (blnVelNEDEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_VEL_NED, &vel_ned_callback, (void*)CurrentDataNode, &(CallbacksNode2.vel_ned_callback_node) );
	// slog(0, SLOG_INFO, "registering vel_ned_callback 2");
  }
  if (blnGPSTimeEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_GPS_TIME, &gps_time_callback, (void*)CurrentDataNode, &(CallbacksNode2.gps_time_callback_node) );
	// slog(0, SLOG_INFO, "registering gps_time_callback 2");
  }
  if (blnUTCTimeEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_UTC_TIME, &utc_time_callback, (void*)CurrentDataNode, &(CallbacksNode2.utc_time_callback_node) );
	// slog(0, SLOG_INFO, "registering utc_time_callback 2");
  }
  if (blnIMUEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_IMU_RAW, &imu_raw_callback, (void*)CurrentDataNode, &(CallbacksNode2.imu_raw_callback_node) );
	// slog(0, SLOG_INFO, "registering imu_raw_callback 2");
  }
  if (blnECEFEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_BASELINE_ECEF, &baseline_ecef_callback, (void*)CurrentDataNode, &(CallbacksNode2.baseline_ecef_callback_node) );
	// slog(0, SLOG_INFO, "registering baseline_ecef_callback 2");
  }
  if (blnECEFEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_POS_ECEF, &pos_ecef_callback, (void*)CurrentDataNode, &(CallbacksNode2.pos_ecef_callback_node) );
	// slog(0, SLOG_INFO, "registering pos_ecef_callback 2");
  }
  if (blnECEFEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_LOG, &log_callback, (void*)CurrentDataNode, &(CallbacksNode2.log_callback_node) );
	// slog(0, SLOG_INFO, "registering log_callback_node 2");
  }
  if (blnECEFEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_DEVICE_MONITOR, &device_monitor_callback, (void*)CurrentDataNode, &(CallbacksNode2.device_monitor_callback_node) );
	// slog(0, SLOG_INFO, "registering device_monitor_callback_node 2");
  }
  if (blnECEFEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_LINUX_SYS_STATE, &linux_sys_callback, (void*)CurrentDataNode, &(CallbacksNode2.linux_sys_callback_node) );
	// slog(0, SLOG_INFO, "registering linux_sys_callback_node 2");
  }
  if (blnECEFEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_AGE_CORRECTIONS, &correction_age_callback, (void*)CurrentDataNode, &(CallbacksNode2.correction_age_callback_node) );
	// slog(0, SLOG_INFO, "registering correction_age_callback_node 2");
  }
  if (blnIMUEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_IMU_RAW, &imu_aux_callback, (void*)CurrentDataNode, &(CallbacksNode2.imu_aux_callback_node) );
	// slog(0, SLOG_INFO, "registering imu_aux_callback 2");
  }
}

void sbp_setup_all_struct3(sbp_state_t *sbp_state, piksi_data_t *CurrentDataNode)
{

  sbp_state_init(sbp_state);

  // register callback functions for each sbp message we wish to process
  
  if (blnHeartbeatEnabled)  {
	sbp_register_callback(sbp_state, SBP_MSG_HEARTBEAT, &heartbeat_callback3, (void*)CurrentDataNode, &(CallbacksNode3.heartbeat_callback_node) );
	// slog(0, SLOG_INFO, "registering heartbeat_callback 3");
  }
  if (blnBasePosEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_BASE_POS_LLH, &base_pos_llh_callback, (void*)CurrentDataNode, &(CallbacksNode3.base_pos_llh_callback_node) );
	// slog(0, SLOG_INFO, "registering base_pos_llh_callback 3");
  }
  if (blnLLHPosEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_POS_LLH, &pos_llh_callback, (void*)CurrentDataNode, &(CallbacksNode3.pos_llh_callback_node) );
	// slog(0, SLOG_INFO, "registering pos_llh_callback 3");
  }
  if (blnBaseNEDEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_BASELINE_NED, &baseline_ned_callback, (void*)CurrentDataNode, &(CallbacksNode3.baseline_ned_callback_node) );
	// slog(0, SLOG_INFO, "registering baseline_ned_callback 3");
  }
  if (blnVelNEDEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_VEL_NED, &vel_ned_callback, (void*)CurrentDataNode, &(CallbacksNode3.vel_ned_callback_node) );
	// slog(0, SLOG_INFO, "registering vel_ned_callback 3");
  }
  if (blnGPSTimeEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_GPS_TIME, &gps_time_callback, (void*)CurrentDataNode, &(CallbacksNode3.gps_time_callback_node) );
	// slog(0, SLOG_INFO, "registering gps_time_callback 3");
  }
  if (blnUTCTimeEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_UTC_TIME, &utc_time_callback, (void*)CurrentDataNode, &(CallbacksNode3.utc_time_callback_node) );
	// slog(0, SLOG_INFO, "registering utc_time_callback 3");
  }
  if (blnIMUEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_IMU_RAW, &imu_raw_callback, (void*)CurrentDataNode, &(CallbacksNode3.imu_raw_callback_node) );
	// slog(0, SLOG_INFO, "registering imu_raw_callback 3");
  }
  if (blnECEFEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_BASELINE_ECEF, &baseline_ecef_callback, (void*)CurrentDataNode, &(CallbacksNode3.baseline_ecef_callback_node) );
	// slog(0, SLOG_INFO, "registering baseline_ecef_callback 3");
  }
  if (blnECEFEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_POS_ECEF, &pos_ecef_callback, (void*)CurrentDataNode, &(CallbacksNode3.pos_ecef_callback_node) );
	// slog(0, SLOG_INFO, "registering pos_ecef_callback 3");
  }
  if (blnECEFEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_LOG, &log_callback, (void*)CurrentDataNode, &(CallbacksNode3.log_callback_node) );
	// slog(0, SLOG_INFO, "registering log_callback_node 3");
  }
  if (blnECEFEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_DEVICE_MONITOR, &device_monitor_callback, (void*)CurrentDataNode, &(CallbacksNode3.device_monitor_callback_node) );
	// slog(0, SLOG_INFO, "registering device_monitor_callback_node 3");
  }
  if (blnECEFEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_LINUX_SYS_STATE, &linux_sys_callback, (void*)CurrentDataNode, &(CallbacksNode3.linux_sys_callback_node) );
	// slog(0, SLOG_INFO, "registering linux_sys_callback_node 3");
  }
  if (blnECEFEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_AGE_CORRECTIONS, &correction_age_callback, (void*)CurrentDataNode, &(CallbacksNode3.correction_age_callback_node) );
	// slog(0, SLOG_INFO, "registering correction_age_callback_node 3");
  }
  if (blnIMUEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_IMU_RAW, &imu_aux_callback, (void*)CurrentDataNode, &(CallbacksNode3.imu_aux_callback_node) );
	// slog(0, SLOG_INFO, "registering imu_aux_callback 3");
  }
}

void sbp_setup_all_struct4(sbp_state_t *sbp_state, piksi_data_t *CurrentDataNode)
{

  sbp_state_init(sbp_state);

  // register callback functions for each sbp message we wish to process
  
  if (blnHeartbeatEnabled)  {
	sbp_register_callback(sbp_state, SBP_MSG_HEARTBEAT, &heartbeat_callback4, (void*)CurrentDataNode, &(CallbacksNode4.heartbeat_callback_node) );
	// slog(0, SLOG_INFO, "registering heartbeat_callback 4");
  }
  if (blnBasePosEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_BASE_POS_LLH, &base_pos_llh_callback, (void*)CurrentDataNode, &(CallbacksNode4.base_pos_llh_callback_node) );
	// slog(0, SLOG_INFO, "registering base_pos_llh_callback 4");
  }
  if (blnLLHPosEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_POS_LLH, &pos_llh_callback, (void*)CurrentDataNode, &(CallbacksNode4.pos_llh_callback_node) );
	// slog(0, SLOG_INFO, "registering pos_llh_callback 4");
  }
  if (blnBaseNEDEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_BASELINE_NED, &baseline_ned_callback, (void*)CurrentDataNode, &(CallbacksNode4.baseline_ned_callback_node) );
	// slog(0, SLOG_INFO, "registering baseline_ned_callback 4");
  }
  if (blnVelNEDEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_VEL_NED, &vel_ned_callback, (void*)CurrentDataNode, &(CallbacksNode4.vel_ned_callback_node) );
	// slog(0, SLOG_INFO, "registering vel_ned_callback 4");
  }
  if (blnGPSTimeEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_GPS_TIME, &gps_time_callback, (void*)CurrentDataNode, &(CallbacksNode4.gps_time_callback_node) );
	// slog(0, SLOG_INFO, "registering gps_time_callback 4");
  }
  if (blnUTCTimeEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_UTC_TIME, &utc_time_callback, (void*)CurrentDataNode, &(CallbacksNode4.utc_time_callback_node) );
	// slog(0, SLOG_INFO, "registering utc_time_callback 4");
  }
  if (blnIMUEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_IMU_RAW, &imu_raw_callback, (void*)CurrentDataNode, &(CallbacksNode4.imu_raw_callback_node) );
	// slog(0, SLOG_INFO, "registering imu_raw_callback 4");
  }
  if (blnECEFEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_BASELINE_ECEF, &baseline_ecef_callback, (void*)CurrentDataNode, &(CallbacksNode4.baseline_ecef_callback_node) );
	// slog(0, SLOG_INFO, "registering baseline_ecef_callback 4");
  }
  if (blnECEFEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_POS_ECEF, &pos_ecef_callback, (void*)CurrentDataNode, &(CallbacksNode4.pos_ecef_callback_node) );
	// slog(0, SLOG_INFO, "registering pos_ecef_callback 4");
  }
  if (blnECEFEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_LOG, &log_callback, (void*)CurrentDataNode, &(CallbacksNode4.log_callback_node) );
	// slog(0, SLOG_INFO, "registering log_callback_node 4");
  }
  if (blnECEFEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_DEVICE_MONITOR, &device_monitor_callback, (void*)CurrentDataNode, &(CallbacksNode4.device_monitor_callback_node) );
	// slog(0, SLOG_INFO, "registering device_monitor_callback_node 4");
  }
  if (blnECEFEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_LINUX_SYS_STATE, &linux_sys_callback, (void*)CurrentDataNode, &(CallbacksNode4.linux_sys_callback_node) );
	// slog(0, SLOG_INFO, "registering linux_sys_callback_node 4");
  }
  if (blnECEFEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_AGE_CORRECTIONS, &correction_age_callback, (void*)CurrentDataNode, &(CallbacksNode4.correction_age_callback_node) );
	// slog(0, SLOG_INFO, "registering correction_age_callback_node 4");
  }
  if (blnIMUEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_IMU_RAW, &imu_aux_callback, (void*)CurrentDataNode, &(CallbacksNode4.imu_aux_callback_node) );
	// slog(0, SLOG_INFO, "registering imu_aux_callback 4");
  }
}

void sbp_setup_all_struct5(sbp_state_t *sbp_state, piksi_data_t *CurrentDataNode)
{

  sbp_state_init(sbp_state);

  // register callback functions for each sbp message we wish to process
  
  if (blnHeartbeatEnabled)  {
	sbp_register_callback(sbp_state, SBP_MSG_HEARTBEAT, &heartbeat_callback5, (void*)CurrentDataNode, &(CallbacksNode5.heartbeat_callback_node) );
	// slog(0, SLOG_INFO, "registering heartbeat_callback 5");
  }
  if (blnBasePosEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_BASE_POS_LLH, &base_pos_llh_callback, (void*)CurrentDataNode, &(CallbacksNode5.base_pos_llh_callback_node) );
	// slog(0, SLOG_INFO, "registering base_pos_llh_callback 5");
  }
  if (blnLLHPosEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_POS_LLH, &pos_llh_callback, (void*)CurrentDataNode, &(CallbacksNode5.pos_llh_callback_node) );
	// slog(0, SLOG_INFO, "registering pos_llh_callback 5");
  }
  if (blnBaseNEDEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_BASELINE_NED, &baseline_ned_callback, (void*)CurrentDataNode, &(CallbacksNode5.baseline_ned_callback_node) );
	// slog(0, SLOG_INFO, "registering baseline_ned_callback 5");
  }
  if (blnVelNEDEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_VEL_NED, &vel_ned_callback, (void*)CurrentDataNode, &(CallbacksNode5.vel_ned_callback_node) );
	// slog(0, SLOG_INFO, "registering vel_ned_callback 5");
  }
  if (blnGPSTimeEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_GPS_TIME, &gps_time_callback, (void*)CurrentDataNode, &(CallbacksNode5.gps_time_callback_node) );
	// slog(0, SLOG_INFO, "registering gps_time_callback 5");
  }
  if (blnUTCTimeEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_UTC_TIME, &utc_time_callback, (void*)CurrentDataNode, &(CallbacksNode5.utc_time_callback_node) );
	// slog(0, SLOG_INFO, "registering utc_time_callback 5");
  }
  if (blnIMUEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_IMU_RAW, &imu_raw_callback, (void*)CurrentDataNode, &(CallbacksNode5.imu_raw_callback_node) );
	// slog(0, SLOG_INFO, "registering imu_raw_callback 5");
  }
  if (blnECEFEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_BASELINE_ECEF, &baseline_ecef_callback, (void*)CurrentDataNode, &(CallbacksNode5.baseline_ecef_callback_node) );
	// slog(0, SLOG_INFO, "registering baseline_ecef_callback 5");
  }
  if (blnECEFEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_POS_ECEF, &pos_ecef_callback, (void*)CurrentDataNode, &(CallbacksNode5.pos_ecef_callback_node) );
	// slog(0, SLOG_INFO, "registering pos_ecef_callback 5");
  }
  if (blnECEFEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_LOG, &log_callback, (void*)CurrentDataNode, &(CallbacksNode5.log_callback_node) );
	// slog(0, SLOG_INFO, "registering log_callback_node 5");
  }
  if (blnECEFEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_DEVICE_MONITOR, &device_monitor_callback, (void*)CurrentDataNode, &(CallbacksNode5.device_monitor_callback_node) );
	// slog(0, SLOG_INFO, "registering device_monitor_callback_node 5");
  }
  if (blnECEFEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_LINUX_SYS_STATE, &linux_sys_callback, (void*)CurrentDataNode, &(CallbacksNode5.linux_sys_callback_node) );
	// slog(0, SLOG_INFO, "registering linux_sys_callback_node 5");
  }
  if (blnECEFEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_AGE_CORRECTIONS, &correction_age_callback, (void*)CurrentDataNode, &(CallbacksNode5.correction_age_callback_node) );
	// slog(0, SLOG_INFO, "registering correction_age_callback_node 5");
  }
  if (blnIMUEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_IMU_RAW, &imu_aux_callback, (void*)CurrentDataNode, &(CallbacksNode5.imu_aux_callback_node) );
	// slog(0, SLOG_INFO, "registering imu_aux_callback 5");
  }
}


/*
void sbp_setup_all_generic(sbp_state_t *sbp_state, sbp_callbacks *callbacksNode, piksi_data_t *CurrentDataNode)
{

  sbp_state_init(sbp_state);

  // register callback functions for each sbp message we wish to process
  
  if (blnHeartbeatEnabled)  {
	sbp_register_callback(sbp_state, SBP_MSG_HEARTBEAT, &heartbeat_callback, (void*)CurrentDataNode, callbacksNode->heartbeat_callback_node);
	slog(0, SLOG_INFO, "registering heartbeat_callback");
  }

  if (blnBasePosEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_BASE_POS_LLH, &base_pos_llh_callback, (void*)CurrentDataNode, callbacksNode->base_pos_llh_callback_node);
	slog(0, SLOG_INFO, "registering base_pos_llh_callback");
  }

  if (blnLLHPosEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_POS_LLH, &pos_llh_callback, (void*)CurrentDataNode, callbacksNode->pos_llh_callback_node);
	slog(0, SLOG_INFO, "registering pos_llh_callback");
  }

  if (blnBaseNEDEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_BASELINE_NED,&baseline_ned_callback, (void*)CurrentDataNode, callbacksNode->baseline_ned_callback_node);
	slog(0, SLOG_INFO, "registering baseline_ned_callback");
  }
  if (blnVelNEDEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_VEL_NED, &vel_ned_callback, (void*)CurrentDataNode, callbacksNode->vel_ned_callback_node);
	slog(0, SLOG_INFO, "registering vel_ned_callback");
  }

  if (blnGPSTimeEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_GPS_TIME, &gps_time_callback, (void*)CurrentDataNode, callbacksNode->gps_time_callback_node);
	slog(0, SLOG_INFO, "registering gps_time_callback");
  }
						
  if (blnUTCTimeEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_UTC_TIME, &utc_time_callback, (void*)CurrentDataNode, callbacksNode->utc_time_callback_node);
	slog(0, SLOG_INFO, "registering utc_time_callback");
  }

  if (blnIMUEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_IMU_RAW, &imu_raw_callback, (void*)CurrentDataNode, callbacksNode->imu_raw_callback_node);
	slog(0, SLOG_INFO, "registering imu_raw_callback");
  }

  if (blnECEFEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_BASELINE_ECEF, &baseline_ecef_callback, (void*)CurrentDataNode, callbacksNode->baseline_ecef_callback_node);
	slog(0, SLOG_INFO, "registering baseline_ecef_callback");
  }

  if (blnECEFEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_POS_ECEF, &pos_ecef_callback, (void*)CurrentDataNode, callbacksNode->pos_ecef_callback_node);
	slog(0, SLOG_INFO, "registering pos_ecef_callback");
  }

  if (blnECEFEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_LOG, &log_callback, (void*)CurrentDataNode, callbacksNode->log_callback_node);
	slog(0, SLOG_INFO, "registering log_callback_node");
  }
  
  if (blnECEFEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_DEVICE_MONITOR, &device_monitor_callback, (void*)CurrentDataNode, callbacksNode->device_monitor_callback_node);
	slog(0, SLOG_INFO, "registering device_monitor_callback_node");
  }
  
  if (blnECEFEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_LINUX_SYS_STATE, &linux_sys_callback, (void*)CurrentDataNode, callbacksNode->linux_sys_callback_node);
	slog(0, SLOG_INFO, "registering linux_sys_callback_node");
  }
  
  if (blnECEFEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_AGE_CORRECTIONS, &correction_age_callback, (void*)CurrentDataNode, callbacksNode->correction_age_callback_node);
	slog(0, SLOG_INFO, "registering correction_age_callback_node");
  }
  if (blnIMUEnabled) {
	sbp_register_callback(sbp_state, SBP_MSG_IMU_RAW, &imu_aux_callback, (void*)CurrentDataNode, callbacksNode->imu_aux_callback_node);
	slog(0, SLOG_INFO, "registering imu_aux_callback");
  }
}
*/


int main(int argc, char **argv)
{
  int opt;
//  int result = 0;
  int intBaudRate = 115200;
  char blnEthernetComms = 1;
  time_t tmCurrentTime;

  struct sockaddr_in serverNode1;
  struct sockaddr_in serverNode2;
  struct sockaddr_in serverNode3;
  struct sockaddr_in serverNode4;
  struct sockaddr_in serverNode5;
  
  

  blnGPSTimeEnabled = 1;
  blnIMUEnabled = 1;
  blnUTCTimeEnabled = 1;
  blnBasePosEnabled = 1;
  blnLLHPosEnabled = 1;
  blnBaseNEDEnabled = 1;
  blnECEFEnabled = 1;
  blnVelNEDEnabled = 1;
  blnHeartbeatEnabled = 1;
  
  tmLastHeartbeat1 = time(NULL); // initialise last heartbeat to a time value, so first time comparison doesn't fail 
  tmLastHeartbeat2 = time(NULL); // initialise last heartbeat to a time value, so first time comparison doesn't fail 
  tmLastHeartbeat3 = time(NULL); // initialise last heartbeat to a time value, so first time comparison doesn't fail 
  tmLastHeartbeat4 = time(NULL); // initialise last heartbeat to a time value, so first time comparison doesn't fail 
  tmLastHeartbeat5 = time(NULL); // initialise last heartbeat to a time value, so first time comparison doesn't fail 
  time_t tmLastHeartbeatOKMessage1 = time(NULL); // temp variable to output diagnosis messages every 5 seconds
  time_t tmLastHeartbeatOKMessage2 = time(NULL); // temp variable to output diagnosis messages every 5 seconds
  time_t tmLastHeartbeatOKMessage3 = time(NULL); // temp variable to output diagnosis messages every 5 seconds
  time_t tmLastHeartbeatOKMessage4 = time(NULL); // temp variable to output diagnosis messages every 5 seconds
  time_t tmLastHeartbeatOKMessage5 = time(NULL); // temp variable to output diagnosis messages every 5 seconds
  
  blnDebugToScreen = 1;
  slog_init("slog.cfg", 0xffff, 1);
  slog(0, SLOG_INFO, "Opening logging file");
  
  int parpid = getpid(), childpid;
  slog(0, SLOG_INFO, "parent processID=%d", parpid);
  //piksi_data_t *CurrentDataNode2 = (piksi_data_t *)mmap(NULL, sizeof(*CurrentDataNode2), PROT_READ|PROT_WRITE, MAP_ANONYMOUS|MAP_SHARED, -1, 0);

  CurrentDataNode1 = (piksi_data_t *)mmap(NULL, sizeof(*CurrentDataNode1), PROT_READ|PROT_WRITE, MAP_ANONYMOUS|MAP_SHARED, -1, 0);
  CurrentDataNode2 = (piksi_data_t *)mmap(NULL, sizeof(*CurrentDataNode2), PROT_READ|PROT_WRITE, MAP_ANONYMOUS|MAP_SHARED, -1, 0);
  CurrentDataNode3 = (piksi_data_t *)mmap(NULL, sizeof(*CurrentDataNode3), PROT_READ|PROT_WRITE, MAP_ANONYMOUS|MAP_SHARED, -1, 0);
  CurrentDataNode4 = (piksi_data_t *)mmap(NULL, sizeof(*CurrentDataNode4), PROT_READ|PROT_WRITE, MAP_ANONYMOUS|MAP_SHARED, -1, 0);
  CurrentDataNode5 = (piksi_data_t *)mmap(NULL, sizeof(*CurrentDataNode5), PROT_READ|PROT_WRITE, MAP_ANONYMOUS|MAP_SHARED, -1, 0);
  piksi_data_setup(CurrentDataNode1);
  piksi_data_setup(CurrentDataNode2);
  piksi_data_setup(CurrentDataNode3);
  piksi_data_setup(CurrentDataNode4);
  piksi_data_setup(CurrentDataNode5);		
  
  if (argc <= 1) {
    usage(argv[0]);
    exit(EXIT_FAILURE);
  }

 // get arguments
  while ((opt = getopt(argc, argv, "p:b:gufinvlead")) != -1)
  {
    switch (opt) {
      case 'm': // mode of communication - IP or serial
        blnEthernetComms = 0; // disables ethernet communicationsand enables serial communications
        //fprintf(stdout, "ethernet disabled. comms will be over serial.\n");
		slog(0, SLOG_INFO, "ethernet disabled. comms will be over serial.");
        break;
      case 'p': // obtain IP address or serial port depending on which is selected. default is ethernet
        
        if (blnEthernetComms) { // if communication method is ethernet
          tcp_ip_addr = (char *)calloc(strlen(optarg) + 1, sizeof(char));
          if (!tcp_ip_addr) {
            //fprintf(stderr, "Cannot allocate memory!\n");
            slog(0, SLOG_ERROR, "Cannot allocate memory!");
            exit(EXIT_FAILURE);
          }
          strcpy(tcp_ip_addr, optarg);
          //fprintf(stdout, "IP address set to \"%s\"\n", tcp_ip_addr);
          slog(0, SLOG_INFO, "IP address set to \"%s\"", tcp_ip_addr);
        }
        else { //  communication method is serial
          serial_port_name = (char *)calloc(strlen(optarg) + 1, sizeof(char));
          if (!serial_port_name) {
            //fprintf(stderr, "Cannot allocate memory!\n");
            slog(0, SLOG_ERROR, "Cannot allocate memory!");
            exit(EXIT_FAILURE);
          }
          strcpy(serial_port_name, optarg);
          //fprintf(stdout, "serial port set to \"%s\"\n", serial_port_name);
          slog(0, SLOG_INFO, "serial port set to \"%s\"", serial_port_name);
        }
        break;    
      // additional communication parameter. if IP comms then this represents IP port. If serial comms then this represents baud rate
      // either way input should be a long
      case 'b': 
        if (blnEthernetComms) { // if communication method is ethernet
          tcp_ip_port = (char *)calloc(strlen(optarg) + 1, sizeof(char));
          if (!tcp_ip_port) {
            //fprintf(stderr, "Cannot allocate memory!\n");
            slog(0, SLOG_ERROR, "Cannot allocate memory!");
            exit(EXIT_FAILURE);
          }
          strcpy(tcp_ip_port, optarg);
          //fprintf(stdout, "IP port set to \"%s\"\n", tcp_ip_port);
          slog(0, SLOG_INFO, "IP port set to \"%s\"", tcp_ip_port);
        }
        else { //  communication method is serial
          {
            long l = -1;
            l=strtol(optarg, 0, 10);
            if ((!optarg) ||  (l <= 0))
            { 
               //fprintf(stderr, "invalid baud rate under option -b  %s - expecting a number\n", optarg?optarg:"");
               slog(0, SLOG_ERROR, "invalid baud rate under option -b  %s - expecting a number", optarg?optarg:"");

               usage(argv[0]);
               exit(EXIT_FAILURE);
            }
            intBaudRate = (int) l;

            //fprintf(stdout, "baud rate set to %d\n\n", intBaudRate);
            slog(0, SLOG_INFO, "baud rate set to %d", intBaudRate);
          }
        }
        break;
      case 'g': // if parameter is in existence, then disable GPS Time callback function
        blnGPSTimeEnabled = 0; // disable GPS Time callback function
        slog(0, SLOG_INFO, "GPS Time disabled");
        break;
      case 'u': // if parameter is in existence, then disable UTC Time callback function
        blnUTCTimeEnabled = 0; // disable UTC Time callback function
        slog(0, SLOG_INFO, "UTC Time disabled");
        break;
      case 'l': // if parameter is in existence, then disable LLH position callback function
        blnLLHPosEnabled = 0; // disable LLH position callback function
        slog(0, SLOG_INFO, "LLH position data collection disabled");
        break;
      case 'f': // if parameter is in existence, then disable base position callback function
        blnBasePosEnabled = 0; // disable base position callback function
        slog(0, SLOG_INFO, "base position data collection disabled");
        break;
      case 'i': // if parameter is in existence, then disable IMU callback function
        blnIMUEnabled = 0; // disable IMU callback function
        slog(0, SLOG_INFO, "IMU data collection disabled");
        break;
      case 'n': // if parameter is in existence, then disable baseline rover NED callback function
        blnBaseNEDEnabled = 0; // disable baseline rover NED callback function
        slog(0, SLOG_INFO, "Baseline NED Rover coordinate collection disabled");
        break;
      case 'e': // if parameter is in existence, then disable all ECEF callback functions
        blnECEFEnabled = 0; // disable disable all ECEF callback functions
        slog(0, SLOG_INFO, "all ECEF coordinate collection disabled");
        break;
      case 'v': // if parameter is in existence, then disable velocity rover NED callback function
        blnVelNEDEnabled = 0; // disable velocity rover NED callback function
        slog(0, SLOG_INFO, "NED velocity collection disabled");
        break;
      case 'a': // if parameter is in existence, then disable heartbeat callback function
        blnHeartbeatEnabled = 0; // disable heartbeat callback function
        slog(0, SLOG_INFO, "heartbeat collection disabled");
        break;
      case 'd': // if parameter is in existence, then send info and debug messages to screen
        blnDebugToScreen = 13; // enable debug to screen
        slog(0, SLOG_INFO, "info and debug messages enabled to screen");
        break;
    }
  }

// start separate process for ModbusTCP server
  switch ((childpid = fork()))  
  {
  case -1:
    err(1, "fork");
    // NOTREACHED
  case 0: // ******************************** in the child process ********************************
    childpid = getpid();
    slog(0, SLOG_INFO, "ModbusTCP:child PID %d", childpid);
        
    //runModBusTcpServer(CurrentDataNode2);
    runModBusTcpServerMulti();
    
    slog(0, SLOG_INFO, "ModbusTCP: terminating child process PID %d", childpid);
        
    piksi_data_close(CurrentDataNode2);
    return EXIT_SUCCESS;
  } // end of child process
  


   
	setup_socket(serverNode1, strNode1IPAddress, intNode1Port, &socketDescNode1);
	setup_socket(serverNode2, strNode2IPAddress, intNode2Port, &socketDescNode2);
	setup_socket(serverNode3, strNode3IPAddress, intNode3Port, &socketDescNode3);
	setup_socket(serverNode4, strNode4IPAddress, intNode4Port, &socketDescNode4);
	setup_socket(serverNode5, strNode5IPAddress, intNode5Port, &socketDescNode5);
	
	//sbp_setup_all(&sbp_stateNode2, CurrentDataNode2);
	sbp_setup_all_struct1(&sbp_stateNode1, CurrentDataNode1);
	sbp_setup_all_struct2(&sbp_stateNode2, CurrentDataNode2);
	sbp_setup_all_struct3(&sbp_stateNode3, CurrentDataNode3);
	sbp_setup_all_struct4(&sbp_stateNode4, CurrentDataNode4);
	sbp_setup_all_struct5(&sbp_stateNode5, CurrentDataNode5);
	//sbp_setup_all_generic(&sbp_stateNode2, callbacksNode2, CurrentDataNode2);

  
	while(1) {
	  
	  tmCurrentTime = time(NULL);

	  // duro 1 - base station
	  if ((blnHeartbeatEnabled) &&(tmCurrentTime - tmLastHeartbeat1 > HeartBeatTimeout) ) // heartbeat enabled and we haven't received a heartbeat in x seconds..
	  {
		slog(0, SLOG_INFO, "Heartbeat1 not detected in %ld seconds", (unsigned long)(tmCurrentTime - tmLastHeartbeat1));

		// if a heartbeat is not detected, its likely comms is lost to the GPS device. attempt to setup the socket again
		slog(0, SLOG_INFO, "Attempt to reconnect socket1");
		close_socket(&socketDescNode1);
		
		setup_socket(serverNode1, strNode1IPAddress, intNode1Port, &socketDescNode1);
		slog(0, SLOG_INFO, "Setup socket command issued.1");
		//sbp_setup_all(&sbp_stateNode1, CurrentDataNode1);
		sbp_setup_all_struct1(&sbp_stateNode1, CurrentDataNode1);
		tmLastHeartbeat1 = time(NULL);
	  }
	  else
	  {
		 if (( tmCurrentTime - tmLastHeartbeatOKMessage1 > OKMessageInterval) && (tmCurrentTime - tmLastSuccessfulRead1 < OKMessageInterval))
		 {
			//slog(2, SLOG_INFO, "Connection to GPS 1 OK. Received a messages within the last %d seconds. so only showing every %d seconds. ", OKMessageInterval, OKMessageInterval);
			//slog(2, SLOG_INFO, "CurrentData1 h_acc = %d. ", CurrentDataNode1->LLH_data->h_accuracy);
			tmLastHeartbeatOKMessage1 = time(NULL);
		 }
	  }
	  
	  // duro 2 - sk1501 fines stacker
	  if ((blnHeartbeatEnabled) &&(tmCurrentTime - tmLastHeartbeat2 > HeartBeatTimeout) ) // heartbeat enabled and we haven't received a heartbeat in x seconds..
	  {
		slog(0, SLOG_INFO, "Heartbeat2 not detected in %2d seconds", (unsigned long)(tmCurrentTime - tmLastHeartbeat2));

		// if a heartbeat is not detected, its likely comms is lost to the GPS device. attempt to setup the socket again
		slog(0, SLOG_INFO, "Attempt to reconnect socket2");
		close_socket(&socketDescNode2);
		
		setup_socket(serverNode2, strNode2IPAddress, intNode2Port, &socketDescNode2);
		slog(0, SLOG_INFO, "Setup socket command issued.2");
		//sbp_setup_all(&sbp_stateNode2, CurrentDataNode2);
		sbp_setup_all_struct2(&sbp_stateNode2, CurrentDataNode2);
		tmLastHeartbeat2 = time(NULL);
	  }
	  else
	  {
		 if (( tmCurrentTime - tmLastHeartbeatOKMessage2 > OKMessageInterval) && (tmCurrentTime - tmLastSuccessfulRead2 < OKMessageInterval))
		 {
			//slog(2, SLOG_INFO, "Connection to GPS 2 OK. Received a messages within the last %d seconds. so only showing every %d seconds. ", OKMessageInterval, OKMessageInterval);
			//slog(2, SLOG_INFO, "CurrentData2 h_acc = %d. ", CurrentDataNode2->LLH_data->h_accuracy);
			tmLastHeartbeatOKMessage2 = time(NULL);
		 }
	  }
	  
	  // duro 3 - ST02 lump stacker
	  if ((blnHeartbeatEnabled) &&(tmCurrentTime - tmLastHeartbeat3 > HeartBeatTimeout) ) // heartbeat enabled and we haven't received a heartbeat in x seconds..
	  {
		slog(0, SLOG_INFO, "Heartbeat3 not detected in %ld seconds", (unsigned long)(tmCurrentTime - tmLastHeartbeat3));

		// if a heartbeat is not detected, its likely comms is lost to the GPS device. attempt to setup the socket again
		slog(0, SLOG_INFO, "Attempt to reconnect socket3");
		close_socket(&socketDescNode3);
		
		setup_socket(serverNode3, strNode3IPAddress, intNode3Port, &socketDescNode3);
		slog(0, SLOG_INFO, "Setup socket command issued.3");
		//sbp_setup_all(&sbp_stateNode3, CurrentDataNode3);
		sbp_setup_all_struct3(&sbp_stateNode3, CurrentDataNode3);
		tmLastHeartbeat3 = time(NULL);
	  }
	  else
	  {
		 if (( tmCurrentTime - tmLastHeartbeatOKMessage3 > OKMessageInterval) && (tmCurrentTime - tmLastSuccessfulRead3 < OKMessageInterval))
		 {
			//slog(2, SLOG_INFO, "Connection to GPS 3 OK. Received a messages within the last %d seconds. so only showing every %d seconds. ", OKMessageInterval, OKMessageInterval);
			//slog(2, SLOG_INFO, "CurrentData3 h_acc = %d. ", CurrentDataNode3->LLH_data->h_accuracy);
			tmLastHeartbeatOKMessage3 = time(NULL);
		 }
	  }
	  
	  // duro 4 - RC01 fixed structure
	  if ((blnHeartbeatEnabled) &&(tmCurrentTime - tmLastHeartbeat4 > HeartBeatTimeout) ) // heartbeat enabled and we haven't received a heartbeat in x seconds..
	  {
		slog(0, SLOG_INFO, "Heartbeat4 not detected in %ld seconds", (unsigned long)(tmCurrentTime - tmLastHeartbeat4));

		// if a heartbeat is not detected, its likely comms is lost to the GPS device. attempt to setup the socket again
		slog(0, SLOG_INFO, "Attempt to reconnect socket4");
		close_socket(&socketDescNode4);
		
		setup_socket(serverNode4, strNode4IPAddress, intNode4Port, &socketDescNode4);
		slog(0, SLOG_INFO, "Setup socket command issued.4");
		//sbp_setup_all(&sbp_stateNode4, CurrentDataNode4);
		sbp_setup_all_struct4(&sbp_stateNode4, CurrentDataNode4);
		tmLastHeartbeat4 = time(NULL);
	  }
	  else
	  {
		 if (( tmCurrentTime - tmLastHeartbeatOKMessage4 > OKMessageInterval) && (tmCurrentTime - tmLastSuccessfulRead4 < OKMessageInterval))
		 {
			//slog(2, SLOG_INFO, "Connection to GPS 4 OK. Received a messages within the last %d seconds. so only showing every %d seconds. ", OKMessageInterval, OKMessageInterval);
			//slog(2, SLOG_INFO, "CurrentData4 h_acc = %d. ", CurrentDataNode4->LLH_data->h_accuracy);
			tmLastHeartbeatOKMessage4 = time(NULL);
		 }
	  }
	  
	  // duro 5 - RC01 boom!
	  if ((blnHeartbeatEnabled) &&(tmCurrentTime - tmLastHeartbeat5 > HeartBeatTimeout) ) // heartbeat enabled and we haven't received a heartbeat in x seconds..
	  {
		slog(0, SLOG_INFO, "Heartbeat5 not detected in %ld seconds", (unsigned long)(tmCurrentTime - tmLastHeartbeat5));

		// if a heartbeat is not detected, its likely comms is lost to the GPS device. attempt to setup the socket again
		slog(0, SLOG_INFO, "Attempt to reconnect socket5");
		close_socket(&socketDescNode5);
		
		setup_socket(serverNode5, strNode5IPAddress, intNode5Port, &socketDescNode5);
		slog(0, SLOG_INFO, "Setup socket command issued.5");
		//sbp_setup_all(&sbp_stateNode5, CurrentDataNode5);
		sbp_setup_all_struct5(&sbp_stateNode5, CurrentDataNode5);
		tmLastHeartbeat5 = time(NULL);
	  }
	  else
	  {
		 if (( tmCurrentTime - tmLastHeartbeatOKMessage5 > OKMessageInterval) && (tmCurrentTime - tmLastSuccessfulRead5 < OKMessageInterval))
		 {
			//slog(2, SLOG_INFO, "Connection to GPS 5 OK. Received a messages within the last %d seconds. so only showing every %d seconds. ", OKMessageInterval, OKMessageInterval);
			//slog(2, SLOG_INFO, "CurrentData5 h_acc = %d. ", CurrentDataNode5->LLH_data->h_accuracy);
			tmLastHeartbeatOKMessage5 = time(NULL);
		 }
	  }
	  

	  sbp_process(&sbp_stateNode1, &socket_readNode1); // process requests from Piksi Multi over Ethernet
	  sbp_process(&sbp_stateNode2, &socket_readNode2); // process requests from Piksi Multi over Ethernet
	  sbp_process(&sbp_stateNode3, &socket_readNode3); // process requests from Piksi Multi over Ethernet
	  sbp_process(&sbp_stateNode4, &socket_readNode4); // process requests from Piksi Multi over Ethernet
	  sbp_process(&sbp_stateNode5, &socket_readNode5); // process requests from Piksi Multi over Ethernet
	  usleep(100); // sleep for 100 milliseconds
	  
	}
	close_socket(&socketDescNode2);
  
  piksi_data_close(CurrentDataNode1);
  piksi_data_close(CurrentDataNode2);
  piksi_data_close(CurrentDataNode3);
  piksi_data_close(CurrentDataNode4);
  piksi_data_close(CurrentDataNode5);
  // fclose(fLogFile);
  //free(tcp_ip_addr);
  //free(tcp_ip_port);
  //free(serial_port_name);
  
  return 0;
  }
