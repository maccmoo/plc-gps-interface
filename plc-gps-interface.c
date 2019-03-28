#define HeartBeatTimeout 10 // if a timeout isn't received in this number of seconds, then attempt to reestablish connection.
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
#define reg_MSG_POS_LLH_lon 124
#define reg_MSG_POS_LLH_height 128
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

char strDummyInput[100];
char *serial_port_name = NULL;
struct sp_port *piksi_port = NULL;


// objects for TCP/IP connection to Piksi Multi
char *tcp_ip_addr = NULL;
char *tcp_ip_port=NULL;
int socket_desc =- 1;

static sbp_msg_callbacks_node_t heartbeat_callback_node;
static sbp_msg_callbacks_node_t base_pos_llh_callback_node;
static sbp_msg_callbacks_node_t pos_llh_callback_node;
static sbp_msg_callbacks_node_t baseline_ned_callback_node;
static sbp_msg_callbacks_node_t vel_ned_callback_node;
static sbp_msg_callbacks_node_t gps_time_callback_node;
static sbp_msg_callbacks_node_t utc_time_callback_node;
static sbp_msg_callbacks_node_t imu_raw_callback_node;
static sbp_msg_callbacks_node_t baseline_ecef_callback_node;
static sbp_msg_callbacks_node_t pos_ecef_callback_node;
static sbp_msg_callbacks_node_t log_callback_node;
static sbp_msg_callbacks_node_t device_monitor_callback_node;
static sbp_msg_callbacks_node_t linux_sys_callback_node;
static sbp_msg_callbacks_node_t correction_age_callback_node;
static sbp_msg_callbacks_node_t imu_aux_callback_node;



char blnGPSTimeEnabled;
char blnIMUEnabled;
char blnUTCTimeEnabled;
char blnBasePosEnabled;
char blnLLHPosEnabled;
char blnBaseNEDEnabled;
char blnECEFEnabled;
char blnVelNEDEnabled;
char blnHeartbeatEnabled;

piksi_data_t *CurrentData;
sbp_state_t sbp_state;

// These are for runModBusTcpServerMulti. not required if we don't complete the multithreaded version.
static modbus_t *ctx = NULL;
static modbus_mapping_t *mb_mapping;
static int server_socket = -1;
// end of definitions for runModBusTcpServerMulti. Remove to here if its removed.




int updateRegistersFromStruct(piksi_data_t *piksi_struct, modbus_mapping_t *mb_mapping)
{
  time_t tmCurrentTime = time(NULL);

  mb_mapping->tab_registers[reg_MSG_GPS_TIME_wn] = piksi_struct->GPS_time_data->wn;
  
  
  // gps time data
  MODBUS_SET_INT32_TO_INT16( mb_mapping -> tab_registers, reg_MSG_GPS_TIME_tow, piksi_struct->GPS_time_data->tow);
  MODBUS_SET_INT32_TO_INT16( mb_mapping -> tab_registers, reg_MSG_GPS_TIME_ns_residual, piksi_struct->GPS_time_data->ns_residual);
  mb_mapping->tab_registers[reg_MSG_GPS_TIME_flags] = piksi_struct->GPS_time_data->flags;
  
  // baseline NED data
  MODBUS_SET_INT32_TO_INT16( mb_mapping -> tab_registers, reg_MSG_BASELINE_NED_tow, piksi_struct->baseline_NED_data->tow);
  MODBUS_SET_INT32_TO_INT16( mb_mapping -> tab_registers, reg_MSG_BASELINE_NED_n, piksi_struct->baseline_NED_data->n);
  MODBUS_SET_INT32_TO_INT16( mb_mapping -> tab_registers, reg_MSG_BASELINE_NED_e, piksi_struct->baseline_NED_data->e);
  MODBUS_SET_INT32_TO_INT16( mb_mapping -> tab_registers, reg_MSG_BASELINE_NED_d, piksi_struct->baseline_NED_data->d);
  mb_mapping->tab_registers[reg_MSG_BASELINE_NED_h_accuracy] = piksi_struct->baseline_NED_data->h_accuracy;
  mb_mapping->tab_registers[reg_MSG_BASELINE_NED_v_accuracy] = piksi_struct->baseline_NED_data->v_accuracy;
  mb_mapping->tab_registers[reg_MSG_BASELINE_NED_n_sats] = piksi_struct->baseline_NED_data->n_sats;
  mb_mapping->tab_registers[reg_MSG_BASELINE_NED_flags] = piksi_struct->baseline_NED_data->flags;

  // LLH data
  MODBUS_SET_INT32_TO_INT16( mb_mapping -> tab_registers, reg_MSG_POS_LLH_tow, piksi_struct->LLH_data->tow);
  mb_mapping->tab_registers[reg_MSG_POS_LLH_lat] = 0; // temporarily set to 0 till 64 bit float support is implemented
  mb_mapping->tab_registers[reg_MSG_POS_LLH_lat+1] = 0; 
  mb_mapping->tab_registers[reg_MSG_POS_LLH_lat+2] = 0; 
  mb_mapping->tab_registers[reg_MSG_POS_LLH_lat+3] = 0; 
  mb_mapping->tab_registers[reg_MSG_POS_LLH_lon] = 0; // temporarily set to 0 till 64 bit float support is implemented
  mb_mapping->tab_registers[reg_MSG_POS_LLH_lon+1] = 0; 
  mb_mapping->tab_registers[reg_MSG_POS_LLH_lon+2] = 0; 
  mb_mapping->tab_registers[reg_MSG_POS_LLH_lon+3] = 0; 
  mb_mapping->tab_registers[reg_MSG_POS_LLH_height] = 0; // temporarily set to 0 till 64 bit float support is implemented
  mb_mapping->tab_registers[reg_MSG_POS_LLH_height+1] = 0; 
  mb_mapping->tab_registers[reg_MSG_POS_LLH_height+2] = 0; 
  mb_mapping->tab_registers[reg_MSG_POS_LLH_height+3] = 0; 
  mb_mapping->tab_registers[reg_MSG_POS_LLH_h_accuracy] = piksi_struct->LLH_data->h_accuracy;
  mb_mapping->tab_registers[reg_MSG_POS_LLH_v_accuracy] = piksi_struct->LLH_data->v_accuracy;
  mb_mapping->tab_registers[reg_MSG_POS_LLH_n_sats] = piksi_struct->LLH_data->n_sats;
  mb_mapping->tab_registers[reg_MSG_POS_LLH_flags] = piksi_struct->LLH_data->flags;
  
  // baseline NED VELOCITY data
  MODBUS_SET_INT32_TO_INT16( mb_mapping -> tab_registers, reg_MSG_VEL_NED_tow, piksi_struct->NED_velocity_data->tow);
  MODBUS_SET_INT32_TO_INT16( mb_mapping -> tab_registers, reg_MSG_VEL_NED_n, piksi_struct->NED_velocity_data->n);
  MODBUS_SET_INT32_TO_INT16( mb_mapping -> tab_registers, reg_MSG_VEL_NED_e, piksi_struct->NED_velocity_data->e);
  MODBUS_SET_INT32_TO_INT16( mb_mapping -> tab_registers, reg_MSG_VEL_NED_d, piksi_struct->NED_velocity_data->d);
  mb_mapping->tab_registers[reg_MSG_VEL_NED_h_accuracy] = piksi_struct->NED_velocity_data->h_accuracy;
  mb_mapping->tab_registers[reg_MSG_VEL_NED_v_accuracy] = piksi_struct->NED_velocity_data->v_accuracy;
  mb_mapping->tab_registers[reg_MSG_VEL_NED_n_sats] = piksi_struct->NED_velocity_data->n_sats;
  mb_mapping->tab_registers[reg_MSG_VEL_NED_flags] = piksi_struct->NED_velocity_data->flags;
  
  // raw IMU data
  MODBUS_SET_INT32_TO_INT16( mb_mapping -> tab_registers, reg_MSG_IMU_RAW_tow, piksi_struct->IMU_data->tow);
  mb_mapping->tab_registers[reg_MSG_IMU_RAW_tow_f] = piksi_struct->IMU_data->tow_f;
  mb_mapping->tab_registers[reg_MSG_IMU_RAW_acc_x] = piksi_struct->IMU_data->acc_x;
  mb_mapping->tab_registers[reg_MSG_IMU_RAW_acc_y] = piksi_struct->IMU_data->acc_y;
  mb_mapping->tab_registers[reg_MSG_IMU_RAW_acc_z] = piksi_struct->IMU_data->acc_z;
  mb_mapping->tab_registers[reg_MSG_IMU_RAW_gyr_x] = piksi_struct->IMU_data->gyr_x;
  mb_mapping->tab_registers[reg_MSG_IMU_RAW_gyr_y] = piksi_struct->IMU_data->gyr_y;
  mb_mapping->tab_registers[reg_MSG_IMU_RAW_gyr_z] = piksi_struct->IMU_data->gyr_z;

  // pos ECEF data
  MODBUS_SET_INT32_TO_INT16( mb_mapping -> tab_registers, reg_MSG_POS_ECEF_tow, piksi_struct->ECEF_data->tow);
  MODBUS_SET_INT32_TO_INT16( mb_mapping -> tab_registers, reg_MSG_POS_ECEF_x, (int)(CurrentData->ECEF_data->x)); // output whole number as 32 bit integer 
  mb_mapping->tab_registers[reg_MSG_POS_ECEF_x_decimal] = abs((int)(((CurrentData->ECEF_data->x)- (int)(CurrentData->ECEF_data->x)) *10000)); // output remainder as integer to 4dp. accuracy
  MODBUS_SET_INT32_TO_INT16( mb_mapping -> tab_registers, reg_MSG_POS_ECEF_y, (int)(CurrentData->ECEF_data->y)); // output whole number as 32 bit integer 
  mb_mapping->tab_registers[reg_MSG_POS_ECEF_y_decimal] = abs((int)(((CurrentData->ECEF_data->y)- (int)(CurrentData->ECEF_data->y)) *10000)); // output remainder as integer to 4dp. accuracy
  MODBUS_SET_INT32_TO_INT16( mb_mapping -> tab_registers, reg_MSG_POS_ECEF_z, (int)(CurrentData->ECEF_data->z)); // output whole number as 32 bit integer 
  mb_mapping->tab_registers[reg_MSG_POS_ECEF_z_decimal] = abs((int)(((CurrentData->ECEF_data->z)- (int)(CurrentData->ECEF_data->z)) *10000)); // output remainder as integer to 4dp. accuracy
  mb_mapping->tab_registers[reg_MSG_POS_ECEF_accuracy] = piksi_struct->ECEF_data->accuracy;
  mb_mapping->tab_registers[reg_MSG_POS_ECEF_n_sats] = piksi_struct->ECEF_data->n_sats;
  mb_mapping->tab_registers[reg_MSG_POS_ECEF_flags] = piksi_struct->ECEF_data->flags;

  // baseline ECEF data
  MODBUS_SET_INT32_TO_INT16( mb_mapping -> tab_registers, reg_MSG_BASELINE_ECEF_tow, piksi_struct->baseline_ECEF_data->tow);
  MODBUS_SET_INT32_TO_INT16( mb_mapping -> tab_registers, reg_MSG_BASELINE_ECEF_x, piksi_struct->baseline_ECEF_data->x);
  MODBUS_SET_INT32_TO_INT16( mb_mapping -> tab_registers, reg_MSG_BASELINE_ECEF_y, piksi_struct->baseline_ECEF_data->y);
  MODBUS_SET_INT32_TO_INT16( mb_mapping -> tab_registers, reg_MSG_BASELINE_ECEF_z, piksi_struct->baseline_ECEF_data->z);
  mb_mapping->tab_registers[reg_MSG_BASELINE_ECEF_accuracy] = piksi_struct->baseline_ECEF_data->accuracy;
  mb_mapping->tab_registers[reg_MSG_BASELINE_ECEF_n_sats] = piksi_struct->baseline_ECEF_data->n_sats;
  mb_mapping->tab_registers[reg_MSG_BASELINE_ECEF_flags] = piksi_struct->baseline_ECEF_data->flags;
  
  // set current system time into 4 registers. can be used for heartbeat to prove comms. using GPS time 
  MODBUS_SET_INT32_TO_INT16( mb_mapping -> tab_registers, reg_UNIX_EPOCH_TIME, (unsigned long)(tmCurrentTime) );
  
  // UTC time data
  mb_mapping->tab_registers[reg_MSG_UTC_TIME_flags] = piksi_struct->UTC_data->flags;
  MODBUS_SET_INT32_TO_INT16( mb_mapping -> tab_registers, reg_MSG_UTC_TIME_tow, piksi_struct->UTC_data->tow);
  mb_mapping->tab_registers[reg_MSG_UTC_TIME_year] = piksi_struct->UTC_data->year;
  mb_mapping->tab_registers[reg_MSG_UTC_TIME_month] = piksi_struct->UTC_data->month;
  mb_mapping->tab_registers[reg_MSG_UTC_TIME_day] = piksi_struct->UTC_data->day;
  mb_mapping->tab_registers[reg_MSG_UTC_TIME_hours] = piksi_struct->UTC_data->hours;
  mb_mapping->tab_registers[reg_MSG_UTC_TIME_minutes] = piksi_struct->UTC_data->minutes;
  mb_mapping->tab_registers[reg_MSG_UTC_TIME_seconds] = piksi_struct->UTC_data->seconds;
  MODBUS_SET_INT32_TO_INT16( mb_mapping -> tab_registers, reg_MSG_UTC_TIME_ns, piksi_struct->UTC_data->ns);

  // device monitor data
  mb_mapping->tab_registers[reg_MSG_DEVICE_MONITOR_dev_vin] = piksi_struct->device_monitor_data->dev_vin;
  mb_mapping->tab_registers[reg_MSG_DEVICE_MONITOR_cpu_vint] = piksi_struct->device_monitor_data->cpu_vint;
  mb_mapping->tab_registers[reg_MSG_DEVICE_MONITOR_cpu_vaux] = piksi_struct->device_monitor_data->cpu_vaux;
  mb_mapping->tab_registers[reg_MSG_DEVICE_MONITOR_cpu_temperature] = piksi_struct->device_monitor_data->cpu_temperature;
  mb_mapping->tab_registers[reg_MSG_DEVICE_MONITOR_fe_temperature] = piksi_struct->device_monitor_data->fe_temperature;

  mb_mapping->tab_registers[reg_MSG_LINUX_SYS_STATE_mem_total] = piksi_struct->linux_sys_data->mem_total;
  mb_mapping->tab_registers[reg_MSG_LINUX_SYS_STATE_pcpu] = piksi_struct->linux_sys_data->pcpu;
  mb_mapping->tab_registers[reg_MSG_LINUX_SYS_STATE_pmem] = piksi_struct->linux_sys_data->pmem;
  mb_mapping->tab_registers[reg_MSG_LINUX_SYS_STATE_procs_starting] = piksi_struct->linux_sys_data->procs_starting;
  mb_mapping->tab_registers[reg_MSG_LINUX_SYS_STATE_procs_stopping] = piksi_struct->linux_sys_data->procs_stopping;
  mb_mapping->tab_registers[reg_MSG_LINUX_SYS_STATE_pid_count] = piksi_struct->linux_sys_data->pid_count;
  
  // correction age
  MODBUS_SET_INT32_TO_INT16( mb_mapping -> tab_registers, reg_SBP_MSG_AGE_CORRECTIONS_tow, piksi_struct->correction_age_data->tow);
  mb_mapping->tab_registers[reg_SBP_MSG_AGE_CORRECTIONS_age] = piksi_struct->correction_age_data->age;

  //imu aux data
  mb_mapping->tab_registers[reg_SBP_MSG_IMU_AUX_imu_type] = piksi_struct->IMU_AUX_data->imu_type;
  mb_mapping->tab_registers[reg_SBP_MSG_IMU_AUX_temp] = piksi_struct->IMU_AUX_data->temp;
  mb_mapping->tab_registers[reg_SBP_MSG_IMU_AUX_imu_conf] = piksi_struct->IMU_AUX_data->imu_conf;

  
  return 0;
}


int runModBusTcpServer(piksi_data_t *piksi_struct)
{
  int socket;
  modbus_t *ctx;
  modbus_mapping_t *mb_mapping;
  int rc;
  int nPort = 502; // port number for modbus
  // we only need holding registers. will define a few of each other type just in case

  slog(0, SLOG_INFO, "ModbusTCP: creating modbus mapping");
  mb_mapping = modbus_mapping_new(1000, 1000, 1000, 1000);
  slog(0, SLOG_INFO, "ModbusTCP: modbus mapping complete");

  if (mb_mapping == NULL) {
      //fprintf(stderr, "Failed to allocate the mapping: %s\n", modbus_strerror(errno));
      slog(0, SLOG_ERROR, "ModbusTCP: Failed to allocate the mapping: %s", modbus_strerror(errno));
      //modbus_free(ctx);
      return -1;
  }

  slog(0, SLOG_INFO, "ModbusTCP: Waiting for TCP connection on Port %i",nPort);

  slog(0, SLOG_INFO, "ModbusTCP: binding to all tcp ports",nPort);
  ctx = modbus_new_tcp("0.0.0.0", nPort); // 0.0.0.0 means to listen on all IP addresses
  slog(0, SLOG_INFO, "ModbusTCP: all tcp ports bound",nPort);
  //modbus_set_debug(ctx, TRUE);
  
  socket = modbus_tcp_listen(ctx, 1);
  
  // immediately start waiting for a request
  modbus_tcp_accept(ctx, &socket);
  slog(0, SLOG_INFO, "ModbusTCP: TCP connection started!");
  
  for(;;) 
  {
    uint8_t query[MODBUS_TCP_MAX_ADU_LENGTH];

    rc = modbus_receive(ctx, query);
    if (rc >= 0) 
    {
      updateRegistersFromStruct(piksi_struct, mb_mapping); // request received. populate registers from structure
      modbus_reply(ctx, query, rc, mb_mapping); // respond to request
    } 
    else 
    {
      /* Connection closed by the client or server */
      //fprintf(stdout, "Con Closed.\n");
      slog(0, SLOG_INFO, "ModbusTCP: Con Closed.");
	    modbus_close(ctx); // close
	    // immediately start waiting for another request again
      modbus_tcp_accept(ctx, &socket);
      slog(0, SLOG_INFO, "ModbusTCP: TCP connection started!");
    }
  }


  modbus_mapping_free(mb_mapping);
  close(socket);
  modbus_free(ctx);
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

int runModBusTcpServerMulti(piksi_data_t *piksi_struct)
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

	// we only need holding registers. will define a few of each other type just in case

	slog(0, SLOG_INFO, "ModbusTCP: creating modbus mapping");
	mb_mapping = modbus_mapping_new(1000, 1000, 1000, 1000);
	slog(0, SLOG_INFO, "ModbusTCP: modbus mapping complete");

	if (mb_mapping == NULL) {
	  //fprintf(stderr, "Failed to allocate the mapping: %s\n", modbus_strerror(errno));
	  slog(0, SLOG_ERROR, "ModbusTCP: Failed to allocate the mapping: %s", modbus_strerror(errno));
	  //modbus_free(ctx);
	  return -1;
	}

    server_socket = modbus_tcp_listen(ctx, nConnections);
    if (server_socket == -1) {
        //fprintf(stderr, "Unable to listen TCP connection\n");
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
                    //printf("New connection from %s:%d on socket %d\n", inet_ntoa(clientaddr.sin_addr), clientaddr.sin_port, newfd);
					slog(0, SLOG_INFO, "ModbusTCP: New connection from %s:%d on socket %d", inet_ntoa(clientaddr.sin_addr), clientaddr.sin_port, newfd);
                }
            } else {
                modbus_set_socket(ctx, master_socket);
                rc = modbus_receive(ctx, query);
                if (rc > 0) {
					updateRegistersFromStruct(piksi_struct, mb_mapping); // request received. populate registers from structure
                    modbus_reply(ctx, query, rc, mb_mapping);
                } else if (rc == -1) {
                     // This example server in ended on connection closing or
                     // any errors. 
                    //printf("Connection closed on socket %d\n", master_socket);
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
  //fprintf(stderr, "usage: %s [-p serial port] [ -b baud rate] \n", prog_name);
  slog(0, SLOG_ERROR, "usage: %s [-p serial port] [ -b baud rate] ", prog_name);
}

void setup_socket(struct sockaddr_in server)
{
  // struct sockaddr_in server;
  socket_desc = socket(AF_INET , SOCK_STREAM | SOCK_NONBLOCK , 0); // changed it to be non-blocking sockets. unsure of the ramifications as things currently stand
  if (socket_desc == -1)
  {
    //fprintf(stderr, "Could not create socket\n");
	slog(0, SLOG_ERROR, "Could not create socket");
  }

  memset(&server, '0', sizeof(server));
  server.sin_addr.s_addr = inet_addr(tcp_ip_addr);
  server.sin_family = AF_INET;
  server.sin_port = htons(atoi(tcp_ip_port));

  
  if (connect(socket_desc, (struct sockaddr *)&server , sizeof(server)) < 0)
  {
    //fprintf(stderr, "Connection error\n");
	slog(0, SLOG_ERROR, "Connection error");
  }
  
}

void close_socket()
{
  close(socket_desc);
}
/*
void setup_port()
{
  int result;

  result = sp_set_baudrate(piksi_port, 115200);
  if (result != SP_OK) {
    //fprintf(stderr, "Cannot set port baud rate!\n");
    slog(0, SLOG_ERROR, "Cannot set port baud rate!");
    exit(EXIT_FAILURE);
  }

  result = sp_set_flowcontrol(piksi_port, SP_FLOWCONTROL_NONE);
  if (result != SP_OK) {
    //fprintf(stderr, "Cannot set flow control!\n");
    slog(0, SLOG_ERROR, "Cannot set flow control!");
    exit(EXIT_FAILURE);
  }

  result = sp_set_bits(piksi_port, 8);
  if (result != SP_OK) {
    //fprintf(stderr, "Cannot set data bits!\n");
    slog(0, SLOG_ERROR, "Cannot set data bits!");
    exit(EXIT_FAILURE);
  }

  result = sp_set_parity(piksi_port, SP_PARITY_NONE);
  if (result != SP_OK) {
    //fprintf(stderr, "Cannot set parity!\n");
    slog(0, SLOG_ERROR, "Cannot set parity!");
    exit(EXIT_FAILURE);
  }

  result = sp_set_stopbits(piksi_port, 1);
  if (result != SP_OK) {
    //fprintf(stderr, "Cannot set stop bits!\n");
    slog(0, SLOG_ERROR, "Cannot set stop bits!");
    exit(EXIT_FAILURE);
  }
}
*/
s32 socket_read(u8 *buff, u32 n, void *context)
{
  (void)context;
  s32 result;

  result = read(socket_desc, buff, n);
  return result;
}

/*s32 piksi_port_read(u8 *buff, u32 n, void *context)
{
  (void)context;
  s32 result;

  result = sp_blocking_read(piksi_port, buff, n, 0);

  return result;
}
*/
void sbp_setup_all()
{

  sbp_state_init(&sbp_state);

  // register callback functions for each sbp message we wish to process
  
  if (blnHeartbeatEnabled)  {
	sbp_register_callback(&sbp_state, SBP_MSG_HEARTBEAT, &heartbeat_callback, (void*)CurrentData,
						&heartbeat_callback_node);
	slog(0, SLOG_INFO, "registering heartbeat_callback");
  }

  if (blnBasePosEnabled) {
	sbp_register_callback(&sbp_state, SBP_MSG_BASE_POS_LLH, &base_pos_llh_callback, (void*)CurrentData,
						&base_pos_llh_callback_node);
	slog(0, SLOG_INFO, "registering base_pos_llh_callback");
  }

  if (blnLLHPosEnabled) {
	sbp_register_callback(&sbp_state, SBP_MSG_POS_LLH, &pos_llh_callback, (void*)CurrentData,
						&pos_llh_callback_node);
	slog(0, SLOG_INFO, "registering pos_llh_callback");
  }

  if (blnBaseNEDEnabled) {
	sbp_register_callback(&sbp_state, SBP_MSG_BASELINE_NED, &baseline_ned_callback, (void*)CurrentData,
						  &baseline_ned_callback_node);
	slog(0, SLOG_INFO, "registering baseline_ned_callback");
  }
  if (blnVelNEDEnabled) {
	sbp_register_callback(&sbp_state, SBP_MSG_VEL_NED, &vel_ned_callback, (void*)CurrentData,
						  &vel_ned_callback_node);
	slog(0, SLOG_INFO, "registering vel_ned_callback");
  }

  if (blnGPSTimeEnabled) {
	sbp_register_callback(&sbp_state, SBP_MSG_GPS_TIME, &gps_time_callback, (void*)CurrentData,
						&gps_time_callback_node);
	//fprintf(stdout, "registering gps_time_callback\n");
	slog(0, SLOG_INFO, "registering gps_time_callback");
  }
						
  if (blnUTCTimeEnabled) {
	sbp_register_callback(&sbp_state, SBP_MSG_UTC_TIME, &utc_time_callback, (void*)CurrentData,
						&utc_time_callback_node);
	slog(0, SLOG_INFO, "registering utc_time_callback");
  }

  if (blnIMUEnabled) {
	sbp_register_callback(&sbp_state, SBP_MSG_IMU_RAW, &imu_raw_callback, (void*)CurrentData,
						  &imu_raw_callback_node);
	slog(0, SLOG_INFO, "registering imu_raw_callback");
  }

  if (blnECEFEnabled) {
	sbp_register_callback(&sbp_state, SBP_MSG_BASELINE_ECEF, &baseline_ecef_callback, (void*)CurrentData,
						  &baseline_ecef_callback_node);
	//fprintf(stdout, "registering baseline_ecef_callback\n");
	slog(0, SLOG_INFO, "registering baseline_ecef_callback");
  }

  if (blnECEFEnabled) {
	sbp_register_callback(&sbp_state, SBP_MSG_POS_ECEF, &pos_ecef_callback, (void*)CurrentData,
						  &pos_ecef_callback_node);
	//fprintf(stdout, "registering pos_ecef_callback\n");
	slog(0, SLOG_INFO, "registering pos_ecef_callback");
  }

  if (blnECEFEnabled) {
	sbp_register_callback(&sbp_state, SBP_MSG_LOG, &log_callback, (void*)CurrentData,
						  &log_callback_node);
	slog(0, SLOG_INFO, "registering log_callback_node");
  }
  
  if (blnECEFEnabled) {
	sbp_register_callback(&sbp_state, SBP_MSG_DEVICE_MONITOR, &device_monitor_callback, (void*)CurrentData,
						  &device_monitor_callback_node);
	slog(0, SLOG_INFO, "registering device_monitor_callback_node");
  }
  
  if (blnECEFEnabled) {
	sbp_register_callback(&sbp_state, SBP_MSG_LINUX_SYS_STATE, &linux_sys_callback, (void*)CurrentData,
						  &linux_sys_callback_node);
	slog(0, SLOG_INFO, "registering linux_sys_callback_node");
  }
  
  if (blnECEFEnabled) {
	sbp_register_callback(&sbp_state, SBP_MSG_AGE_CORRECTIONS, &correction_age_callback, (void*)CurrentData,
						  &correction_age_callback_node);
	slog(0, SLOG_INFO, "registering correction_age_callback_node");
  }
  if (blnIMUEnabled) {
	sbp_register_callback(&sbp_state, SBP_MSG_IMU_RAW, &imu_aux_callback, (void*)CurrentData,
						  &imu_aux_callback_node);
	slog(0, SLOG_INFO, "registering imu_aux_callback");
  }

  
  
  
}



int main(int argc, char **argv)
{
  int opt;
//  int result = 0;
  int intBaudRate = 115200;
  char blnEthernetComms = 1;
  time_t tmCurrentTime;

  struct sockaddr_in server;

  blnGPSTimeEnabled = 1;
  blnIMUEnabled = 1;
  blnUTCTimeEnabled = 1;
  blnBasePosEnabled = 1;
  blnLLHPosEnabled = 1;
  blnBaseNEDEnabled = 1;
  blnECEFEnabled = 1;
  blnVelNEDEnabled = 1;
  blnHeartbeatEnabled = 1;
  
  
  // fLogFile = fopen("outfile.txt", "w"); // write only 
  
  tmLastHeartbeat = time(NULL); // initialise last heartbeat to a time value, so first time comparison doesn't fail 
  
  time_t tmLastHeartbeatOKMessage = time(NULL); // temp variable to output diagnosis messages every 5 seconds
  
  blnDebugToScreen = 1;
  slog_init("plc-gps-interface", "slog.cfg", 1, 1);
//  slog_init("plc-gps-interface", "slog.cfg", 1, 3, 1);
  slog(0, SLOG_INFO, "Opening logging file");
  
  
  int parpid = getpid(), childpid;
  //fprintf(stdout, "parent processID=%d\n", parpid);
  slog(0, SLOG_INFO, "parent processID=%d", parpid);
  //piksi_data_t *CurrentData = (piksi_data_t *)mmap(NULL, sizeof(*CurrentData), PROT_READ|PROT_WRITE, MAP_ANONYMOUS|MAP_SHARED, -1, 0);
  CurrentData = (piksi_data_t *)mmap(NULL, sizeof(*CurrentData), PROT_READ|PROT_WRITE, MAP_ANONYMOUS|MAP_SHARED, -1, 0);
  piksi_data_setup(CurrentData);
  
  if (argc <= 1) {
    usage(argv[0]);
    exit(EXIT_FAILURE);
  }

  while ((opt = getopt(argc, argv, "p:b:gufinvlead")) != -1) // get arguments
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
        //fprintf(stdout, "GPS Time disabled\n");
        slog(0, SLOG_INFO, "GPS Time disabled");
        break;
      case 'u': // if parameter is in existence, then disable UTC Time callback function
        blnUTCTimeEnabled = 0; // disable UTC Time callback function
        //fprintf(stdout, "UTC Time disabled\n");
        slog(0, SLOG_INFO, "UTC Time disabled");
        break;
      case 'l': // if parameter is in existence, then disable LLH position callback function
        blnLLHPosEnabled = 0; // disable LLH position callback function
        //fprintf(stdout, "LLH position data collection disabled\n");
        slog(0, SLOG_INFO, "LLH position data collection disabled");
        break;
      case 'f': // if parameter is in existence, then disable base position callback function
        blnBasePosEnabled = 0; // disable base position callback function
        //fprintf(stdout, "base position data collection disabled\n");
        slog(0, SLOG_INFO, "base position data collection disabled");
        break;
      case 'i': // if parameter is in existence, then disable IMU callback function
        blnIMUEnabled = 0; // disable IMU callback function
        //fprintf(stdout, "IMU data collection disabled\n");
        slog(0, SLOG_INFO, "IMU data collection disabled");
        break;
      case 'n': // if parameter is in existence, then disable baseline rover NED callback function
        blnBaseNEDEnabled = 0; // disable baseline rover NED callback function
        //fprintf(stdout, "Baseline NED Rover coordinate collection disabled\n");
        slog(0, SLOG_INFO, "Baseline NED Rover coordinate collection disabled");
        break;
      case 'e': // if parameter is in existence, then disable all ECEF callback functions
        blnECEFEnabled = 0; // disable disable all ECEF callback functions
        //fprintf(stdout, "all ECEF coordinate collection disabled\n");
        slog(0, SLOG_INFO, "all ECEF coordinate collection disabled");
        break;
      case 'v': // if parameter is in existence, then disable velocity rover NED callback function
        blnVelNEDEnabled = 0; // disable velocity rover NED callback function
        //fprintf(stdout, "NED velocity collection disabled\n");
        slog(0, SLOG_INFO, "NED velocity collection disabled");
        break;
      case 'a': // if parameter is in existence, then disable heartbeat callback function
        blnHeartbeatEnabled = 0; // disable heartbeat callback function
        //fprintf(stdout, "heartbeat collection disabled\n");
        slog(0, SLOG_INFO, "heartbeat collection disabled");
        break;
      case 'd': // if parameter is in existence, then send info and debug messages to screen
        blnDebugToScreen = 13; // enable debug to screen
        //fprintf(stdout, "info and debug messages enabled to screen\n");
        slog(0, SLOG_INFO, "info and debug messages enabled to screen");
        break;
//      case 'h':
//        usage(argv[0]);
//        exit(EXIT_FAILURE);
    }
  }

  switch ((childpid = fork()))  // start separate process for ModbusTCP server
  {
  case -1:
    err(1, "fork");
    // NOTREACHED
  case 0: // ******************************** in the child process ********************************
    childpid = getpid();
    //fprintf(stdout, "child PID %d\n", childpid);
    slog(0, SLOG_INFO, "ModbusTCP:child PID %d", childpid);
        
    //runModBusTcpServer(CurrentData);
    runModBusTcpServerMulti(CurrentData);
    
    //fprintf(stdout, "terminating child process PID %d\n", childpid);
    slog(0, SLOG_INFO, "ModbusTCP: terminating child process PID %d", childpid);
        
    piksi_data_close(CurrentData);
    return EXIT_SUCCESS;
  } // end of child process
  


  if (blnEthernetComms) {
    
    if (!tcp_ip_addr) {
      //fprintf(stderr, "Please supply the IP address of the SBP data stream!\n");
      slog(0, SLOG_ERROR, "Please supply the IP address of the SBP data stream!");
      exit(EXIT_FAILURE);
    }

    if (!tcp_ip_port) {
      //fprintf(stderr, "Please supply the IP port of the SBP data stream!\n");
      slog(0, SLOG_ERROR, "Please supply the IP port of the SBP data stream!");
      exit(EXIT_FAILURE);
    }
    
    setup_socket(server);
  }
/*  else {
	  
  
    if (!serial_port_name) {
      //fprintf(stderr, "Please supply the serial port path where the Piksi is connected!\n");
      slog(0, SLOG_ERROR, "Please supply the serial port path where the Piksi is connected!");
          
      exit(EXIT_FAILURE);
    }

    result = sp_get_port_by_name(serial_port_name, &piksi_port);
    if (result != SP_OK) {
      //fprintf(stderr, "Cannot find provided serial port!\n");
      slog(0, SLOG_ERROR, "Cannot find provided serial port!");
      exit(EXIT_FAILURE);
    }

    result = sp_open(piksi_port, SP_MODE_READ);
    if (result != SP_OK) {
      //fprintf(stderr, "Cannot open %s for reading!\n", serial_port_name);
      slog(0, SLOG_ERROR, "Cannot open %s for reading!", serial_port_name);
      exit(EXIT_FAILURE);
    }

    setup_port();
}
*/  
  sbp_setup_all();

  
  if (blnEthernetComms) {
    while(1) {
	  

      tmCurrentTime = time(NULL);
      if ((blnHeartbeatEnabled) &&(tmCurrentTime - tmLastHeartbeat > HeartBeatTimeout) ) // heartbeat enabled and we haven't received a heartbeat in x seconds..
      {

        slog(0, SLOG_INFO, "Heartbeat not detected in %ld seconds", (unsigned long)(tmCurrentTime - tmLastHeartbeat));
        // fprintf(stderr, "Heartbeat not detected in %ld seconds\n", (unsigned long)(tmCurrentTime - tmLastHeartbeat));

        // if a heartbeat is not detected, its likely comms is lost to the GPS device. 
        // attempt to setup the socket again
        slog(0, SLOG_INFO, "Attempt to reconnect socket");
        close_socket();
		
		setup_socket(server);
        slog(0, SLOG_INFO, "Setup socket command issued.");
        sbp_setup_all();
        tmLastHeartbeat = time(NULL);
      }
	  else
	  {
         if (( tmCurrentTime - tmLastHeartbeatOKMessage > OKMessageInterval) && (tmCurrentTime - tmLastSuccessfulRead < OKMessageInterval))
		 {
            slog(2, SLOG_INFO, "Connection to GPS OK. Received a messages within the last %d seconds. so only showing every %d seconds", OKMessageInterval, OKMessageInterval);
			tmLastHeartbeatOKMessage = time(NULL);
		 }
	  }
      

      sbp_process(&sbp_state, &socket_read); // process requests from Piksi Multi over Ethernet
      usleep(100); // sleep for a millisecond
	  
    }
    close_socket();
  }  
/*  else {
    while(1) {
      sbp_process(&sbp_state, &piksi_port_read); // process requests from Piksi Multi over serial
    }
  }
*/

  
  piksi_data_close(CurrentData);
  // fclose(fLogFile);
  //free(tcp_ip_addr);
  //free(tcp_ip_port);
  //free(serial_port_name);
  return 0;
  }
