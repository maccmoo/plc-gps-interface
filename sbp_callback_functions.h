#ifndef SBPCALLBACK
#define SBPCALLBACK

 #include <stdio.h>
 #include <stdlib.h>
 #include <string.h>
 #include <sys/mman.h>
 // #include <unistd.h>
// #include <libserialport.h>

 #include <libsbp/sbp.h>
 #include <libsbp/system.h>
 #include <libsbp/logging.h>
 #include <libsbp/observation.h>
 #include <libsbp/navigation.h>
 #include <libsbp/imu.h>
 #include <slog.h>
 #include "cJSON.h"

typedef struct {
  // see definition of SBP protocol at https://github.com/swift-nav/libsbp/blob/master/docs/sbp.pdf

  // populated from SBP message MSG_GPS_TIME
  // uint16_t weeks; // # of weeks since unix epoch MSG_GPS_TIME
  // uint32_t tow; // time of week. 
  // int32_t ns_residual; // residual nanoseconds for precise time
  
  msg_startup_t *STARTUP_data;
  msg_log_t *LOG_data;
  msg_gps_time_t *GPS_time_data;
  msg_dops_t * *DOPS_data;
  msg_pos_ecef_t *ECEF_data;
  msg_pos_llh_t *LLH_data;
  msg_baseline_ecef_t *baseline_ECEF_data;
  msg_baseline_ned_t *baseline_NED_data;
  msg_vel_ecef_t *ECEF_velocity_data;
  msg_vel_ned_t *NED_velocity_data;
  msg_baseline_heading_t *baseline_heading_data;
  msg_age_corrections_t *age_corrections_data;
  msg_base_pos_llh_t *base_position_llh_data;
  msg_base_pos_ecef_t *base_position_ecef_data;
  msg_dgnss_status_t *dgnss_status;
  msg_heartbeat_t *heartbeat_data;
  msg_imu_raw_t *IMU_data;
  
  
} piksi_data_t;

int piksi_data_setup(piksi_data_t *piksidata);
int piksi_data_close(piksi_data_t *piksidata);
FILE * fp;
FILE *fLogFile;


void heartbeat_callback(u16 sender_id, u8 len, u8 msg[], void *context);

void pos_llh_callback(u16 sender_id, u8 len, u8 msg[], void *context);

void baseline_ned_callback(u16 sender_id, u8 len, u8 msg[], void *context);

void vel_ned_callback(u16 sender_id, u8 len, u8 msg[], void *context);

void base_pos_llh_callback(u16 sender_id, u8 len, u8 msg[], void *context);

void gps_time_callback(u16 sender_id, u8 len, u8 msg[], void *context);

void utc_time_callback(u16 sender_id, u8 len, u8 msg[], void *context);

void imu_raw_callback(u16 sender_id, u8 len, u8 msg[], void *context);


#endif