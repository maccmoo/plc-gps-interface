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
 #include <libsbp/piksi.h>
 #include <libsbp/linux.h>
 #include <slog.h>
 #include "cJSON.h"
 #include <time.h>

typedef struct {
  // see definition of SBP protocol at https://github.com/swift-nav/libsbp/blob/master/docs/sbp.pdf

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
  msg_baseline_heading_dep_a_t *baseline_heading_data;
  msg_age_corrections_t *age_corrections_data;
  msg_base_pos_llh_t *base_position_llh_data;
  msg_base_pos_ecef_t *base_position_ecef_data;
  msg_dgnss_status_t *dgnss_status;
  msg_heartbeat_t *heartbeat_data;
  msg_imu_raw_t *IMU_data;
  msg_utc_time_t *UTC_data;
  msg_log_t *log_data;
  msg_device_monitor_t *device_monitor_data;
  msg_linux_sys_state_t *linux_sys_data;
  msg_age_corrections_t *correction_age_data;
  msg_imu_aux_t *IMU_AUX_data;
  
} piksi_data_t;

int piksi_data_setup(piksi_data_t *piksidata);
int piksi_data_close(piksi_data_t *piksidata);
extern FILE * fp;
extern FILE *fLogFile;
extern char blnDebugToScreen;
extern time_t tmLastHeartbeat1;
extern time_t tmLastHeartbeat2;
extern time_t tmLastHeartbeat3;
extern time_t tmLastHeartbeat4;
extern time_t tmLastHeartbeat5;
extern time_t tmLastSuccessfulRead1;
extern time_t tmLastSuccessfulRead2;
extern time_t tmLastSuccessfulRead3;
extern time_t tmLastSuccessfulRead4;
extern time_t tmLastSuccessfulRead5;

void heartbeat_callback1(u16 sender_id, u8 len, u8 msg[], void *context);
void heartbeat_callback2(u16 sender_id, u8 len, u8 msg[], void *context);
void heartbeat_callback3(u16 sender_id, u8 len, u8 msg[], void *context);
void heartbeat_callback4(u16 sender_id, u8 len, u8 msg[], void *context);
void heartbeat_callback5(u16 sender_id, u8 len, u8 msg[], void *context);

void pos_llh_callback(u16 sender_id, u8 len, u8 msg[], void *context);

void baseline_ned_callback(u16 sender_id, u8 len, u8 msg[], void *context);

void vel_ned_callback(u16 sender_id, u8 len, u8 msg[], void *context);

void base_pos_llh_callback(u16 sender_id, u8 len, u8 msg[], void *context);

void gps_time_callback(u16 sender_id, u8 len, u8 msg[], void *context);

void utc_time_callback(u16 sender_id, u8 len, u8 msg[], void *context);

void imu_raw_callback(u16 sender_id, u8 len, u8 msg[], void *context);

void baseline_ecef_callback(u16 sender_id, u8 len, u8 msg[], void *context);

void pos_ecef_callback(u16 sender_id, u8 len, u8 msg[], void *context);

void log_callback(u16 sender_id, u8 len, u8 msg[], void *context);

void device_monitor_callback(u16 sender_id, u8 len, u8 msg[], void *context);

void linux_sys_callback(u16 sender_id, u8 len, u8 msg[], void *context);

void correction_age_callback(u16 sender_id, u8 len, u8 msg[], void *context);

void imu_aux_callback(u16 sender_id, u8 len, u8 msg[], void *context);




#endif
