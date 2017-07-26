#ifndef SBPCALLBACK
#define SBPCALLBACK

// #include <stdio.h>
// #include <stdlib.h>
// #include <string.h>
// #include <unistd.h>

// #include <libserialport.h>

 #include <libsbp/sbp.h>
 #include <libsbp/system.h>
 #include <libsbp/observation.h>
 #include <libsbp/navigation.h>
 #include <libsbp/imu.h>

void heartbeat_callback(u16 sender_id, u8 len, u8 msg[], void *context);

void pos_llh_callback(u16 sender_id, u8 len, u8 msg[], void *context);

void baseline_ned_callback(u16 sender_id, u8 len, u8 msg[], void *context);

void base_pos_llh_callback(u16 sender_id, u8 len, u8 msg[], void *context);

void gps_time_callback(u16 sender_id, u8 len, u8 msg[], void *context);

void utc_time_callback(u16 sender_id, u8 len, u8 msg[], void *context);

void imu_raw_callback(u16 sender_id, u8 len, u8 msg[], void *context);


#endif