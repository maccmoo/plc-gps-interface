#include "sbp_callback_functions.h"

#include <stdio.h>
// #include <stdlib.h>
// #include <string.h>
// #include <unistd.h>

// #include <libserialport.h>

// #include <libsbp/sbp.h>
// #include <libsbp/system.h>
// #include <libsbp/observation.h>
// #include <libsbp/navigation.h>
// #include <libsbp/imu.h>



void heartbeat_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  (void)sender_id, (void)len, (void)msg, (void)context;
  fprintf(stdout, "%s\n", __FUNCTION__);
}

void pos_llh_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  (void)sender_id, (void)len, (void)msg, (void)context;
    int i, intExpectedLength=34;
  
  msg_pos_llh_t *pos_llh_struct;
  
  if (len != intExpectedLength)
  {
    fprintf(stdout, "Message received was of invalid length: len=%d\n", len);
  }
  else
  {
    pos_llh_struct = (msg_pos_llh_t *) msg; // cast msg pointer to appropriate type and save for later use
    for(i=0;i<len;i++)
    {
      //fprintf(stdout, "%02x ", msg[i]); // loop through msg and print out data as hex
    }
    // output items in structure
    fprintf(stdout, "tow = %d ms; ", pos_llh_struct -> tow);
    fprintf(stdout, "-> latitude/longitude/height = %lf/%lf/%lf ", pos_llh_struct -> lat,  pos_llh_struct -> lon,  pos_llh_struct -> height);
  }
  fprintf(stdout, "%s\n", __FUNCTION__);
}

void baseline_ned_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  (void)sender_id, (void)len, (void)msg, (void)context;
    int i, intExpectedLength=22;
  
  msg_baseline_ned_t *baseline_ned_struct;
  
  if (len != intExpectedLength)
  {
    fprintf(stdout, "Message received was of invalid length: len=%d\n", len);
  }
  else
  {
    baseline_ned_struct = (msg_baseline_ned_t *) msg; // cast msg pointer to appropriate type and save for later use
    for(i=0;i<len;i++)
    {
      //fprintf(stdout, "%02x ", msg[i]); // loop through msg and print out data as hex
    }
    // output items in structure
    fprintf(stdout, "tow = %d ms; ", baseline_ned_struct -> tow);
    fprintf(stdout, "-> north/east/down = %d/%d/%d ", baseline_ned_struct -> n,  baseline_ned_struct -> e,  baseline_ned_struct -> d);
    fprintf(stdout, "-> h_acc/v_acc/#sats = %d/%d/%d ", baseline_ned_struct -> h_accuracy,  baseline_ned_struct -> v_accuracy,  baseline_ned_struct -> n_sats);
    if (baseline_ned_struct -> flags | 0x04){
      fprintf(stdout, "-> src=fixed RTK ");
    }
  }
  fprintf(stdout, "%s\n", __FUNCTION__);
}

void vel_ned_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  (void)sender_id, (void)len, (void)msg, (void)context;
    int i, intExpectedLength=22;
  
  msg_vel_ned_t *vel_ned_struct;
  
  if (len != intExpectedLength)
  {
    fprintf(stdout, "Message received was of invalid length: len=%d\n", len);
  }
  else
  {
    vel_ned_struct = (msg_vel_ned_t *) msg; // cast msg pointer to appropriate type and save for later use
    for(i=0;i<len;i++)
    {
      //fprintf(stdout, "%02x ", msg[i]); // loop through msg and print out data as hex
    }
    // output items in structure
    fprintf(stdout, "tow = %d ms; ", vel_ned_struct -> tow);
    fprintf(stdout, "-> north_vel/east_vel/down_vel = %d/%d/%d ", vel_ned_struct -> n,  vel_ned_struct -> e,  vel_ned_struct -> d);
    fprintf(stdout, "-> h_acc/v_acc/#sats = %d/%d/%d ", vel_ned_struct -> h_accuracy,  vel_ned_struct -> v_accuracy,  vel_ned_struct -> n_sats);
    switch (vel_ned_struct -> flags)
    {
      case 0x00:
        fprintf(stdout, "-> src=invalid ");
        break;
      case 0x01:
        fprintf(stdout, "-> src=Measured Doppler derived ");
        break;
      case 0x02:
        fprintf(stdout, "-> src=Computed Doppler derived ");
        break;
    }    
  }
  fprintf(stdout, "%s\n", __FUNCTION__);
}

void base_pos_llh_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  (void)sender_id, (void)len, (void)msg, (void)context;
    int i, intExpectedLength=24;
  
  msg_base_pos_llh_t *base_pos_llh_struct;
  
  if (len != intExpectedLength)
  {
    fprintf(stdout, "Message received was of invalid length: len=%d\n", len);
  }
  else
  {
    base_pos_llh_struct = (msg_base_pos_llh_t *) msg; // cast msg pointer to appropriate type and save for later use
    for(i=0;i<len;i++)
    {
      //fprintf(stdout, "%02x ", msg[i]); // loop through msg and print out data as hex
    }
    // output items in structure
    fprintf(stdout, "-> latitude/longitude/height = %lf/%lf/%lf ", base_pos_llh_struct -> lat,  base_pos_llh_struct -> lon,  base_pos_llh_struct -> height);
  }
  fprintf(stdout, "%s\n", __FUNCTION__);
}

void gps_time_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  (void)sender_id, (void)len, (void)msg, (void)context;
  
  
  
  int i, intExpectedLength=11;
  
  msg_gps_time_t *gps_time_struct;
  
  if (len != intExpectedLength)
  {
    fprintf(stdout, "Message received was of invalid length: len=%d\n", len);
  }
  else
  {
    gps_time_struct = (msg_gps_time_t *) msg; // cast msg pointer to appropriate type and save for later use
    for(i=0;i<len;i++)
    {
      fprintf(stdout, "%02x ", msg[i]); // loop through msg and print out data as hex
    }
    // output items in structure
    fprintf(stdout, "-> # weeks = %d, ", gps_time_struct -> wn);
    fprintf(stdout, "tow = %dms, ", gps_time_struct -> tow);
    fprintf(stdout, "residual nanosecond = %dns, ", gps_time_struct -> ns_residual);
    fprintf(stdout, "flags = %d: ", gps_time_struct -> flags);
    
  }
  fprintf(stdout, "%s\n", __FUNCTION__); // output function name
}

void utc_time_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  (void)sender_id, (void)len, (void)msg, (void)context;
  int i, intExpectedLength=16;
  
  msg_utc_time_t *utc_time_struct;
  
  if (len != intExpectedLength)
  {
    fprintf(stdout, "Message received was of invalid length: len=%d\n", len);
  }
  else
  {
    utc_time_struct = (msg_utc_time_t *) msg; // cast msg pointer to appropriate type and save for later use
    for(i=0;i<len;i++)
    {
      //fprintf(stdout, "%02x ", msg[i]); // loop through msg and print out data as hex
    }
    // output items in structure
    fprintf(stdout, "-> flags = %d, ", utc_time_struct -> flags);
    fprintf(stdout, "tow = %d, ", utc_time_struct -> tow);
    fprintf(stdout, "date = %d/%d/%d %d:%d:%d ", utc_time_struct -> year, utc_time_struct -> month, utc_time_struct -> day, utc_time_struct -> hours, utc_time_struct -> minutes, utc_time_struct -> seconds );
    fprintf(stdout, "nanoseconds = %10dns : ", utc_time_struct -> ns);
    
  }
  fprintf(stdout, "%s\n", __FUNCTION__); // output function name
}

void imu_raw_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  (void)sender_id, (void)len, (void)msg, (void)context;
  int i, intExpectedLength=17;
  
  msg_imu_raw_t *imu_raw_struct;
  
  if (len != intExpectedLength)
  {
    fprintf(stdout, "Message received was of invalid length: len=%d\n", len);
  }
  else
  {
    imu_raw_struct = (msg_imu_raw_t *) msg; // cast msg pointer to appropriate type and save for later use
    for(i=0;i<len;i++)
    {
      //fprintf(stdout, "%02x ", msg[i]); // loop through msg and print out data as hex
    }
    // output items in structure
    fprintf(stdout, "tow = %d ms, %d; ", imu_raw_struct -> tow, imu_raw_struct -> tow_f);
    fprintf(stdout, "acc x/y/z %d/%d/%d gyr x/y/z %d/%d/%d ", imu_raw_struct -> acc_x, imu_raw_struct -> acc_y, imu_raw_struct -> acc_z, imu_raw_struct -> gyr_x, imu_raw_struct -> gyr_y, imu_raw_struct -> gyr_z );
    //fprintf(stdout, "nanoseconds = %10dns : ", imu_raw_struct -> ns);
  }
  fprintf(stdout, "%s\n", __FUNCTION__); // output function name
}
