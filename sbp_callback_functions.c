#include "sbp_callback_functions.h"

int piksi_data_setup(piksi_data_t *piksidata)
{
  //piksidata = calloc(1, sizeof (*piksidata));
  
    piksi_data_t *CurrentData = (piksi_data_t *)mmap(NULL, sizeof(*CurrentData), PROT_READ|PROT_WRITE, MAP_ANONYMOUS|MAP_SHARED, -1, 0);
  //piksi_data_t *CurrentData = calloc(1, sizeof(*CurrentData));

  
  piksidata->GPS_time_data = (msg_gps_time_t *)mmap(NULL, sizeof(piksidata->GPS_time_data), PROT_READ|PROT_WRITE, MAP_ANONYMOUS|MAP_SHARED, -1, 0);
  piksidata->NED_data = (msg_baseline_ned_t *)mmap(NULL, sizeof(piksidata->NED_data), PROT_READ|PROT_WRITE, MAP_ANONYMOUS|MAP_SHARED, -1, 0);
  piksidata->LLH_data = (msg_pos_llh_t *)mmap(NULL, sizeof(piksidata->LLH_data), PROT_READ|PROT_WRITE, MAP_ANONYMOUS|MAP_SHARED, -1, 0);
  piksidata->NED_velocity_data = (msg_vel_ned_t *)mmap(NULL, sizeof(piksidata->NED_velocity_data), PROT_READ|PROT_WRITE, MAP_ANONYMOUS|MAP_SHARED, -1, 0);
  piksidata->IMU_data = (msg_imu_raw_t *)mmap(NULL, sizeof(piksidata->IMU_data), PROT_READ|PROT_WRITE, MAP_ANONYMOUS|MAP_SHARED, -1, 0);

  // piksidata->GPS_time_data = calloc(1, sizeof(piksidata-> GPS_time_data));
  // piksidata->NED_data = calloc(1, sizeof(piksidata-> NED_data));
  // piksidata->LLH_data = calloc(1, sizeof(piksidata-> LLH_data));
  // piksidata->NED_velocity_data = calloc(1, sizeof(piksidata-> NED_velocity_data));
  // piksidata->IMU_data = calloc(1, sizeof(piksidata-> IMU_data));

  
  fprintf(stdout, "piksi_data_setup complete. totalsize = %d bytes\n", (int)(sizeof (*piksidata) + sizeof(piksidata-> GPS_time_data) + sizeof(piksidata-> NED_data) + sizeof(piksidata-> LLH_data) + sizeof(piksidata-> NED_velocity_data) + sizeof(piksidata-> IMU_data)) );
  
  return 0; // need to add error code here
}

int piksi_data_close(piksi_data_t *piksidata)
{
  
  munmap(piksidata->GPS_time_data, sizeof(piksidata->GPS_time_data));
  munmap(piksidata->NED_data, sizeof(piksidata->NED_data));
  munmap(piksidata->LLH_data, sizeof(piksidata->LLH_data));
  munmap(piksidata->NED_velocity_data, sizeof(piksidata->NED_velocity_data));
  munmap(piksidata->IMU_data, sizeof(piksidata->NED_velocity_data));

  munmap(piksidata, sizeof(piksidata));

  // free(piksidata->GPS_time_data);
  // free(piksidata->NED_data);
  // free(piksidata->LLH_data);
  // free(piksidata->NED_velocity_data);
  // free(piksidata->IMU_data);

  // free(piksidata);

  return 0; // need to add error code here
}

// currently no data in this fucntion. just used for diagnostics atm.
void heartbeat_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  (void)sender_id, (void)len, (void)msg, (void)context;

  //piksi_data_t *piksi_data= (piksi_data_t*)context;

  fprintf(stdout, "%s\n", __FUNCTION__);
}

void pos_llh_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  (void)sender_id, (void)len, (void)msg, (void)context;

  piksi_data_t *piksi_data= (piksi_data_t*)context;
  msg_pos_llh_t *pos_llh_struct;

  pos_llh_struct = (msg_pos_llh_t *) msg; // cast msg pointer to appropriate type and save for later use
  
  memcpy( piksi_data-> LLH_data, pos_llh_struct, sizeof(*pos_llh_struct));

  // output items in structure
  fprintf(stdout, "tow = %d ms; ", pos_llh_struct -> tow);
  fprintf(stdout, "-> latitude/longitude/height = %lf/%lf/%lf ", pos_llh_struct -> lat,  pos_llh_struct -> lon,  pos_llh_struct -> height);

  fprintf(stdout, "%s\n", __FUNCTION__);
}

void baseline_ned_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  (void)sender_id, (void)len, (void)msg, (void)context;

  piksi_data_t *piksi_data= (piksi_data_t*)context;
  msg_baseline_ned_t *baseline_ned_struct;
  
  baseline_ned_struct = (msg_baseline_ned_t *) msg; // cast msg pointer to appropriate type and save for later use
  
  memcpy( piksi_data-> NED_data, baseline_ned_struct, sizeof(*baseline_ned_struct));

  // output items in structure
  fprintf(stdout, "tow = %d ms; ", baseline_ned_struct -> tow);
  fprintf(stdout, "-> north/east/down = %d/%d/%d ", baseline_ned_struct -> n,  baseline_ned_struct -> e,  baseline_ned_struct -> d);
  fprintf(stdout, "-> h_acc/v_acc/#sats = %d/%d/%d ", baseline_ned_struct -> h_accuracy,  baseline_ned_struct -> v_accuracy,  baseline_ned_struct -> n_sats);
  if (baseline_ned_struct -> flags | 0x04){
    fprintf(stdout, "-> src=fixed RTK ");
  }

  fprintf(stdout, "%s\n", __FUNCTION__);
}

void vel_ned_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  (void)sender_id, (void)len, (void)msg, (void)context;

  piksi_data_t *piksi_data= (piksi_data_t*)context;
  msg_vel_ned_t *vel_ned_struct;

  vel_ned_struct = (msg_vel_ned_t *) msg; // cast msg pointer to appropriate type and save for later use
  
  memcpy( piksi_data-> NED_velocity_data, vel_ned_struct, sizeof(*vel_ned_struct));

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
  
  fprintf(stdout, "%s\n", __FUNCTION__);
}

// not currently any useful data stored in this packet.
void base_pos_llh_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  (void)sender_id, (void)len, (void)msg, (void)context;
  
  //piksi_data_t *piksi_data= (piksi_data_t*)context;
  msg_base_pos_llh_t *base_pos_llh_struct;

  base_pos_llh_struct = (msg_base_pos_llh_t *) msg; // cast msg pointer to appropriate type and save for later use
  
  //memcpy( piksi_data-> ?????, base_pos_llh_struct, sizeof(*base_pos_llh_struct));

  // output items in structure
  fprintf(stdout, "-> latitude/longitude/height = %lf/%lf/%lf ", base_pos_llh_struct -> lat,  base_pos_llh_struct -> lon,  base_pos_llh_struct -> height);

  fprintf(stdout, "%s\n", __FUNCTION__);
}

void gps_time_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  (void)sender_id, (void)len, (void)msg, (void)context;
  
  piksi_data_t *piksi_data= (piksi_data_t*)context;
  msg_gps_time_t *gps_time_struct;

  gps_time_struct = (msg_gps_time_t *) msg; // cast msg pointer to appropriate type and save for later use
  
  memcpy( piksi_data->GPS_time_data, gps_time_struct, sizeof(*gps_time_struct));

  // output items in structure
  fprintf(stdout, "-> # weeks = %d, ", gps_time_struct -> wn);
  fprintf(stdout, "tow = %dms, ", gps_time_struct -> tow);
  fprintf(stdout, "residual nanosecond = %dns, ", gps_time_struct -> ns_residual);
  fprintf(stdout, "flags = %d: ", gps_time_struct -> flags);
  
  fprintf(stdout, "%s\n", __FUNCTION__); // output function name
}

// utc_time is currently not stored in the struct, so this function is not currently useful.
void utc_time_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  (void)sender_id, (void)len, (void)msg, (void)context;
  
  //piksi_data_t *piksi_data= (piksi_data_t*)context;
  msg_utc_time_t *utc_time_struct;
  
  utc_time_struct = (msg_utc_time_t *) msg; // cast msg pointer to appropriate type and save for later use
  
  //memcpy( piksi_data-> ?????, utc_time_struct, sizeof(*utc_time_struct));

  // output items in structure
  fprintf(stdout, "-> flags = %d, ", utc_time_struct -> flags);
  fprintf(stdout, "tow = %d, ", utc_time_struct -> tow);
  fprintf(stdout, "date = %d/%d/%d %d:%d:%d ", utc_time_struct -> year, utc_time_struct -> month, utc_time_struct -> day, utc_time_struct -> hours, utc_time_struct -> minutes, utc_time_struct -> seconds );
  fprintf(stdout, "nanoseconds = %10dns : ", utc_time_struct -> ns);
  
  fprintf(stdout, "%s\n", __FUNCTION__); // output function name
}

void imu_raw_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  (void)sender_id, (void)len, (void)msg, (void)context;

  piksi_data_t *piksi_data= (piksi_data_t*)context;
  msg_imu_raw_t *imu_raw_struct;
  
  imu_raw_struct = (msg_imu_raw_t *) msg; // cast msg pointer to appropriate type and save for later use
  
  //fprintf(stdout, "old acc x/y/z %d/%d/%d gyr x/y/z %d/%d/%d \n", piksi_data->IMU_data->acc_x, piksi_data->IMU_data->acc_y, piksi_data->IMU_data->acc_z, piksi_data->IMU_data->gyr_x, piksi_data->IMU_data->gyr_y, piksi_data->IMU_data->gyr_z );
  //fprintf(stdout, "new acc x/y/z %d/%d/%d gyr x/y/z %d/%d/%d \n", imu_raw_struct -> acc_x, imu_raw_struct -> acc_y, imu_raw_struct -> acc_z, imu_raw_struct -> gyr_x, imu_raw_struct -> gyr_y, imu_raw_struct -> gyr_z );
  
  memcpy( piksi_data-> IMU_data, imu_raw_struct, sizeof(*imu_raw_struct));
  
  // output items in structure
  //fprintf(stdout, "tow = %d ms, %d; ", imu_raw_struct -> tow, imu_raw_struct -> tow_f);
  //fprintf(stdout, "acc x/y/z %d/%d/%d gyr x/y/z %d/%d/%d ", imu_raw_struct -> acc_x, imu_raw_struct -> acc_y, imu_raw_struct -> acc_z, imu_raw_struct -> gyr_x, imu_raw_struct -> gyr_y, imu_raw_struct -> gyr_z );
  //fprintf(stdout, "nanoseconds = %10dns : ", imu_raw_struct -> ns);
  fprintf(stdout, "%s\n", __FUNCTION__); // output function name
}




