#include "sbp_callback_functions.h"

int piksi_data_setup(piksi_data_t *piksidata)
{

  // CurrentData is piksidata. one is used globally, the other passed by reference. should probably consolidate this!
  piksi_data_t *CurrentData     = (piksi_data_t *)          mmap(NULL, sizeof(*CurrentData),                   PROT_READ|PROT_WRITE, MAP_ANONYMOUS|MAP_SHARED, -1, 0); 
  piksidata->GPS_time_data      = (msg_gps_time_t *)        mmap(NULL, sizeof(piksidata->GPS_time_data),       PROT_READ|PROT_WRITE, MAP_ANONYMOUS|MAP_SHARED, -1, 0);
  piksidata->baseline_NED_data  = (msg_baseline_ned_t *)    mmap(NULL, sizeof(piksidata->baseline_NED_data),   PROT_READ|PROT_WRITE, MAP_ANONYMOUS|MAP_SHARED, -1, 0);
  piksidata->LLH_data           = (msg_pos_llh_t *)         mmap(NULL, sizeof(piksidata->LLH_data),            PROT_READ|PROT_WRITE, MAP_ANONYMOUS|MAP_SHARED, -1, 0);
  piksidata->NED_velocity_data  = (msg_vel_ned_t *)         mmap(NULL, sizeof(piksidata->NED_velocity_data),   PROT_READ|PROT_WRITE, MAP_ANONYMOUS|MAP_SHARED, -1, 0);
  piksidata->IMU_data           = (msg_imu_raw_t *)         mmap(NULL, sizeof(piksidata->IMU_data),            PROT_READ|PROT_WRITE, MAP_ANONYMOUS|MAP_SHARED, -1, 0);
  piksidata->ECEF_data          = (msg_pos_ecef_t *)        mmap(NULL, sizeof(piksidata->ECEF_data),           PROT_READ|PROT_WRITE, MAP_ANONYMOUS|MAP_SHARED, -1, 0);
  piksidata->baseline_ECEF_data = (msg_baseline_ecef_t *)   mmap(NULL, sizeof(piksidata->baseline_ECEF_data),  PROT_READ|PROT_WRITE, MAP_ANONYMOUS|MAP_SHARED, -1, 0);
  piksidata->UTC_data           = (msg_utc_time_t *)        mmap(NULL, sizeof(piksidata->UTC_data),            PROT_READ|PROT_WRITE, MAP_ANONYMOUS|MAP_SHARED, -1, 0);
  piksidata->log_data           = (msg_log_t *)             mmap(NULL, sizeof(piksidata->log_data),            PROT_READ|PROT_WRITE, MAP_ANONYMOUS|MAP_SHARED, -1, 0);
  piksidata->device_monitor_data= (msg_device_monitor_t *)  mmap(NULL, sizeof(piksidata->device_monitor_data), PROT_READ|PROT_WRITE, MAP_ANONYMOUS|MAP_SHARED, -1, 0);
  piksidata->linux_sys_data     = (msg_linux_sys_state_t *) mmap(NULL, sizeof(piksidata->linux_sys_data),      PROT_READ|PROT_WRITE, MAP_ANONYMOUS|MAP_SHARED, -1, 0);
  piksidata->correction_age_data= (msg_age_corrections_t *) mmap(NULL, sizeof(piksidata->correction_age_data), PROT_READ|PROT_WRITE, MAP_ANONYMOUS|MAP_SHARED, -1, 0);
  

  slog(0, SLOG_INFO, "piksi_data_setup complete. totalsize = %d bytes", 
      (int)(
	  sizeof (*piksidata) + sizeof(piksidata-> GPS_time_data) + 
	  sizeof(piksidata-> baseline_NED_data) + sizeof(piksidata-> LLH_data) + 
	  sizeof(piksidata-> NED_velocity_data) + sizeof(piksidata-> IMU_data) +
	  sizeof(piksidata-> ECEF_data) + sizeof(piksidata-> baseline_ECEF_data) + 
	  sizeof(piksidata-> UTC_data) + sizeof(piksidata-> log_data)  + 
	  sizeof(piksidata-> device_monitor_data) + sizeof(piksidata-> linux_sys_data) +
	  sizeof(piksidata-> correction_age_data)
	  ) );

  
  return 0; // need to add error code here
}

int piksi_data_close(piksi_data_t *piksidata)
{
  
  munmap(piksidata->GPS_time_data, sizeof(piksidata->GPS_time_data));
  munmap(piksidata->baseline_NED_data, sizeof(piksidata->baseline_NED_data));
  munmap(piksidata->LLH_data, sizeof(piksidata->LLH_data));
  munmap(piksidata->NED_velocity_data, sizeof(piksidata->NED_velocity_data));
  munmap(piksidata->IMU_data, sizeof(piksidata->IMU_data));
  munmap(piksidata->ECEF_data, sizeof(piksidata->ECEF_data));
  munmap(piksidata->baseline_ECEF_data, sizeof(piksidata->baseline_ECEF_data));
  munmap(piksidata->UTC_data, sizeof(piksidata->UTC_data));
  munmap(piksidata->log_data, sizeof(piksidata->log_data));
  munmap(piksidata->device_monitor_data, sizeof(piksidata->device_monitor_data));
  munmap(piksidata->linux_sys_data, sizeof(piksidata->linux_sys_data));
  munmap(piksidata->correction_age_data, sizeof(piksidata->correction_age_data));

  munmap(piksidata, sizeof(piksidata));


  return 0; // need to add error code here
}

// currently no data in this fucntion. just used for diagnostics atm.
void heartbeat_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  // time_t tmLastHeartbeat;
  (void)sender_id, (void)len, (void)msg, (void)context;

  tmLastHeartbeat = time(NULL);
  tmLastSuccessfulRead = time(NULL);
  
  if (blnDebugToScreen == 13) {fprintf(stdout, "%s\n", __FUNCTION__);}
}

void pos_llh_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  (void)sender_id, (void)len, (void)msg, (void)context;

  piksi_data_t *piksi_data= (piksi_data_t*)context;
  msg_pos_llh_t *pos_llh_struct;

  pos_llh_struct = (msg_pos_llh_t *) msg; // cast msg pointer to appropriate type and save for later use
  
  memcpy( piksi_data-> LLH_data, pos_llh_struct, sizeof(*pos_llh_struct));

  // cJSON *root;
  // cJSON *dta;
  // root = cJSON_CreateObject();
  // cJSON_AddItemToObject(root, "name", cJSON_CreateString("pos_llh"));
  // cJSON_AddItemToObject(root, "data", dta = cJSON_CreateObject());
  // cJSON_AddNumberToObject(dta, "tow", pos_llh_struct -> tow);
  // cJSON_AddNumberToObject(dta, "lat", pos_llh_struct -> lat);
  // cJSON_AddNumberToObject(dta, "lon", pos_llh_struct -> lon);
  // cJSON_AddNumberToObject(dta, "height", pos_llh_struct -> height);
  // cJSON_AddNumberToObject(dta, "h_accuracy", pos_llh_struct -> h_accuracy);
  // cJSON_AddNumberToObject(dta, "v_accuracy", pos_llh_struct -> v_accuracy);
  // cJSON_AddNumberToObject(dta, "n_sats", pos_llh_struct -> n_sats);
  // cJSON_AddNumberToObject(dta, "flags", pos_llh_struct -> flags );
  if (blnDebugToScreen == 13) {
    // fprintf(stdout, "JSON string is %s\n", cJSON_PrintUnformatted(root));
  }
//  slog(2, SLOG_LIVE, cJSON_PrintUnformatted(root));

  // cJSON_Delete(root);

  tmLastSuccessfulRead = time(NULL);

  
  
  // output items in structure
  if (blnDebugToScreen == 13) {
    fprintf(stdout, "tow = %d ms; ", pos_llh_struct -> tow);
  }
  if (blnDebugToScreen == 13) {
    fprintf(stdout, "-> latitude/longitude/height = %lf/%lf/%lf ", pos_llh_struct -> lat,  pos_llh_struct -> lon,  pos_llh_struct -> height);
  }

  //if (blnDebugToScreen == 13) {fprintf(stdout, "%s\n", __FUNCTION__);}
}

void baseline_ned_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  (void)sender_id, (void)len, (void)msg, (void)context;

  piksi_data_t *piksi_data= (piksi_data_t*)context;
  msg_baseline_ned_t *baseline_ned_struct;
  
  baseline_ned_struct = (msg_baseline_ned_t *) msg; // cast msg pointer to appropriate type and save for later use
  
  memcpy( piksi_data-> baseline_NED_data, baseline_ned_struct, sizeof(*baseline_ned_struct));

  // cJSON *root;
  // cJSON *dta;
  // root = cJSON_CreateObject();
  // cJSON_AddItemToObject(root, "name", cJSON_CreateString("baseline_ned"));
  // cJSON_AddItemToObject(root, "data", dta = cJSON_CreateObject());
  // cJSON_AddNumberToObject(dta, "tow", baseline_ned_struct -> tow);
  // cJSON_AddNumberToObject(dta, "n", baseline_ned_struct -> n);
  // cJSON_AddNumberToObject(dta, "e", baseline_ned_struct -> e);
  // cJSON_AddNumberToObject(dta, "d", baseline_ned_struct -> d);
  // cJSON_AddNumberToObject(dta, "h_accuracy", baseline_ned_struct -> h_accuracy);
  // cJSON_AddNumberToObject(dta, "v_accuracy", baseline_ned_struct -> v_accuracy);
  // cJSON_AddNumberToObject(dta, "n_sats", baseline_ned_struct -> n_sats);
  // cJSON_AddNumberToObject(dta, "flags", baseline_ned_struct -> flags );
  if (blnDebugToScreen == 13) {
    // fprintf(stdout, "JSON string is %s\n", cJSON_PrintUnformatted(root));
  }
//  slog(2, SLOG_LIVE, cJSON_PrintUnformatted(root));
  
  // cJSON_Delete(root);

  tmLastSuccessfulRead = time(NULL);
  
  // output items in structure
  if (blnDebugToScreen == 13) {
    fprintf(stdout, "tow = %d ms; ", baseline_ned_struct -> tow);
	fprintf(stdout, "-> north/east/down = %d/%d/%d ", baseline_ned_struct -> n,  baseline_ned_struct -> e,  baseline_ned_struct -> d);
	fprintf(stdout, "-> h_acc/v_acc/#sats = %d/%d/%d ", baseline_ned_struct -> h_accuracy,  baseline_ned_struct -> v_accuracy,  baseline_ned_struct -> n_sats);
	if (baseline_ned_struct -> flags | 0x04){
      fprintf(stdout, "-> src=fixed RTK ");
	}
  }
  if (blnDebugToScreen == 13) {fprintf(stdout, "%s\n", __FUNCTION__);}
}

void vel_ned_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  (void)sender_id, (void)len, (void)msg, (void)context;

  piksi_data_t *piksi_data= (piksi_data_t*)context;
  msg_vel_ned_t *vel_ned_struct;

  vel_ned_struct = (msg_vel_ned_t *) msg; // cast msg pointer to appropriate type and save for later use
  
  memcpy( piksi_data-> NED_velocity_data, vel_ned_struct, sizeof(*vel_ned_struct));

  // cJSON *root;
  // cJSON *dta;
  // root = cJSON_CreateObject();
  // cJSON_AddItemToObject(root, "name", cJSON_CreateString("vel_ned"));
  // cJSON_AddItemToObject(root, "data", dta = cJSON_CreateObject());
  // cJSON_AddNumberToObject(dta, "tow", vel_ned_struct -> tow);
  // cJSON_AddNumberToObject(dta, "n", vel_ned_struct -> n);
  // cJSON_AddNumberToObject(dta, "e", vel_ned_struct -> e);
  // cJSON_AddNumberToObject(dta, "d", vel_ned_struct -> d);
  // cJSON_AddNumberToObject(dta, "h_accuracy", vel_ned_struct -> h_accuracy);
  // cJSON_AddNumberToObject(dta, "v_accuracy", vel_ned_struct -> v_accuracy);
  // cJSON_AddNumberToObject(dta, "n_sats", vel_ned_struct -> n_sats);
  // cJSON_AddNumberToObject(dta, "flags", vel_ned_struct -> flags );
  if (blnDebugToScreen == 13) {
    // fprintf(stdout, "JSON string is %s\n", cJSON_PrintUnformatted(root));
  }
//  slog(2, SLOG_LIVE, cJSON_PrintUnformatted(root));

  // cJSON_Delete(root);

  tmLastSuccessfulRead = time(NULL);


  // output items in structure
  if (blnDebugToScreen == 13) {
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
  if (blnDebugToScreen == 13) {fprintf(stdout, "%s\n", __FUNCTION__);}
}

// not currently any useful data stored in this packet.
void base_pos_llh_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  (void)sender_id, (void)len, (void)msg, (void)context;
  
  //piksi_data_t *piksi_data= (piksi_data_t*)context;
  msg_base_pos_llh_t *base_pos_llh_struct;

  base_pos_llh_struct = (msg_base_pos_llh_t *) msg; // cast msg pointer to appropriate type and save for later use
  
  //memcpy( piksi_data-> base_pos_llh_struct, sizeof(*base_pos_llh_struct));

  // output items in structure
  // cJSON *root;
  // cJSON *dta;
  // root = cJSON_CreateObject();
  // cJSON_AddItemToObject(root, "name", cJSON_CreateString("base_pos_llh"));
  // cJSON_AddItemToObject(root, "data", dta = cJSON_CreateObject());
  // cJSON_AddNumberToObject(dta, "lat", base_pos_llh_struct -> lat);
  // cJSON_AddNumberToObject(dta, "lon", base_pos_llh_struct -> lon);
  // cJSON_AddNumberToObject(dta, "height", base_pos_llh_struct -> height);
//  slog(2, SLOG_LIVE, cJSON_PrintUnformatted(root));

  // cJSON_Delete(root);

  tmLastSuccessfulRead = time(NULL);
  
  if (blnDebugToScreen == 13) 
   {
	// fprintf(stdout, "JSON string is %s\n", cJSON_PrintUnformatted(root));
	fprintf(stdout, "-> latitude/longitude/height = %lf/%lf/%lf ", base_pos_llh_struct -> lat,  base_pos_llh_struct -> lon,  base_pos_llh_struct -> height);
	fprintf(stdout, "%s\n", __FUNCTION__);
  }
}

void gps_time_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  (void)sender_id, (void)len, (void)msg, (void)context;
  
  piksi_data_t *piksi_data= (piksi_data_t*)context;
  msg_gps_time_t *gps_time_struct;

  gps_time_struct = (msg_gps_time_t *) msg; // cast msg pointer to appropriate type and save for later use
  
  memcpy( piksi_data->GPS_time_data, gps_time_struct, sizeof(*gps_time_struct));

  tmLastSuccessfulRead = time(NULL);

  //fprintf(stdout, "GPS time callback tow:\"%d\"\n", piksi_data->GPS_time_data->tow);

  
}

// utc_time is currently not stored in the struct, so this function is not currently useful.
void utc_time_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  (void)sender_id, (void)len, (void)msg, (void)context;
  
  piksi_data_t *piksi_data= (piksi_data_t*)context;
  msg_utc_time_t *utc_time_struct;
  
  utc_time_struct = (msg_utc_time_t *) msg; // cast msg pointer to appropriate type and save for later use
  
  memcpy( piksi_data->UTC_data, utc_time_struct, sizeof(*utc_time_struct));

  // output items in structure
  // cJSON *root;
  // cJSON *dta;
  // root = cJSON_CreateObject();
  // cJSON_AddItemToObject(root, "name", cJSON_CreateString("utc_time"));
  // cJSON_AddItemToObject(root, "data", dta = cJSON_CreateObject());
  // cJSON_AddNumberToObject(dta, "flags", utc_time_struct -> flags);
  // cJSON_AddNumberToObject(dta, "tow", utc_time_struct -> tow);
  // cJSON_AddNumberToObject(dta, "year", utc_time_struct -> year);
  // cJSON_AddNumberToObject(dta, "month", utc_time_struct -> month);
  // cJSON_AddNumberToObject(dta, "day", utc_time_struct -> day);
  // cJSON_AddNumberToObject(dta, "hours", utc_time_struct -> hours);
  // cJSON_AddNumberToObject(dta, "minutes", utc_time_struct -> minutes);
  // cJSON_AddNumberToObject(dta, "seconds", utc_time_struct -> seconds);
  // cJSON_AddNumberToObject(dta, "ns", utc_time_struct -> ns);
  if (blnDebugToScreen == 13) {
    // fprintf(stdout, "JSON string is %s\n", cJSON_PrintUnformatted(root));
}
//  slog(2, SLOG_LIVE, cJSON_PrintUnformatted(root));

  // cJSON_Delete(root);
  
  tmLastSuccessfulRead = time(NULL);
  
  if (blnDebugToScreen == 13) {
	  fprintf(stdout, "-> flags = %d, ", utc_time_struct -> flags);
	  fprintf(stdout, "tow = %d, ", utc_time_struct -> tow);
	  fprintf(stdout, "date = %d/%d/%d %d:%d:%d ", utc_time_struct -> year, utc_time_struct -> month, utc_time_struct -> day, utc_time_struct -> hours, utc_time_struct -> minutes, utc_time_struct -> seconds );
	  fprintf(stdout, "nanoseconds = %10dns : ", utc_time_struct -> ns);
  }
  if (blnDebugToScreen == 13) {fprintf(stdout, "%s\n", __FUNCTION__);} // output function name
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


  // cJSON *root;
  // cJSON *dta;
  // root = cJSON_CreateObject();
  // cJSON_AddItemToObject(root, "name", cJSON_CreateString("IMU_data"));
  // cJSON_AddItemToObject(root, "data", dta = cJSON_CreateObject());
  // cJSON_AddNumberToObject(dta, "tow", imu_raw_struct -> tow);
  // cJSON_AddNumberToObject(dta, "tow_f", imu_raw_struct -> tow_f);
  // cJSON_AddNumberToObject(dta, "acc_x", imu_raw_struct -> acc_x);
  // cJSON_AddNumberToObject(dta, "acc_y", imu_raw_struct -> acc_y);
  // cJSON_AddNumberToObject(dta, "acc_z", imu_raw_struct -> acc_z);
  // cJSON_AddNumberToObject(dta, "gyr_x", imu_raw_struct -> gyr_x);
  // cJSON_AddNumberToObject(dta, "gyr_y", imu_raw_struct -> gyr_y);
  // cJSON_AddNumberToObject(dta, "gyr_z", imu_raw_struct -> gyr_z);
  if (blnDebugToScreen == 13) {
    //fprintf(stdout, "JSON string is %s\n", cJSON_PrintUnformatted(root));
  }
//  slog(2, SLOG_LIVE, cJSON_PrintUnformatted(root));

  // cJSON_Delete(root);

  tmLastSuccessfulRead = time(NULL);
  
  // output items in structure
  //fprintf(stdout, "tow = %d ms, %d; ", imu_raw_struct -> tow, imu_raw_struct -> tow_f);
  if (blnDebugToScreen == 13) {fprintf(stdout, "acc x/y/z %d/%d/%d gyr x/y/z %d/%d/%d ", imu_raw_struct -> acc_x, imu_raw_struct -> acc_y, imu_raw_struct -> acc_z, imu_raw_struct -> gyr_x, imu_raw_struct -> gyr_y, imu_raw_struct -> gyr_z );}
  //fprintf(stdout, "nanoseconds = %10dns : ", imu_raw_struct -> ns);
  if (blnDebugToScreen == 13) {fprintf(stdout, "%s\n", __FUNCTION__);} // output function name
}


void baseline_ecef_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  (void)sender_id, (void)len, (void)msg, (void)context;
  piksi_data_t *piksi_data= (piksi_data_t*)context;
  
  msg_baseline_ecef_t *baseline_ECEF_struct;
  baseline_ECEF_struct = (msg_baseline_ecef_t *) msg;
  memcpy( piksi_data-> baseline_ECEF_data, baseline_ECEF_struct, sizeof(*baseline_ECEF_struct));
  
  //fprintf(stdout, "baseline_ecef_callback tow:\"%d\"\n", piksi_data->baseline_ECEF_data->tow);

  // cJSON *root;
  // cJSON *dta;
  // root = cJSON_CreateObject();
  // cJSON_AddItemToObject(root, "name", cJSON_CreateString("baseline_ECEF"));
  // cJSON_AddItemToObject(root, "data", dta = cJSON_CreateObject());
  // cJSON_AddNumberToObject(dta, "tow", baseline_ECEF_struct -> tow);
  // cJSON_AddNumberToObject(dta, "x", baseline_ECEF_struct -> x);
  // cJSON_AddNumberToObject(dta, "y", baseline_ECEF_struct -> y);
  // cJSON_AddNumberToObject(dta, "z", baseline_ECEF_struct -> z);
  // cJSON_AddNumberToObject(dta, "accuracy", baseline_ECEF_struct -> accuracy);
  // cJSON_AddNumberToObject(dta, "n_sats", baseline_ECEF_struct -> n_sats);
  // cJSON_AddNumberToObject(dta, "flags", baseline_ECEF_struct -> flags );
  if (blnDebugToScreen == 13) {
    // fprintf(stdout, "JSON string is %s\n", cJSON_PrintUnformatted(root));
  }
//  slog(2, SLOG_LIVE, cJSON_PrintUnformatted(root));

  // cJSON_Delete(root);
  
  tmLastSuccessfulRead = time(NULL);
  
  // output items in structure
  if (blnDebugToScreen == 13) {
	  fprintf(stdout, "tow = %d ms; ", baseline_ECEF_struct -> tow);
	fprintf(stdout, "-> x/y/z = %d/%d/%d ", baseline_ECEF_struct -> x,  baseline_ECEF_struct -> y,  baseline_ECEF_struct -> z);
	fprintf(stdout, "-> acc/#sats = %d/%d ", baseline_ECEF_struct -> accuracy, baseline_ECEF_struct -> n_sats);
	if (baseline_ECEF_struct -> flags | 0x04){
		fprintf(stdout, "-> src=fixed RTK ");
	}
  }
  if (blnDebugToScreen == 13) {fprintf(stdout, "%s\n", __FUNCTION__);}

  
 
}


void pos_ecef_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  
  (void)sender_id, (void)len, (void)msg, (void)context;
  piksi_data_t *piksi_data= (piksi_data_t*)context;
  
  msg_pos_ecef_t *ECEF_struct;

  ECEF_struct = (msg_pos_ecef_t *) msg;
  
  memcpy( piksi_data-> ECEF_data, ECEF_struct, sizeof(*ECEF_struct));
 
  //fprintf(stdout, "pos_ecef_callback tow:\"%d\"\n", piksi_data->ECEF_data->tow);

  // cJSON *root;
  // cJSON *dta;
  // root = cJSON_CreateObject();
  // cJSON_AddItemToObject(root, "name", cJSON_CreateString("pos_ecef"));
  // cJSON_AddItemToObject(root, "data", dta = cJSON_CreateObject());
  // cJSON_AddNumberToObject(dta, "tow", ECEF_struct -> tow);
  // cJSON_AddNumberToObject(dta, "x", ECEF_struct -> x);
  // cJSON_AddNumberToObject(dta, "y", ECEF_struct -> y);
  // cJSON_AddNumberToObject(dta, "z", ECEF_struct -> z);
  // cJSON_AddNumberToObject(dta, "accuracy", ECEF_struct -> accuracy);
  // cJSON_AddNumberToObject(dta, "n_sats", ECEF_struct -> n_sats);
  // cJSON_AddNumberToObject(dta, "flags", ECEF_struct -> flags );
//  slog(2, SLOG_LIVE, cJSON_PrintUnformatted(root));

  // cJSON_Delete(root);

  tmLastSuccessfulRead = time(NULL);

  // output items in structure
  if (blnDebugToScreen == 13) {
    // fprintf(stdout, "JSON string is %s\n", cJSON_PrintUnformatted(root));
  }
  if (blnDebugToScreen == 13) {
    fprintf(stdout, "tow = %d ms; ", ECEF_struct -> tow);
  }
  if (blnDebugToScreen == 13) {
    fprintf(stdout, "-> x/y/z = %lf/%lf/%lf ", ECEF_struct -> x,  ECEF_struct -> y,  ECEF_struct -> z);
  }

  //if (blnDebugToScreen == 13) {fprintf(stdout, "%s\n", __FUNCTION__);}
  
  

}

void log_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  (void)sender_id, (void)len, (void)msg, (void)context;
  piksi_data_t *piksi_data= (piksi_data_t*)context;
  //char *strQuality = "Quality of time solution changed";
  
  msg_log_t *log_struct;

  log_struct = (msg_log_t *) msg;
  
  memcpy( piksi_data-> log_data, log_struct, sizeof(*log_struct));

  
  switch (log_struct -> level)
  {
	  case 0:
		slog(2, SLOG_LIVE, "EMERGENCY: %s", log_struct -> text);
	  break;
	  case 1:
		slog(2, SLOG_LIVE, "ALERT: %s", log_struct -> text);
	  break;
	  case 2:
		slog(2, SLOG_LIVE, "CRITICAL: %s", log_struct -> text);
	  break;
	  case 3:
		slog(2, SLOG_LIVE, "ERROR: %s", log_struct -> text);
	  break;
	  case 4:
		slog(2, SLOG_LIVE, "WARN: %s", log_struct -> text);
	  break;
	  case 5:
		slog(2, SLOG_LIVE, "NOTICE: %s", log_struct -> text);
	  break;
	  // don't log info or debug messages. too much repetitive crap!
	  // case 6:
		// if(strstr(log_struct -> text, strQuality) == NULL) {
			// slog(2, SLOG_LIVE, "INFO: %s", log_struct -> text);
		// }
	  // break;
	  // case 7:
		// slog(2, SLOG_LIVE, "DEBUG: %s", log_struct -> text);
	  // break;
  }
	
}


void device_monitor_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  (void)sender_id, (void)len, (void)msg, (void)context;
  piksi_data_t *piksi_data= (piksi_data_t*)context;
  
  msg_device_monitor_t *device_monitor_struct;
  device_monitor_struct = (msg_device_monitor_t *) msg;
  memcpy( piksi_data-> device_monitor_data, device_monitor_struct, sizeof(*device_monitor_struct));
  
}


void linux_sys_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  (void)sender_id, (void)len, (void)msg, (void)context;
  piksi_data_t *piksi_data= (piksi_data_t*)context;
  
  msg_linux_sys_state_t *linux_sys_data_struct;
  linux_sys_data_struct = (msg_linux_sys_state_t *) msg;
  memcpy( piksi_data-> linux_sys_data, linux_sys_data_struct, sizeof(*linux_sys_data_struct));
  
  slog(2, SLOG_LIVE, "total memory=%d", linux_sys_data_struct->mem_total);
}

void correction_age_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  (void)sender_id, (void)len, (void)msg, (void)context;
  piksi_data_t *piksi_data= (piksi_data_t*)context;
  
  msg_age_corrections_t *correction_age_data_struct;
  correction_age_data_struct = (msg_age_corrections_t *) msg;
  memcpy( piksi_data-> correction_age_data, correction_age_data_struct, sizeof(*correction_age_data_struct));

}
