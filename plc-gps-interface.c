

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <libserialport.h>

#include <libsbp/sbp.h>
#include <libsbp/system.h>
#include <libsbp/observation.h>
#include <libsbp/navigation.h>
#include <libsbp/imu.h>

char *serial_port_name = NULL;
struct sp_port *piksi_port = NULL;

static sbp_msg_callbacks_node_t heartbeat_callback_node;


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

void heartbeat_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  (void)sender_id, (void)len, (void)msg, (void)context;
  fprintf(stdout, "%s\n", __FUNCTION__);
}

int main(int argc, char **argv)
{
  //fprintf(stdout, ".1\n");
  int opt;
  int result = 0;
  int intBaudRate = 115200;
  sbp_state_t s;
  
  //fprintf(stdout, ".2\n");
  if (argc <= 1) {
    usage(argv[0]);
    exit(EXIT_FAILURE);
  }
  //fprintf(stdout, ".3\n");

  while ((opt = getopt(argc, argv, "p:b:")) != -1) 
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
                usage(argv[0]);
                exit(EXIT_FAILURE);
             };
           //fprintf(stdout, "3\n");
           intBaudRate = (int) l;
        }
        //fprintf(stdout, "4\n");
        fprintf(stdout, "baud rate set to %d\n\n", intBaudRate);
        break;
      case 'h':
        usage(argv[0]);
        exit(EXIT_FAILURE);
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

  sbp_register_callback(&s, SBP_MSG_HEARTBEAT, &heartbeat_callback, NULL,
                        &heartbeat_callback_node);
  
  while(1) {
    sbp_process(&s, &piksi_port_read);
  }

  result = sp_close(piksi_port);
  if (result != SP_OK) {
    fprintf(stderr, "Cannot close %s properly!\n", serial_port_name);
  }

  sp_free_port(piksi_port);

  free(serial_port_name);

  return 0;

  
  }
