#include <stdio.h>
#ifndef _MSC_VER
#include <unistd.h>
#endif
#include <string.h>
#include <stdlib.h>
#include <errno.h>

#include <modbus.h>

#define LOOP             1
#define SERVER_ID       17
#define ADDRESS_START    0
#define ADDRESS_END     99

int main(void)
{
  modbus_t *ctx;
  int rc;
  int nb_fail=0;
  int addr = 100; // address of base register to start reading
  int nb = 5; // number of registers to read
  uint16_t *tab_rp_registers;


  // TCP
  ctx = modbus_new_tcp("127.0.0.1", 502);
  modbus_set_debug(ctx, TRUE);

  if (modbus_connect(ctx) == -1) {
      fprintf(stderr, "Connection failed: %s\n",
              modbus_strerror(errno));
      modbus_free(ctx);
      return -1;
  }

  tab_rp_registers = (uint16_t *) malloc(nb * sizeof(uint16_t));
  memset(tab_rp_registers, 0, nb * sizeof(uint16_t));

  while (1) { 
    // READ REGISTER
    //fprintf(stdout, "1\n");
    rc = modbus_read_registers(ctx, addr, nb, tab_rp_registers);
    //fprintf(stdout, "2\n");
    if (rc != nb) 
    {
      printf("ERROR modbus_read_registers single (%d)\n", rc);
      printf("Address = %d\n", addr);
      nb_fail++;
    } 
    //fprintf(stdout, "3\n");
    
    fprintf(stdout, "\nFrom modbus server: week/tow/ns_residual: %d/%d/%d\n\n", tab_rp_registers[0], MODBUS_GET_INT32_FROM_INT16(tab_rp_registers,1), MODBUS_GET_INT32_FROM_INT16(tab_rp_registers,3));
    
    // ****************************************** clean up ************************************************
    //printf("Test: ");
    if (nb_fail)
    {
      printf("%d FAILS\n", nb_fail);
    }
    else
    {
      printf("SUCCESS\n");
    }

  }
  
  // Free the memory 
  free(tab_rp_registers);
  
  // Close the connection 
  modbus_close(ctx);
  modbus_free(ctx);

  return 0;
}
