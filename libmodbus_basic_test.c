#include <stdio.h>
#include <modbus.h>


int main (void)
{
  int i;
  uint16_t value[100];
  modbus_t *mb;
  uint16_t tab_reg[32];

  mb = modbus_new_tcp("127.0.0.1", 1502);
  modbus_connect(mb);

/*  for (i=0;i<10;i++)
  {

    value[i] = 5+i;
    fprintf(stdout, "writing to array => %d=%d\n", i, value[i]);
    
  }
 
  for (i=0;i<10;i++)
  {
    fprintf(stdout, "%d|", value[i]);
  }
 
  fprintf(stdout, "\n\n\n");
  */
  
  for (i=0;i<10;i++)
  {
    value[i] = 5+i;
    modbus_write_registers(mb, i, 1, value);
    fprintf(stdout, "writing to register  = %d\n", value[i]);
    
    
  }
  
  
  for (i=0;i<10;i++)
  {
    modbus_read_registers(mb, i, 1, tab_reg);
    fprintf(stdout, "Value received from register = %d\n", tab_reg[0]);
  }
  
  
  /* Read 5 registers from the address 0 */
  //modbus_read_registers(mb, 5, 5, tab_reg);
  
  
  
  
  //fprintf(stdout, "%d|%d|%d|%d|%d", tab_reg[0], tab_reg[1], tab_reg[2], tab_reg[3], tab_reg[4]);

  modbus_close(mb);
  modbus_free(mb);

  
}

