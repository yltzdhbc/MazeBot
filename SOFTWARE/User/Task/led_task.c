#include <string.h>
#include <stdio.h>

#include "i2c.h"
#include "usart.h"
#include "cmsis_os.h"
 
#include "bsp_led.h"
#include "led_task.h"


#include "ssd1306_tests.h"


void LedTask(void const * argument)
{

  for(;;)
  {

    osDelay(2000);
  }

}
