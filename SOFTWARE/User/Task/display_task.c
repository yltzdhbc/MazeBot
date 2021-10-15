#include <string.h>
#include <stdio.h>
#include "i2c.h"
#include "usart.h"
#include "cmsis_os.h"
#include "bsp_led.h"
#include "ssd1306_tests.h"
#include "ssd1306.h"
#include "display_task.h"
#include "movebase.h"
char temp;
void DisplayTask(void const *argument)
{
  I2C_Scan();
	
	LED_GRB_SHOW(BLACK);

  ssd1306_Init();
  ssd1306_Fill(Black);
  ssd1306_SetCursor(8, 0);
  ssd1306_WriteString("MazeBot", Font_16x26, White);
  ssd1306_SetCursor(30, 26);
  ssd1306_WriteString("By Ylt", Font_11x18, White);
  ssd1306_SetCursor(30, 26 + 18);
  ssd1306_WriteString("200520", Font_11x18, White);
  ssd1306_UpdateScreen();
  osDelay(1000);

  for (;;)
  {
		
//		ssd1306_TestAll();

//    ssd1306_Fill(Black);
//    ssd1306_SetCursor(2, 0);
//    ssd1306_WriteString("odom", Font_11x18, White);
//		
//		ssd1306_SetCursor(2, 26 + 18);
//    ssd1306_WriteString("200520", Font_11x18, White);
//		
//		ssd1306_SetCursor(30, 18);
//    sprintf(&temp, " %d",(char)(HAL_GetTick() / 1000));
//		
////    ssd1306_WriteString(Odom.Pos.x, Font_11x18, White);
////		
////		  Odom->Pos.theta += delta_theta;
////  Odom->Pos.y += delta_s * cos(Odom->Pos.theta);
////  Odom->Pos.x -= delta_s * sin(Odom->Pos.theta);

//		
//    ssd1306_SetCursor(30, 26 + 18);
//    ssd1306_WriteString("200520", Font_11x18, White);
//    ssd1306_UpdateScreen();

    LED_GRB_SHOW(RED);
    osDelay(500);
    LED_GRB_SHOW(GREEN);
    osDelay(500);
    LED_GRB_SHOW(BLUE);
    osDelay(500);
    LED_GRB_SHOW(YELLOW);
    osDelay(500);
    LED_GRB_SHOW(MAGENTA);
    osDelay(500);
    LED_GRB_SHOW(CYAN);
    osDelay(500);
    LED_GRB_SHOW(WHITE);
    osDelay(500);
    LED_GRB_SHOW(BLACK);
    osDelay(500);

    osDelay(20);
  }
}

void I2C_Scan()
{
  char info[] = "Scanning I2C bus...\r\n";
  HAL_UART_Transmit(&huart3, (uint8_t *)info, strlen(info), HAL_MAX_DELAY);

  HAL_StatusTypeDef res;
  for (uint16_t i = 0; i < 128; i++)
  {
    res = HAL_I2C_IsDeviceReady(&hi2c1, i << 1, 1, 10);
    if (res == HAL_OK)
    {
      char msg[64];
      snprintf(msg, sizeof(msg), "0x%02X", i);
      HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
    }
    else
    {
      HAL_UART_Transmit(&huart3, (uint8_t *)" ", 1, HAL_MAX_DELAY);
    }
  }

  HAL_UART_Transmit(&huart3, (uint8_t *)"\r\n", 2, HAL_MAX_DELAY);
}
