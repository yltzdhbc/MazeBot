/**
  ****************************(C) COPYRIGHT 2020 HCRT****************************
  * @file       bsp_led.c/h
  * @brief      led rgb 灯混控函数
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2020.4.11    		 YLT             V1.0 done
  @verbatim
  ==============================================================================
	* 2020.4.11(update):完成初版本
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2020 HCRT****************************
  */

#include "bsp_led.h"

ledRgb_e aRGB;

void LED_GRB_SHOW(ledRgb_e aRGB)
{
  switch (aRGB)
  {
  case RED:
    HAL_GPIO_WritePin(LED_RGB_R_GPIO_Port, LED_RGB_R_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_RGB_G_GPIO_Port, LED_RGB_G_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_RGB_B_GPIO_Port, LED_RGB_B_Pin, GPIO_PIN_SET);
    break;
  case GREEN:
    HAL_GPIO_WritePin(LED_RGB_R_GPIO_Port, LED_RGB_R_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_RGB_G_GPIO_Port, LED_RGB_G_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_RGB_B_GPIO_Port, LED_RGB_B_Pin, GPIO_PIN_SET);
    break;
  case BLUE:
    HAL_GPIO_WritePin(LED_RGB_R_GPIO_Port, LED_RGB_R_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_RGB_G_GPIO_Port, LED_RGB_G_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_RGB_B_GPIO_Port, LED_RGB_B_Pin, GPIO_PIN_RESET);
    break;
  case YELLOW:
    HAL_GPIO_WritePin(LED_RGB_R_GPIO_Port, LED_RGB_R_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_RGB_G_GPIO_Port, LED_RGB_G_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_RGB_B_GPIO_Port, LED_RGB_B_Pin, GPIO_PIN_SET);
    break;
  case MAGENTA:
    HAL_GPIO_WritePin(LED_RGB_R_GPIO_Port, LED_RGB_R_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_RGB_G_GPIO_Port, LED_RGB_G_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_RGB_B_GPIO_Port, LED_RGB_B_Pin, GPIO_PIN_RESET);
    break;
  case CYAN:
    HAL_GPIO_WritePin(LED_RGB_R_GPIO_Port, LED_RGB_R_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_RGB_G_GPIO_Port, LED_RGB_G_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_RGB_B_GPIO_Port, LED_RGB_B_Pin, GPIO_PIN_RESET);
    break;
  case WHITE:
    HAL_GPIO_WritePin(LED_RGB_R_GPIO_Port, LED_RGB_R_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_RGB_G_GPIO_Port, LED_RGB_G_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_RGB_B_GPIO_Port, LED_RGB_B_Pin, GPIO_PIN_RESET);
    break;
  case BLACK:
    HAL_GPIO_WritePin(LED_RGB_R_GPIO_Port, LED_RGB_R_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_RGB_G_GPIO_Port, LED_RGB_G_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_RGB_B_GPIO_Port, LED_RGB_B_Pin, GPIO_PIN_SET);
    break;
  default:
    break;
  }
}
