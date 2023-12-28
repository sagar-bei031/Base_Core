#include "encoder.hpp"
#include "stm32f4xx.h"
#include "stdio.h"

int32_t Encoder::get_count(void)
{
  int32_t count = henc->Instance->CNT;
  if (count > (int32_t)32768)
    count = count - (int32_t)65536;
  return count;
}

int64_t Encoder::get_count_aggregate(void)
{
  return count_aggregate + get_count();
}

float Encoder::get_omega(void)
{
  int32_t count = get_count() - prevCount;
  int32_t sample_time = HAL_GetTick() - last_reset_time;
  omega = (2.0f * 3.141592f) * (float)count / ((float)ppr * (float)sample_time) * 1000.0f;
  prevCount = get_count();
  return omega;
}

void Encoder::init(void)
{
  HAL_TIM_Encoder_Start(henc, TIM_CHANNEL_ALL);
  henc->Instance->CNT = 0;
}

void Encoder::reset_encoder_count(void)
{
  prevCount = 0;
  count_aggregate += get_count();
  henc->Instance->CNT = 0;
  last_reset_time = HAL_GetTick();
}
