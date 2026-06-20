#include "MotorTask.hpp"

/* 实例电机 --------------------------------------------------------------------------------------------*/
BSP::Motor::Dji::GM3508<ChassisConfig::kWheelCount> Motor3508(
    ChassisConfig::kMotorBaseCanId,
    ChassisConfig::kMotorCanRxIds,
    ChassisConfig::kMotorSendCanId);

/* CAN接收 ---------------------------------------------------------------------------------------------*/
/**
 * @brief CAN初始化函数
 *
 * 初始化CAN总线并注册电机反馈数据解析回调函数。
 */
void MotorInit(void)
{
    // 实例CAN
    static auto &can1 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can1);
  
    can1.register_rx_callback([](const HAL::CAN::Frame &frame) 
    {
        Motor3508.Parse(frame);
    });
   
}


static void motor_control_logic(uint32_t tick)
{
   
    if (tick % 2 == 0)
    {
        for(int i = 0; i < ChassisConfig::kWheelCount; i++)
        {
            Motor3508.setCAN(static_cast<int16_t>(chassis_output.out_wheel[i]), i + 1);
        }
        Motor3508.sendCAN();
    }

  
}

extern "C"{
void Motor(void const * argument)
{
    MotorInit();
    static uint32_t loop_count = 0;
    for(;;)
    {
        loop_count++;
        motor_control_logic(loop_count);
        osDelay(1);
    } 
}

}
