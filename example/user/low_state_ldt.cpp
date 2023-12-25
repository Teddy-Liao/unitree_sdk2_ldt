#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>

#define TOPIC_LOWSTATE "rt/lowstate"


void LowStateHandler(const void* message){

    unitree_go::msg::dds_::LowState_ state = *(unitree_go::msg::dds_::LowState_*)message;

    //打印FR外摆关节电机信息
    std::cout<< "FR ab-ad motor state: " << std::endl;
    std::cout<< (unsigned int)state.motor_state()[0].mode() <<", "
             << state.motor_state()[0].q()  <<", "
             << state.motor_state()[0].dq()  <<", "
             << state.motor_state()[0].ddq() <<", "
             << state.motor_state()[0].tau_est() << ","
             << (unsigned int)state.motor_state()[0].temperature() << std::endl;
    //打印遥控器信息
    // std::cout<< "wireless joystick: " << std::endl;
    // std::cout << (unsigned int)state.wireless_remote()[0];
    // std::cout << std::endl;
    //打印IMU信息
    std::cout << "IMU state: " << std::endl;
    std::cout << "roll angle: " << state.imu_state().rpy()[0] << std::endl;
    std::cout << "pitch angle: " << state.imu_state().rpy()[1] << std::endl;
    std::cout << "yaw angle: " << state.imu_state().rpy()[2] << std::endl;
    std::cout << "accelerometer x: " << state.imu_state().accelerometer()[0] << std::endl;
    std::cout << "accelerometer y: " << state.imu_state().accelerometer()[1] << std::endl;
    std::cout << "accelerometer z: " << state.imu_state().accelerometer()[2] << std::endl;
    std::cout << "gyroscope x: " << state.imu_state().gyroscope()[0] << std::endl;
    std::cout << "gyroscope y: " << state.imu_state().gyroscope()[1] << std::endl;
    std::cout << "gyroscope z: " << state.imu_state().gyroscope()[2] << std::endl;
    //打印当前电池电量
    std::cout << "Battery: " << (float)state.bms_state().soc() << "%" << std::endl;
    //打印足端力
    std::cout<<"foot force: "
            <<state.foot_force()[0]<<", "
            <<state.foot_force()[1]<<", "
            <<state.foot_force()[2]<<", "
            <<state.foot_force()[3]<<std::endl;
    std::cout << "---------------------------" << std::endl;
}


int main(int argc, char **argv)
{
    if (argc < 2)
    {
    std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
    exit(-1);
    }
    unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);

    // 创建一个subscriber
    unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LowState_> suber(TOPIC_LOWSTATE);
    //初始化Channel
    suber.InitChannel(LowStateHandler);

    while(1)
    {
        usleep(20000);
    }

    return 0;
}
