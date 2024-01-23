#include <lcm/lcm-cpp.hpp>
#include "leg_control_data_lcmt.hpp"
// 
#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>


#define TOPIC_LOWSTATE "rt/lowstate"


class Custom
{
public:
    explicit Custom()
    {}

    ~Custom()
    {}

    void Init();
    void LowStateMessageHandler(const void* messages);
    void lcm_send();
    leg_control_data_lcmt leg_control_lcm_data;
    unitree_go::msg::dds_::LowState_ low_state{};
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber;
    lcm::LCM lc;
};

void Custom::LowStateMessageHandler(const void* message)
{
    low_state = *(unitree_go::msg::dds_::LowState_*)message;
}


void Custom::Init(){
    /*create subscriber*/
    lowstate_subscriber.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    lowstate_subscriber->InitChannel(std::bind(&Custom::LowStateMessageHandler, this, std::placeholders::_1), 1);
}

void Custom::lcm_send(){
    for (int i = 0; i < 12; i++)
    {
        leg_control_lcm_data.q[i] = low_state.motor_state()[i].q();
        leg_control_lcm_data.qd[i] = low_state.motor_state()[i].dq();
        leg_control_lcm_data.tau_est[i] = low_state.motor_state()[i].tau_est();
    }
    lc.publish("EXAMPLE", &leg_control_lcm_data);
}


int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
        exit(-1);
    }
    unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);

    Custom custom;
    custom.Init();
    custom.lcm_send();


    while(1)
    {
        usleep(20000);
    }

    return 0;
}
