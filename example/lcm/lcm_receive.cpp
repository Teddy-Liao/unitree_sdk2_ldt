#include <stdio.h>
#include <iostream>
#include <lcm/lcm-cpp.hpp>
#include "leg_control_data_lcmt.hpp"

class Handler 
{
    public:
        ~Handler() {}

        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const leg_control_data_lcmt* msg)
        {
            //打印关节电机信息
             std::cout<< "motor state: " << std::endl;
                    std::cout<< "motor 0" << std::endl
                            << msg->q[0] << std::endl
                            << msg->qd[0]  << std::endl
                            << msg->tau_est[0] << std::endl
                            << "-------------------" << std::endl;
        }
};

int main(int argc, char** argv)
{
    lcm::LCM lc;
    if(!lc.good())
        return 1;

    Handler handlerObject;
    lc.subscribe("EXAMPLE", &Handler::handleMessage, &handlerObject);

    while(0 == lc.handle());

    return 0;
}