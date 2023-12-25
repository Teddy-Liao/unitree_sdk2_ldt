/*
    此程序基于高层运动控制，运行此程序前，Go2自带的sport_client服务需要开启
    运行程序前保持在阻尼趴地状态，运行后效果是：
    恢复站立 - 坐下 - 恢复站立
*/
#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unistd.h>

int main(int argc, char *argv[])
{
    if(argc < 2){
        // argv[0]的默认值是一个指向程序本身的可执行文件的名称的字符串
        // 额外的传入参数则是argv[1]
        std::cout << "Network Interface is: " << argv[0] << std::endl;
        // 由于没有传入网卡名作为参数，exit(-1)用于程序的异常退出
        exit(-1);
    }

    //argv[1]由终端传入，为机器人连接的网卡名称
    unitree::robot::ChannelFactory::Instance()-> Init(0,argv[1]);

    unitree::robot::go2::SportClient sport_client;
    sport_client.SetTimeout(10.0f);
    sport_client.Init();
    // 状态名称和对应值的map键值对
    std::map<std::string, std::string> state_map;
    //
    std::vector<std::string> state_name = {"state",
                                        "bodyHeight",
                                        "footRaiseHeight",
                                        "speedLevel",
                                        "gait",
                                        "joystick",
                                        "dance",
                                        "continuousGait",
                                        "economicGait"};
    

    sport_client.RecoveryStand();
    sleep(3);
    // 获取高层状态
    sport_client.GetState(state_name, state_map);
    std::cout << state_map["state"] << std::endl;
    std::cout << state_map["bodyHeight"] << std::endl;
    std::cout << state_map["footRaiseHeight"] << std::endl;
    std::cout << state_map["speedLevel"] << std::endl;
    std::cout << state_map["gait"] << std::endl;
    std::cout << state_map["joystick"] << std::endl;
    std::cout << state_map["dance"] << std::endl;
    std::cout << state_map["continuousGait"] << std::endl;
    std::cout << state_map["economicGait"] << std::endl;
    

    sport_client.Sit();
    sleep(3);
    // 获取高层状态
    sport_client.GetState(state_name, state_map);
    std::cout << state_map["state"] << std::endl;
    std::cout << state_map["bodyHeight"] << std::endl;
    std::cout << state_map["footRaiseHeight"] << std::endl;
    std::cout << state_map["speedLevel"] << std::endl;
    std::cout << state_map["gait"] << std::endl;
    std::cout << state_map["joystick"] << std::endl;
    std::cout << state_map["dance"] << std::endl;
    std::cout << state_map["continuousGait"] << std::endl;
    std::cout << state_map["economicGait"] << std::endl;
    
    sport_client.RiseSit();
    sleep(3);
    // 获取高层状态
    sport_client.GetState(state_name, state_map);
    std::cout << state_map["state"] << std::endl;
    std::cout << state_map["bodyHeight"] << std::endl;
    std::cout << state_map["footRaiseHeight"] << std::endl;
    std::cout << state_map["speedLevel"] << std::endl;
    std::cout << state_map["gait"] << std::endl;
    std::cout << state_map["joystick"] << std::endl;
    std::cout << state_map["dance"] << std::endl;
    std::cout << state_map["continuousGait"] << std::endl;
    std::cout << state_map["economicGait"] << std::endl;
    

    return 0;
}
