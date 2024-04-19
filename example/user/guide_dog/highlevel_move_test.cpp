#include <cmath>

#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>

#define TOPIC_HIGHSTATE "rt/sportmodestate"

// using namespace unitree::common; 

enum test_mode
{
  /*---Basic motion---*/
  normal_stand,       // 0 正常站立
  balance_stand,      // 1 平衡站立
  velocity_move,      // 2 速度控制
  trajectory_follow,  // 3 轨迹跟踪控制
  stand_down,         // 4 趴下
  stand_up,           // 5 站高
  damp,               // 6 阻尼模式，软急停
  recovery_stand,     // 7 恢复站立
  /*---Special motion ---*/
  sit,                // 8 坐下
  rise_sit,           // 9 从坐下状态下恢复
  stop_move = 99      // 停止运动
};

class Custom
{
public:
    explicit Custom(){}
    ~Custom(){}

    void InitSportClient();
    void HighStateHandler(const void *message);
    void Loop();
    void high_level_control();

    unitree_go::msg::dds_::SportModeState_ state;
    unitree::robot::go2::SportClient sport_client;
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> suber;
    unitree::common::ThreadPtr SengHighCmdThreadPtr;

    float dt = 0.002; // unit [second]

};

void Custom::InitSportClient(){
    sport_client.SetTimeout(10.0f);
    sport_client.Init();

    suber.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(TOPIC_HIGHSTATE));
    suber->InitChannel(std::bind(&Custom::HighStateHandler, this, std::placeholders::_1), 1);
}

void Custom::Loop(){
    // 新增线程可以实现loop function的功能

    // intervalMicrosec : 1微秒 = 0.000001秒
    // 当dt=0.002s
    // ntervalMicrosec = 2000us
    SengHighCmdThreadPtr = unitree::common::CreateRecurrentThreadEx("send_high_level_cmd_thread", UT_CPU_ID_NONE, dt*1e6, &Custom::high_level_control, this);
}

void Custom::high_level_control(){
    sport_client.StandDown();
    sleep(2);
    sport_client.StandUp();
}

void Custom::HighStateHandler(const void *message)
{
    state = *(unitree_go::msg::dds_::SportModeState_ *)message;

};

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
        exit(-1);
    }

    unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);// 传入本机的网卡地址（PC or Jetson Orin）
    sleep(1); // Wait for 1 second to obtain a stable state

    Custom custom;
    custom.InitSportClient();
    custom.Loop();

    while (1)
    {
        sleep(10);
    }
    return 0;
}