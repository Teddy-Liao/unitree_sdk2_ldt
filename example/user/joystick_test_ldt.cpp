#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/WirelessController_.hpp>
#include <unitree/robot/client/client.hpp>

#define TOPIC_JOYSTICK "rt/wirelesscontroller"

using namespace unitree::common;

// 遥控器键值联合体
typedef union
{
  struct
  {
    uint8_t R1 : 1;
    uint8_t L1 : 1;
    uint8_t start : 1;
    uint8_t select : 1;
    uint8_t R2 : 1;
    uint8_t L2 : 1;
    uint8_t F1 : 1;
    uint8_t F2 : 1;
    uint8_t A : 1;
    uint8_t B : 1;
    uint8_t X : 1;
    uint8_t Y : 1;
    uint8_t up : 1;
    uint8_t right : 1;
    uint8_t down : 1;
    uint8_t left : 1;
  } components;
  uint16_t value;
} xKeySwitchUnion;

// 获取运动状态的回调函数
void JoystickHandler(const void *message)
{
  unitree_go::msg::dds_::WirelessController_ joystick = *(unitree_go::msg::dds_::WirelessController_ *)message;
  
  // 遥控器原始数据
  std::cout << "lx: " << joystick.lx() << std::endl     // 左摇杆x
            << "ly: " << joystick.ly() << std::endl     // 左摇杆y
            << "rx: " << joystick.rx() << std::endl     // 右摇杆x
            << "ry: " << joystick.ry() << std::endl     // 右摇杆y
            << "keys: " << joystick.keys() << std::endl // 键值
            << "------" << std::endl;
  xKeySwitchUnion key;
  key.value = joystick.keys();

  // 判断某个按键是否被按下
  if ((int)key.components.A == 1)
  {
    std::cout << "The key A is pressed " << std::endl;
  }

  if ((int)key.components.B == 1)
  {
    std::cout << "The key B is pressed " << std::endl;
  }

  if ((int)key.components.X == 1)
  {
    std::cout << "The key X is pressed " << std::endl;
  }

  if ((int)key.components.Y == 1)
  {
    std::cout << "The key Y is pressed " << std::endl;
  }

  if ((int)key.components.L1 == 1)
  {
    std::cout << "The key L1 is pressed " << std::endl;
  }
 
  if ((int)key.components.L2 == 1)
  {
    std::cout << "The key L2 is pressed " << std::endl;
  }

  if ((int)key.components.R1 == 1)
  {
    std::cout << "The key R1 is pressed " << std::endl;
  }
 
  if ((int)key.components.R2 == 1)
  {
    std::cout << "The key R2 is pressed " << std::endl;
  }
}

int main(int argc, char **argv)
{
    // 初始化sdk接口
  if (argc < 2)
  {
    std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
    exit(-1);
  }

  unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);

  unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::WirelessController_> joystick_suber;
  joystick_suber.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::WirelessController_>(TOPIC_JOYSTICK));
  joystick_suber->InitChannel(JoystickHandler, 1);

  while (1)
  {
    usleep(20000);
  }
  return 0;
}