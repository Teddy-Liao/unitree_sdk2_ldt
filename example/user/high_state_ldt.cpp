#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

//高层状态topic，其中rt表示实时，lf表示低频
#define TOPIC_HIGHSTATE "rt/sportmodestate"

using namespace unitree::robot;

//获取运动状态的回调函数
void HighStateHandler(const void* message)
{
    unitree_go::msg::dds_::SportModeState_ state = *(unitree_go::msg::dds_::SportModeState_*)message;

    //打印输出机器狗位置
    std::cout<<"position: "
            <<state.position()[0]<<", "
            <<state.position()[1]<<", "
            <<state.position()[2]<<std::endl;
    //打印输出机器狗速度
    std::cout<<"velocity: "
            <<state.velocity()[0]<<", "
            <<state.velocity()[1]<<", "
            <<state.velocity()[2]<<std::endl;
    //打印输出机器狗姿态四元数 (w,x,y,z)
    std::cout<<"quaternion: "
            <<state.imu_state().quaternion()[0]<<", "
            <<state.imu_state().quaternion()[1]<<", "
            <<state.imu_state().quaternion()[2]<<", "
            <<state.imu_state().quaternion()[3]<<std::endl;
    //打印运动模式
    std::cout<<"sport mode: " << (unsigned int)state.mode() << std::endl;
    //打印是否处于舞蹈状态
    std::cout<<"dance mode: " << state.progress() << std::endl;
    //打印步态状态
    std::cout<<"gait type: " << (unsigned int)state.gait_type() << std::endl;
    //打印抬腿高度
    std::cout<<"foot raise height: " << state.foot_raise_height() << std::endl;
    //打印足端力
    std::cout<<"foot force: "
            <<state.foot_force()[0]<<", "
            <<state.foot_force()[1]<<", "
            <<state.foot_force()[2]<<", "
            <<state.foot_force()[3]<<std::endl;
    //打印分割线
    std::cout << "----------------------" << std::endl;
}


int main(int argc, char **argv)
{
  if (argc < 2)
  {
    std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
    exit(-1);
  }
  unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);

  //创建一个Subscriber
  ChannelSubscriber<unitree_go::msg::dds_::SportModeState_> suber(TOPIC_HIGHSTATE);

  //初始化Channel
  suber.InitChannel(HighStateHandler);

  while(1)
  {
    usleep(20000);
  }

  return 0;
}