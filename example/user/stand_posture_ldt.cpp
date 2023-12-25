#include <cmath>
#include <signal.h>
#include <unistd.h>

#include <unitree/robot/go2/sport/sport_client.hpp>

bool stopped = false;

void sigint_handler(int sig)
{
  if (sig == SIGINT)
  {
    stopped = true;
  }
}

int main(int argc, char **argv)
{
  if (argc < 2)
  {
    std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
    exit(-1);
  }
  unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
  //argv[1]由终端传入，为机器人连接的网卡名称

  //创建一个sport_client对象
  unitree::robot::go2::SportClient sport_client;
  sport_client.SetTimeout(10.0f); //超时时间
  sport_client.Init(); //初始化sport client

  double t = 0; //程序运行时间
  double dt = 0.01; //控制步长

  //创建一个signal对象用于捕获程序退出的信号
  //SIGINT 信号:通常是由用户按下 Ctrl+C 触发）
  signal(SIGINT, sigint_handler);
  //进入一个无限循环,直到用户按下Ctrl+C
  while (1)
  {
    t += dt;
    //姿态跟踪一个三角函数轨迹
    //设置 Go2 平衡站立或移动时的机体姿态角。欧拉角采用绕机体相对轴和 z-y-x 旋转顺序的表示方式。
    sport_client.Euler(0.2 * sin(2 * t), 0.2 * cos(2 * t) - 0.2, 0.);
    sport_client.BalanceStand();
    //使用 usleep 控制程序的运行速度，以达到一定的时间间隔。
    usleep(int(dt * 1000000));
    if(stopped)
    {
      break;
    }

  }
  //程序退出时复位姿态
  sport_client.Euler(0, 0, 0);
  return 0;
}