project(helloworld)
cmake_minimum_required(VERSION 3.5)

SET(CMAKE_CXX_STANDARD 17)

include_directories(/usr/local/include/ddscxx /usr/local/include/iceoryx/v2.0.2)
link_libraries(unitree_sdk2 ddsc ddscxx rt pthread)

# 
add_executable(test_publisher example/helloworld/publisher.cpp example/helloworld/HelloWorldData.cpp)
add_executable(test_subscriber example/helloworld/subscriber.cpp example/helloworld/HelloWorldData.cpp)
# 
add_executable(high_follow_sin example/high_level/follow_sin.cpp)
add_executable(sportmode_test example/high_level/sportmode_test.cpp)
add_executable(low_level example/low_level/low_level.cpp)
add_executable(wireless example/wireless/wireless.cpp)
add_executable(advanced_gamepad example/advanced_gamepad/main.cpp)
add_executable(state_machine_example example/state_machine/main.cpp)
add_executable(test_jsonize example/jsonize/test_jsonize.cpp)
add_executable(robot_state_client_example example/client/robot_state_client_example.cpp)
add_executable(sport_client_example example/client/sport_client_example.cpp)
add_executable(video_client_example example/client/video_client_example.cpp)
add_executable(vui_client_example example/client/vui_client_example.cpp)
#
add_executable(low_level_ldt example/user/low_level_ldt.cpp)
