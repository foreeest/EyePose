#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp" // 假设使用std_msgs的Float64MultiArray消息类型

#include <thread> // 包含线程库
#include <mutex>  // 包含互斥锁
#include <unistd.h>
#include <vector>

#include <cmath>
#include <iostream>
#include <functional>
#include <fstream>
#include "Interpolator_cubic.cpp"
#include "IK_solution.cpp"
#include "ini.h"
#include "joint_motion.h"
#include "print_rci.h"
#include "rci_data/command_types.h"
#include "rci_data/robot_datas.h"
#include "robot.h"
#include "move.h"

# define NMBD 7 //机械臂关节数量，至于为什么叫这个，参考IK_solution.cpp

using namespace std;
using namespace Eigen;

using namespace xmate;
using JointControl = std::function<JointPositions(RCI::robot::RobotState robot_state)>;

// 全局共享变量，存储控制信号
std::vector<float> control_signal(7,0.0f);//这个7可不是关节数量，是空间6个自由度加一个控制信号，别搞错了
// 互斥锁，用于同步访问共享变量
std::mutex control_signal_mutex;

// 全局共享变量，存储状态信号
std::vector<float> state_signal(13,0.0f);//这个13是空间6个自由度加7个关节角度
// 互斥锁，用于同步访问共享变量
std::mutex state_signal_mutex;

// 消息回调函数
void control_msg_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    // 锁定互斥锁，确保线程安全
    std::lock_guard<std::mutex> lock(control_signal_mutex);
    
    // 清空旧的控制信号
    control_signal.clear();
    
    // 将消息中的浮点数复制到共享变量中
    for (auto &number : msg->data)
    {
        control_signal.push_back(static_cast<float>(number));
    }
    
    // 这里可以添加其他需要在主线程中执行的代码
}

void send_state_signal(rclcpp::Node::SharedPtr node, std::vector<float>& state_signal, std::mutex& state_signal_mutex)
{
    auto publisher = node->create_publisher<std_msgs::msg::Float64MultiArray>("state_msg", 10);//队列大小设置为1就得了，也不需要保存那么多，反正那边也是写进共享内存里
    while (rclcpp::ok())
    {
        std_msgs::msg::Float64MultiArray msg;
        {
            std::lock_guard<std::mutex> lock(state_signal_mutex);
            msg.data.assign(state_signal.begin(), state_signal.end());
        }
        publisher->publish(msg);
        // RCLCPP_INFO(node->get_logger(), "Published to state_msg topic");
        rclcpp::sleep_for(std::chrono::milliseconds(50)); // 调整发布频率
    }
}
int main(int argc, char **argv)
{
    //----------------------------------------R***O***S***2***节***点---------------------------
    // 初始化ROS2节点
    rclcpp::init(argc, argv);
    
    // 创建ROS2节点对象
    auto node = rclcpp::Node::make_shared("control_node");
    
    // 创建消息订阅者
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub =
        node->create_subscription<std_msgs::msg::Float64MultiArray>(
            "control_msg", rclcpp::SystemDefaultsQoS(),
            std::bind(&control_msg_callback, std::placeholders::_1));
    
    // 创建辅助线程，用于运行ROS2的事件循环
    std::thread spin_thread([=]() {
        rclcpp::spin(node);
    });
    
    // 创建发送线程，用于定期发布消息
    std::thread sender_thread([=]() {
        send_state_signal(node, state_signal, state_signal_mutex);
    });

    // 主线程可以在这里执行其他任务
    //----------------------------------------I***K***初***始***化---------------------------
    double pi_over_2 = M_PI / 2.0;
    double pi_over_180 = M_PI / 180.0;

    Eigen::Matrix<double, 7, 4> DH;
    std::vector<double> joint_lower_bond;
    std::vector<double> joint_upper_bond;
    Eigen::Matrix4d First_bias = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d EE_bias;
    // std::vector<double> angles;
    Eigen::VectorXd angles(NMBD); 
    
    //xmate3 DH参数
    DH << 0, 0, 0.3415, 0,
                0, -pi_over_2, 0, 0,
                0, pi_over_2, 0.394, 0,
                0, -pi_over_2, 0, 0,
                0, pi_over_2, 0.366, 0,
                0, -pi_over_2, 0, 0,
                0, pi_over_2, 0.2503, 0;

    EE_bias<< 1, 0, 0, 0.0195, //最后一个是x偏移
               0, 1, 0, 0,//最后一个是y偏移
               0, 0, 1, 0.066,//最后一个是z偏移
               0, 0, 0, 1;

    // 初始化关节角度的下界
    joint_lower_bond = {-170 * pi_over_180, -120 * pi_over_180, -170 * pi_over_180, -120 * pi_over_180, -170 * pi_over_180, -120 * pi_over_180, -360 * pi_over_180};
    // 初始化关节角度的上界
    joint_upper_bond = {170 * pi_over_180, 120 * pi_over_180, 170 * pi_over_180, 120 * pi_over_180, 170 * pi_over_180, 120 * pi_over_180, 360 * pi_over_180};
    
    IK_solution IK(DH,joint_lower_bond,joint_upper_bond,First_bias,EE_bias); //初始化IK解算器对象

    //-------------------------------机***械***臂***连***接***初***始***化***归***位---------------------------
    std::string ipaddr = "10.1.1.160";
    std::string name = "callback";
    uint16_t port = 1337;
    std::string file = "xmate.ini";
    INIParser ini;
    if (ini.ReadINI(file)) {
        ipaddr = ini.GetString("network", "ip");
        port = static_cast<uint16_t>(ini.GetInt("network", "port"));
    }
    // RCI连接机器人
    xmate::Robot robot(ipaddr, port,XmateType::XMATE3_PRO,false);
    //防止网络连接失败
    sleep(2);
    int res = robot.getMotorState();
    std::cout << "机器人上电状态:" << res << std::endl;
    int power_state = 1;
    robot.setMotorPower(power_state);

    std::array<double,NMBD> q_current;
    std::array<double,NMBD> q_init = {{0,M_PI/6,0,M_PI/3,0,M_PI/2,0}};
    q_current = robot.receiveRobotState().q;
    MOVEJ(0.2,q_current,q_init,robot);//移动到初始位置
        
    //开始运动前先设置控制模式和运动模式
    robot.startMove(RCI::robot::StartMoveRequest::ControllerMode::kJointPosition,
                    RCI::robot::StartMoveRequest::MotionGeneratorMode::kJointPosition);
    JointPositions output; //定义回调函数返回值
    
    std::string joint_callback = "joint_callback";
    JointControl joint_position_callback;

    //----------------------------------------创***建***插***值***器---------------------------
    std::vector<double> joint_pos(std::begin(q_init), std::end(q_init));

    // Interpolator interpolator(joint_pos);//创建插值器对象
    // std::vector<std::array<double,NMBD>>result_mat = interpolator.Step(joint_pos);//创建初始插值


    //-------------------------------------一***些***参***数----------------------------------

    // std::array<double, 7> init_position;
    // std::array<double, 7> joint_delta;
    // static bool init = true;
    double time = 0;
    int count = 0;
    int M = 50; //50ms取一个控制值
    // double max_tran = 0.01; //最大距离差和角度差，高于这个不接收新位置 0.01m=1cm
    // double max_rota = M_PI/36; //大概5度？没用上，最后设置为按照当前位置加一个变量来运动了
    double scale_tran = 0.01/1000;
    double scale_rota = 0.02/1000; //平移和旋转速度的缩放因子，修改这个可以修改末端执行器目标的步长
    double scale_v = 0.5; //关节速度缩放因子，改这个可以修改关节的移动速度，具体来说就是每一步角度变化相对于IK解的比例

    Eigen::VectorXd action(NMBD); //action是个7位的玩意，因为是7轴的机械臂

    // Eigen::VectorXd angles(q_init);
    for(int i=0;i<NMBD;++i){
        angles[i]=q_current[i];//转换格式 std::array->Eigen::Vectorxd 我也不知道为啥当时脑抽弄一堆格式不一样的
    }
    Eigen::Vector3d target_pos = IK.get_position(angles);
    Eigen::Vector3d target_eular = IK.get_eular(angles);//初始化目标位置和欧拉角

    Eigen::Vector3d current_pos = IK.get_position(angles);
    Eigen::Vector3d current_eular = IK.get_eular(angles);//初始化目标位置和欧拉角

    cout<<"开始控制"<<endl;
    joint_position_callback = [&](RCI::robot::RobotState robot_state) -> JointPositions {
        time += 0.001; 

        if(robot_state.control_command_success_rate <0.9){
            std::cout<<"通信质量较差："<<robot_state.control_command_success_rate<<std::endl;
        }

        if(control_signal[6]<0.5){//控制信号[6]是0的时候有效，大概应该是0，但是它是个浮点数，所以就...
            
            q_current = robot_state.q;//获取当前姿态
            // cout<<"current q"<<endl;
            for(int i=0;i<NMBD;++i){
                angles[i]=q_current[i];//转换格式 std::array->Eigen::Vectorxd 我也不知道为啥当时脑抽弄一堆格式不一样的
                // cout<<q_current[i]<<" ";
            }
            // cout<<endl;
            current_pos = IK.get_position(angles);
            current_eular = IK.get_eular(angles);//计算当前末端执行其的位置和欧拉角

            //给共享内存赋值
            state_signal[0] = current_pos[0];
            state_signal[1] = current_pos[1];
            state_signal[2] = current_pos[2];//前三个是当前位置
            state_signal[3] = current_eular[0];
            state_signal[4] = current_eular[1];
            state_signal[5] = current_eular[2];//接下来三个是当前的欧拉角
            state_signal[6] = angles[0];
            state_signal[7] = angles[1];
            state_signal[8] = angles[2];
            state_signal[9] = angles[3];
            state_signal[10] = angles[4];
            state_signal[11] = angles[5];
            state_signal[12] = angles[6];//接下来7个是当前关节位置！！注意，这些值都是IK算出来的，并不是从机械臂的API里读取的

            // cout<<"current pos end eul"<<current_pos[0]<<" "<<current_pos[1]<<" "<<current_pos[2]<<" "<<current_eular[0]<<" "<<current_eular[1]<<" "<<current_eular[2]<<endl;

            // target_pos[0] = current_pos[0] + control_signal[0]*scale_tran;
            // target_pos[1] = current_pos[1] + control_signal[1]*scale_tran;
            // target_pos[2] = current_pos[2] + control_signal[2]*scale_tran;

            // target_eular[0] = current_eular[0] + control_signal[3]*scale_rota;
            // target_eular[1] = current_eular[1] + control_signal[4]*scale_rota;
            // target_eular[2] = current_eular[2] + control_signal[5]*scale_rota; //更新目标位置和欧拉角
            // cout<<"current target pos end eul"<<target_pos[0]<<" "<<target_pos[1]<<" "<<target_pos[2]<<" "<<target_eular[0]<<" "<<target_eular[1]<<" "<<target_eular[2]<<endl;

            target_pos[0] = target_pos[0] + control_signal[0]*scale_tran;
            target_pos[1] = target_pos[1] + control_signal[1]*scale_tran;
            target_pos[2] = target_pos[2] + control_signal[2]*scale_tran;

            target_eular[0] = target_eular[0] + control_signal[3]*scale_rota;
            target_eular[1] = target_eular[1] + control_signal[4]*scale_rota;
            target_eular[2] = target_eular[2] + control_signal[5]*scale_rota; //更新目标位置和欧拉角

            // cout<<"target pos end eul"<<target_pos[0]<<" "<<target_pos[1]<<" "<<target_pos[2]<<" "<<target_eular[0]<<" "<<target_eular[1]<<" "<<target_eular[2]<<endl;
            // cout<<"pos speed"<<control_signal[0]*scale_tran<<" "<<control_signal[1]*scale_tran<<" "<<control_signal[2]*scale_tran<<endl;
            // cout<<"eur speed"<<control_signal[3]*scale_rota<<" "<<control_signal[4]*scale_rota<<" "<<control_signal[5]*scale_rota<<endl;

            action = IK.IK_solver(angles,target_pos,target_eular);//算反向运动学

            // cout<<"action"<<endl;
            for(int i=0;i<NMBD;++i){ //目标的关节角度是当前角度加上反向运动学算出来的目标速度乘以一个常数变量
                joint_pos[i]=angles[i]+action[i]*scale_v;//这格式也太乱了，不同时间拉的屎同一时间扣到了我的头上
                // cout<<action[i]<<" ";
            }
            // cout<<endl;

            for(int i=0;i<NMBD;++i){
                output.q[i]=joint_pos[i];//给关节赋值
                
            }

        }
        else{//这时候控制信号应该给1了

            // 等待辅助线程结束
            std::cout<<"关闭辅助线程"<<std::endl;
            spin_thread.join();
    
            // 关闭ROS2节点
            std::cout<<"关闭ROS节点"<<std::endl;
            rclcpp::shutdown();

            std::cout<<"运动结束"<<std::endl;
            return MotionFinished(output);
            }
        return output;        
    };

    robot.Control(joint_position_callback);

    // int count = 0; //创建一个计数器
    // while(count<200){
    //     cout<<"wait"<<endl;
    //     for(int i=0;i<6;++i){
    //         cout<<control_signal[i]<<" ";
    //     }
    //     cout<<endl;
    //     count++;
    //     if(control_signal[6]>0.5){
    //         cout<<"检测到ESC按键，终止控制程序"<<endl;
    //         // 等待辅助线程结束
    //         spin_thread.join();
    
    //         // 关闭ROS2节点
    //         rclcpp::shutdown();
    
    //         break;
    //     }
    //     sleep(2);
    // }
    
    // 等待辅助线程结束
    spin_thread.join();
    
    // 关闭ROS2节点
    rclcpp::shutdown();
    
    return 0;
}