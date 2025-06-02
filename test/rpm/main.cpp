#include <stdio.h>
#include <ros/ros.h>

#include <ecat/EcatSlaveInfos.h>
#include <ecat/EcatState.h>
#include <ecat/Commands.h>
#include <ecat/Infos.h>

using namespace std;

enum class CommandMode {
    ESTOP = -2, STOP = -1, NO_ACTION = 0, VELOCITY = 1, POSITION = 2, HOMING = 3, INIT = 4, RESET = 5
};

ecat::EcatSlaveInfos ecat_slave_info_;
void ecatSlaveInfoCallBack(const ecat::EcatSlaveInfos ecat_slave_info) {
    ecat_slave_info_ = ecat_slave_info;
}

ecat::EcatState ecat_state_;
void ecatStateCallBack(const ecat::EcatState ecat_state) {
    ecat_state_ = ecat_state;
}

ecat::Infos info_;
void infoCallBack(const ecat::Infos info) {
    info_ = info;
}

int main_hz;
int wheel_num;
double dt;
std::vector<double> vel_profile;
int vel_profile_idx;

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "test");
    ros::NodeHandle nh("~");

    #if 1
    ros::param::get("~main_hz", main_hz);
    ros::param::get("~wheel_num", wheel_num);
    ros::param::get("~velocity", vel_profile);
    ros::param::get("~dt", dt);
    #else
    nh.getParam("main_hz", main_hz);
    nh.getParam("wheel_num", wheel_num);
    nh.getParam("velocity", vel_profile);
    nh.getParam("dt", dt);
    #endif

    // ecat_slave_info topic
    ros::Subscriber ecat_slave_info_sub = nh.subscribe("/ecat/ecat_slave_info", 1, ecatSlaveInfoCallBack);
    ecat_slave_info_.slaves = std::vector<ecat::EcatSlaveInfo>(wheel_num);
    // ecat_state topic
    ros::Subscriber ecat_state_sub = nh.subscribe("/ecat/ecat_state", 1, ecatStateCallBack);
    // info topic
    ros::Subscriber info_sub = nh.subscribe("/ecat/info", 1, infoCallBack);
    info_.slaves = std::vector<ecat::Info>(wheel_num);

    // commands topic
    ros::Publisher command_pub = nh.advertise<ecat::Commands>("/ecat/command", 100);
    static ecat::Commands commands;
    commands.slaves = std::vector<ecat::Command>(wheel_num);

    ros::Rate r(main_hz);

    printf("all motor checking\n");
    while (ros::ok()){
        if (ecat_state_.allMotorInited) {
            break;
        }
        ros::spinOnce();
        r.sleep();
    }
    printf("all motor inited\n");

    printf("dt: %f sec, profile: %d ea\n", dt, vel_profile.size());

    double time_now = ros::Time::now().toSec();
    double time_pre = time_now;

    while (ros::ok()){
        time_now = ros::Time::now().toSec();

        if ((time_now-time_pre) > dt) {
            time_pre = time_now;

            if (vel_profile_idx < vel_profile.size()) {
                ROS_INFO("%f", vel_profile[vel_profile_idx]);

                commands.slaves[0].command = (int)CommandMode::VELOCITY;
                commands.slaves[0].uu = vel_profile[vel_profile_idx] / 30.0;
                commands.slaves[0].uu_per_sec = 0.0;
                command_pub.publish(commands);

                vel_profile_idx++;
            } else {
                break;
            }
        }

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}