#include <boost/thread.hpp>
#include <boost/lockfree/queue.hpp>
#include <std_msgs/Empty.h>

#include "ecat/Commands.h"
#include "ecat/Infos.h"

#include "ecat/Control.h"
#include "ecat/Controls.h"

#include "main.hpp"

#include <ros/ros.h>

#include "ethercat/ethercatMain.h"

int motor_num;
std::vector<double> gear_ratio;
std::vector<double> motor_tick;
std::vector<double> hom_offset;
std::vector<double> min_val;
std::vector<double> max_val;
std::vector<double> pully_pd_mm;
std::vector<double> screw_lead_mm;
std::vector<double> wheel_radius;
std::vector<double> min_val_per_sec;
std::vector<double> max_val_per_sec;
std::vector<double> inv_motor_in_arr;
double qmsg_ts_period;
double pub_ts_period;
std::string ifname;
std::vector<int> ecat_drive_mode;
std::vector<int> slave_arch;
int freerun;
double acc_dec_factor;
int32_t sdo_ca_support;

ecat::Infos infos;

std::vector<int32_t> cmd_position;
std::vector<int32_t> cmd_rpm;
void baseCommandCallback(ecat::Commands commands) {
    static std::vector<ecat::Command> command = std::vector<ecat::Command>(motor_num);
    static std::vector<ecat::Info> info = std::vector<ecat::Info>(motor_num);

    static double rad_in;
    static double bevel_rev;
    static double gear_rev;
    static double encoder;
    static double rad_sec_in;
    static double rad_min;
    static double bevel_rev_min;
    static double gear_rev_min;
    static double time_diff;
    static double rpm_in;
    static double m_in;
    static double m_sec_in;

    for (int i=0; i<commands.slaves.size(); i++) {
        command[i] = commands.slaves[i];
    }

    for (int i=0; i<infos.slaves.size(); i++) {
        info[i] = infos.slaves[i];
    }

    for (int i=0; i<command.size(); i++) {
        switch (command[i].command) {
            case (int)CommandMode::ESTOP:
            case (int)CommandMode::STOP:
                pEcatSlaveCommandsMsg[IDX_ECAT_SLV_START+i]->type = (int)EcatSlaveCmdType::STOP;

                ecatSlaveCommandsMsg_enabled[IDX_ECAT_SLV_START+i] = 1;
                break;
            case (int)CommandMode::NO_ACTION:
                break;
            case (int)CommandMode::VELOCITY:
                #if 0
                cmd_rpm[i] = command[i].uu * inv_motor_in_arr[i];
                #else
                if (min_val[i] > cmd_rpm[i] || cmd_rpm[i] > max_val[i]) {
                    ROS_WARN("[slave:%d][CommandMode::VELOCITY]min(%f>%f) || max(%f>%f) error", i+IDX_ECAT_SLV_START, min_val[i], cmd_rpm[i], cmd_rpm[i], max_val[i]);
                    break;
                }

                switch (ecat_drive_mode[i]) {
                    case (int)EcatDriveMode::TORQUE:
                        break;
                    case (int)EcatDriveMode::VELOCITY:
                        switch (slave_arch[i]) {
                            case (int)EcatSlaveArch::ANGULAR_WHEEL_WELCON:
                                pEcatSlaveCommandsMsg[IDX_ECAT_SLV_START+i]->type = (int)EcatSlaveCmdType::RUN;
                                pEcatSlaveCommandsMsg[IDX_ECAT_SLV_START+i]->command.rpm = VEL_RAD_TO_RPM(VEL_L_TO_VEL_RAD((command[i].uu * inv_motor_in_arr[i]), wheel_radius[i])) * gear_ratio[i];

                                ecatSlaveCommandsMsg_enabled[IDX_ECAT_SLV_START+i] = 0;
                                ecatSlaveCommandsMsg_callbacked[IDX_ECAT_SLV_START+i] = 1;
                                break;
                            case (int)EcatSlaveArch::RPM_LS:
                                pEcatSlaveCommandsMsg[IDX_ECAT_SLV_START+i]->type = (int)EcatSlaveCmdType::RUN;
                                pEcatSlaveCommandsMsg[IDX_ECAT_SLV_START+i]->command.rpm = command[i].uu * inv_motor_in_arr[i] * gear_ratio[i];

                                ecatSlaveCommandsMsg_enabled[IDX_ECAT_SLV_START+i] = 0;
                                ecatSlaveCommandsMsg_callbacked[IDX_ECAT_SLV_START+i] = 1;
                            case (int)EcatSlaveArch::RPM_WELCON:
                                pEcatSlaveCommandsMsg[IDX_ECAT_SLV_START+i]->type = (int)EcatSlaveCmdType::RUN;
                                pEcatSlaveCommandsMsg[IDX_ECAT_SLV_START+i]->command.rpm = command[i].uu * inv_motor_in_arr[i] * gear_ratio[i];

                                ecatSlaveCommandsMsg_enabled[IDX_ECAT_SLV_START+i] = 0;
                                ecatSlaveCommandsMsg_callbacked[IDX_ECAT_SLV_START+i] = 1;
                            default:
                                break;
                        }
                        break;
                    case (int)EcatDriveMode::PP:
                        break;
                    case (int)EcatDriveMode::PV:
                        break;
                    default:
                        PRINT_DETAIL_LOG(PRINT_DETAIL_LOG_EN, __LINE__);
                        ROS_ERROR("unknown case[ecat_drive_mode[%d](%dd)]", i, ecat_drive_mode[i]);
                        break;
                }
                #endif
                break;
            case (int)CommandMode::POSITION:
                switch (slave_arch[i]) {
                    case (int)EcatSlaveArch::ANGULAR_SWIVEL:
                        rad_in = command[i].uu;
                        if (min_val[i] > rad_in || rad_in > max_val[i]) {
                            ROS_WARN("[slave_num:%d][CommandMode::POSITION:ANGULAR_SWIVEL]min_val(%f>%f) || max_val(%f>%f) error", i+IDX_ECAT_SLV_START, min_val[i], rad_in, rad_in, max_val[i]);
                            break;
                        }
                        rad_in -= hom_offset[i];
                        bevel_rev = RAD_TO_REV(rad_in);
                        gear_rev = bevel_rev * gear_ratio[i];
                        encoder = gear_rev * motor_tick[i];
                        cmd_position[i] = (int)encoder;
                        rad_sec_in = command[i].uu_per_sec;
                        if (min_val_per_sec[i] > rad_sec_in || rad_sec_in > max_val_per_sec[i]) {
                            ROS_WARN("[slave_num:%d][CommandMode::POSITION:ANGULAR_SWIVEL]min_val_per_sec(%f>%f) || max_val_per_sec(%f>%f) error", i+IDX_ECAT_SLV_START, min_val_per_sec[i], rad_sec_in, rad_sec_in, max_val_per_sec[i]);
                            break;
                        }
                        rad_min = rad_sec_in * MIN_TO_SEC(1.0);
                        bevel_rev_min = RAD_TO_REV(rad_min);
                        gear_rev_min = bevel_rev_min * gear_ratio[i];
                        cmd_rpm[i] = gear_rev_min;
                        #if 0
                        printf("[in ] rad: %7.3f, rad_sec: %7.3f, pos: %10d, rpm: %5d\n", rad_in, rad_sec_in, cmd_position[i], cmd_rpm[i]);
                        #endif

                        switch (ecat_drive_mode[i]) {
                            case (int)EcatDriveMode::TORQUE:
                                break;
                            case (int)EcatDriveMode::VELOCITY:
                                break;
                            case (int)EcatDriveMode::PP:
                                pEcatSlaveCommandsMsg[IDX_ECAT_SLV_START+i]->type = (int)EcatSlaveCmdType::RUN;
                                pEcatSlaveCommandsMsg[IDX_ECAT_SLV_START+i]->command.rpm = cmd_rpm[i] * inv_motor_in_arr[i];
                                pEcatSlaveCommandsMsg[IDX_ECAT_SLV_START+i]->command.position = cmd_position[i] * inv_motor_in_arr[i];
                                pEcatSlaveCommandsMsg[IDX_ECAT_SLV_START+i]->command.acc = cmd_rpm[i] * inv_motor_in_arr[i] * acc_dec_factor;
                                pEcatSlaveCommandsMsg[IDX_ECAT_SLV_START+i]->command.dec = cmd_rpm[i] * inv_motor_in_arr[i] * acc_dec_factor;

                                ecatSlaveCommandsMsg_enabled[IDX_ECAT_SLV_START+i] = 0;
                                ecatSlaveCommandsMsg_callbacked[IDX_ECAT_SLV_START+i] = 1;
                                break;
                            case (int)EcatDriveMode::PV:
                                break;
                            default:
                                PRINT_DETAIL_LOG(PRINT_DETAIL_LOG_EN, __LINE__);
                                ROS_ERROR("unknown case[ecat_drive_mode[%d](%dd)]", i, ecat_drive_mode[i]);
                                break;
                        }
                        break;
                    case (int)EcatSlaveArch::LINEAR_PULLY:
                        m_in = command[i].uu;
                        if (min_val[i] > m_in || m_in > max_val[i]) {
                            ROS_WARN("[slave_num:%d][CommandMode::POSITION:LINEAR_PULLY]min_val(%f>%f) || max_val(%f>%f) error", i+IDX_ECAT_SLV_START, min_val[i], m_in, m_in, max_val[i]);
                            break;
                        }
                        m_in -= hom_offset[i];
                        gear_rev = (m_in / (pully_pd_mm[i] * M_PI)) * gear_ratio[i];
                        encoder = gear_rev * motor_tick[i];
                        cmd_position[i] = (int)encoder;
                        m_sec_in = command[i].uu_per_sec;
                        if (min_val_per_sec[i] > m_sec_in || m_sec_in > max_val_per_sec[i]) {
                            ROS_WARN("[slave_num:%d][CommandMode::POSITION:LINEAR_PULLY]min_val_per_sec(%f>%f) || max_val_per_sec(%f>%f) error", i+IDX_ECAT_SLV_START, min_val_per_sec[i], m_sec_in, m_sec_in, max_val_per_sec[i]);
                            break;
                        }
                        gear_rev_min = (m_sec_in / (pully_pd_mm[i] * M_PI)) * MIN_TO_SEC(1.0) * gear_ratio[i];
                        cmd_rpm[i] = gear_rev_min;
                        #if 0
                        printf("[in ] m: %7.3f, m_sec: %7.3f, pos: %10d, rpm: %5d\n", m_in, m_sec_in, cmd_position[i], cmd_rpm[i]);
                        #endif

                        switch (ecat_drive_mode[i]) {
                            case (int)EcatDriveMode::TORQUE:
                                break;
                            case (int)EcatDriveMode::VELOCITY:
                                break;
                            case (int)EcatDriveMode::PP:
                                pEcatSlaveCommandsMsg[IDX_ECAT_SLV_START+i]->type = (int)EcatSlaveCmdType::RUN;
                                pEcatSlaveCommandsMsg[IDX_ECAT_SLV_START+i]->command.rpm = cmd_rpm[i] * inv_motor_in_arr[i];
                                pEcatSlaveCommandsMsg[IDX_ECAT_SLV_START+i]->command.position = cmd_position[i] * inv_motor_in_arr[i];
                                pEcatSlaveCommandsMsg[IDX_ECAT_SLV_START+i]->command.acc = cmd_rpm[i] * inv_motor_in_arr[i] * acc_dec_factor;
                                pEcatSlaveCommandsMsg[IDX_ECAT_SLV_START+i]->command.dec = cmd_rpm[i] * inv_motor_in_arr[i] * acc_dec_factor;

                                ecatSlaveCommandsMsg_enabled[IDX_ECAT_SLV_START+i] = 0;
                                ecatSlaveCommandsMsg_callbacked[IDX_ECAT_SLV_START+i] = 1;
                                break;
                            case (int)EcatDriveMode::PV:
                                break;
                            default:
                                PRINT_DETAIL_LOG(PRINT_DETAIL_LOG_EN, __LINE__);
                                ROS_ERROR("unknown case[ecat_drive_mode[%d](%dd)]", i, ecat_drive_mode[i]);
                                break;
                        }
                        break;
                    case (int)EcatSlaveArch::LINEAR_SCREW:
                        m_in = command[i].uu;
                        if (min_val[i] > m_in || m_in > max_val[i]) {
                            ROS_WARN("[slave_num:%d][CommandMode::POSITION:LINEAR_SCREW]min_val(%f>%f) || max_val(%f>%f) error", i+IDX_ECAT_SLV_START, min_val[i], m_in, m_in, max_val[i]);
                            break;
                        }
                        m_in -= hom_offset[i];
                        gear_rev = (m_in / screw_lead_mm[i]) * gear_ratio[i];
                        encoder = gear_rev * motor_tick[i];
                        cmd_position[i] = (int)encoder;
                        m_sec_in = command[i].uu_per_sec;
                        if (min_val_per_sec[i] > m_sec_in || m_sec_in > max_val_per_sec[i]) {
                            ROS_WARN("[slave_num:%d][CommandMode::POSITION:LINEAR_PULLY]min_val_per_sec(%f>%f) || max_val_per_sec(%f>%f) error", i+IDX_ECAT_SLV_START, min_val_per_sec[i], m_sec_in, m_sec_in, max_val_per_sec[i]);
                            break;
                        }
                        gear_rev_min = (m_sec_in / screw_lead_mm[i]) * MIN_TO_SEC(1.0) * gear_ratio[i];
                        cmd_rpm[i] = gear_rev_min;
                        #if 0
                        printf("[in ] m: %7.3f, m_sec: %7.3f, pos: %10d, rpm: %5d\n", m_in, m_sec_in, cmd_position[i], cmd_rpm[i]);
                        #endif

                        switch (ecat_drive_mode[i]) {
                            case (int)EcatDriveMode::TORQUE:
                                break;
                            case (int)EcatDriveMode::VELOCITY:
                                break;
                            case (int)EcatDriveMode::PP:
                                pEcatSlaveCommandsMsg[IDX_ECAT_SLV_START+i]->type = (int)EcatSlaveCmdType::RUN;
                                pEcatSlaveCommandsMsg[IDX_ECAT_SLV_START+i]->command.rpm = cmd_rpm[i] * inv_motor_in_arr[i];
                                pEcatSlaveCommandsMsg[IDX_ECAT_SLV_START+i]->command.position = cmd_position[i] * inv_motor_in_arr[i];
                                pEcatSlaveCommandsMsg[IDX_ECAT_SLV_START+i]->command.acc = cmd_rpm[i] * inv_motor_in_arr[i] * acc_dec_factor;
                                pEcatSlaveCommandsMsg[IDX_ECAT_SLV_START+i]->command.dec = cmd_rpm[i] * inv_motor_in_arr[i] * acc_dec_factor;

                                ecatSlaveCommandsMsg_enabled[IDX_ECAT_SLV_START+i] = 0;
                                ecatSlaveCommandsMsg_callbacked[IDX_ECAT_SLV_START+i] = 1;
                                break;
                            case (int)EcatDriveMode::PV:
                                break;
                            default:
                                PRINT_DETAIL_LOG(PRINT_DETAIL_LOG_EN, __LINE__);
                                ROS_ERROR("unknown case[ecat_drive_mode[%d](%dd)]", i, ecat_drive_mode[i]);
                                break;
                        }
                        break;
                    default:
                        PRINT_DETAIL_LOG(PRINT_DETAIL_LOG_EN, __LINE__);
                        ROS_ERROR("unknown case[ecat_drive_mode[%d](%dd)]", i, ecat_drive_mode[i]);
                        break;
                }
                break;
            case (int)CommandMode::HOMING:
                pEcatSlaveCommandsMsg[IDX_ECAT_SLV_START+i]->type = (int)EcatSlaveCmdType::HOME;

                ecatSlaveCommandsMsg_enabled[IDX_ECAT_SLV_START+i] = 1;
                break;
            case (int)CommandMode::INIT:
                pEcatSlaveCommandsMsg[IDX_ECAT_SLV_START+i]->type = (int)EcatSlaveCmdType::INIT;

                ecatSlaveCommandsMsg_enabled[IDX_ECAT_SLV_START+i] = 1;
                break;
            case (int)CommandMode::RESET:
                pEcatSlaveCommandsMsg[IDX_ECAT_SLV_START+i]->type = (int)EcatSlaveCmdType::RESET;

                ecatSlaveCommandsMsg_enabled[IDX_ECAT_SLV_START+i] = 1;
                break;
            default:
                ROS_ERROR("unknown command : %d", command[i].command);
                break;
        }
    }
}

void commandsCallback(const ecat::Commands& commands) {
    baseCommandCallback(commands);
}

ecat::Commands srv_control_msg;
bool controlCallback(ecat::Control::Request &req, ecat::Control::Response &res) {
    for (int i=0; i<srv_control_msg.slaves.size(); i++) {
        srv_control_msg.slaves[i] = req.slaves[i];
    }
    baseCommandCallback(srv_control_msg);

#define SRV_SUCCESS	true
    res.success = SRV_SUCCESS;

    return true;
}

ecat::Commands srv_controls_msg;
bool controlsCallback(ecat::Controls::Request &req, ecat::Controls::Response &res) {
    static ecat::Command* pSlaveReq = &(req.s1);

    for (int i=0; i<srv_controls_msg.slaves.size(); i++) {
        srv_controls_msg.slaves[i] = pSlaveReq[i];
    }
    baseCommandCallback(srv_controls_msg);

#define SRV_SUCCESS	true
    res.success = SRV_SUCCESS;

    return true;
}

int main(int argc, char** argv) {
    std::string id = "ecat";
    ros::init(argc, argv, id.c_str());
    ros::NodeHandle nh("~");

    int main_hz;

    #if 1
    ros::param::get("~motor_num", motor_num);
    ros::param::get("~gear_ratio", gear_ratio);
    ros::param::get("~motor_tick", motor_tick);
    ros::param::get("~hom_offset", hom_offset);
    ros::param::get("~min_val", min_val);
    ros::param::get("~max_val", max_val);
    ros::param::get("~pully_pd_mm", pully_pd_mm);
    ros::param::get("~screw_lead_mm", screw_lead_mm);
    ros::param::get("~wheel_radius", wheel_radius);
    ros::param::get("~min_val_per_sec", min_val_per_sec);
    ros::param::get("~max_val_per_sec", max_val_per_sec);
    ros::param::get("~main_hz", main_hz);
    ros::param::get("~inv_motor_in_arr", inv_motor_in_arr);
    ros::param::get("~qmsg_ts_period", qmsg_ts_period);
    ros::param::get("~pub_ts_period", pub_ts_period);
    ros::param::get("~ifname", ifname);
    ros::param::get("~ecat_drive_mode", ecat_drive_mode);
    ros::param::get("~slave_arch", slave_arch);
    ros::param::get("~freerun", freerun);
    ros::param::get("~acc_dec_factor", acc_dec_factor);
    ros::param::get("~sdo_ca_support", sdo_ca_support);
    #else
    nh.getParam("motor_num", motor_num);
    nh.getParam("gear_ratio", gear_ratio);
    nh.getParam("motor_tick", motor_tick);
    nh.getParam("hom_offset", hom_offset);
    nh.getParam("min_val", min_val);
    nh.getParam("max_val", max_val);
    nh.getParam("pully_pd_mm", pully_pd_mm);
    nh.getParam("screw_lead_mm", screw_lead_mm);
    nh.getParam("wheel_radius", wheel_radius);
    nh.getParam("min_val_per_sec", min_val_per_sec);
    nh.getParam("max_val_per_sec", max_val_per_sec);
    nh.getParam("main_hz", main_hz);
    nh.getParam("inv_motor_in_arr", inv_motor_in_arr);
    nh.getParam("qmsg_ts_period", qmsg_ts_period);
    nh.getParam("pub_ts_period", pub_ts_period);
    nh.getParam("ifname", ifname);
    nh.getParam("ecat_drive_mode", ecat_drive_mode);
    nh.getParam("slave_arch", slave_arch);
    nh.getParam("freerun", freerun);
    nh.getParam("acc_dec_factor", acc_dec_factor);
    nh.getParam("sdo_ca_support", sdo_ca_support);
    #endif

    #if 0
    std::vector<std::string> keys;
    nh.getParamNames(keys);
    for(std::string key: keys) {
        printf("par: %s\n", key.c_str());
    }
    #endif

    ecatStateMsg.motorInited = std::vector<uint32_t>(motor_num);
    ecatStateMsg.ecatSlaveCommandsMsg_callbacked = std::vector<uint32_t>(motor_num);
    ecatStateMsg.ecatSlaveCommandsMsg_enabled = std::vector<uint32_t>(motor_num);
    ecatStateMsg.ecatSlaveCmdType = std::vector<std::string>(motor_num);
    ecatStateMsg.ecatSlaveCmdTypePre = std::vector<std::string>(motor_num);
    ecatStateMsg.subFsm = std::vector<uint32_t>(motor_num);
    ecatStateMsg.subFsmPP = std::vector<uint32_t>(motor_num);
    ecatStateMsg.statusword = std::vector<std::string>(motor_num);
    ecatStateMsg.controlword = std::vector<std::string>(motor_num);
    ecatStateMsg.fault = std::vector<uint32_t>(motor_num);
    ecatStateMsg.error_code = std::vector<std::string>(motor_num);

    infos.slaves = std::vector<ecat::Info>(motor_num);
    srv_control_msg.slaves = std::vector<ecat::Command>(motor_num);
    srv_controls_msg.slaves = std::vector<ecat::Command>(motor_num);
    ecatSlaveCommandsMsg.slaves = std::vector<ecat::EcatSlaveCommand>(motor_num);
    ecatSlaveInfosMsg.slaves = std::vector<ecat::EcatSlaveInfo>(motor_num);
    ecatMasterStateMsg.slave.slaves = std::vector<ecat::EcatSlaveState>(motor_num);
    srv_ecat_control_msg.slaves = std::vector<ecat::EcatSlaveCommand>(motor_num);

    if (inv_motor_in_arr.size() != motor_num ||
        slave_arch.size() != motor_num ||
        min_val_per_sec.size() != motor_num ||
        max_val_per_sec.size() != motor_num ||
        hom_offset.size() != motor_num ||
        min_val.size() != motor_num ||
        max_val.size() != motor_num ||
        pully_pd_mm.size() != motor_num ||
        screw_lead_mm.size() != motor_num ||
        wheel_radius.size() != motor_num ||
        motor_tick.size() != motor_num ||
        gear_ratio.size() != motor_num ||
        ecat_drive_mode.size() != motor_num) {
        ROS_ERROR("wrong rosparam: PARAM element sizeisn't matching PARAM size(motor_num)");

        return -1;
    }

    cmd_position = std::vector<int32_t>(motor_num);
    cmd_rpm = std::vector<int32_t>(motor_num);

    if (ifname.empty()) {
        ROS_ERROR("wrong rosparam: ifname is empty");

        return -1;
    }

    ecat_init();

    ros::Subscriber sub_command = nh.subscribe("/ecat/command", 100, commandsCallback);

    ros::Publisher pub_info = nh.advertise<ecat::Infos>("/ecat/info", 100);

    ros::ServiceServer srv_control = nh.advertiseService("/ecat/control", controlCallback);
    ros::ServiceServer srv_controls = nh.advertiseService("/ecat/controls", controlsCallback);

    ros::Publisher pub_ecatMasterState;
    ros::Publisher pub_ecatSlaveInfos;
    ros::Publisher pub_ecatState;

    ros::Subscriber sub_ecatSlaveCommands;

    boost::thread threadSendMsgMd1k;
    boost::thread thread_ecat_check;
    boost::thread thread_ecat;

    for (int i=0; i<motor_num; i++) {
        switch (ecat_drive_mode[i]) {
            case (int)EcatDriveMode::NO_ECAT:
            case (int)EcatDriveMode::TORQUE:
            case (int)EcatDriveMode::VELOCITY:
            case (int)EcatDriveMode::PP:
            case (int)EcatDriveMode::PV:
                break;
            default:
                PRINT_DETAIL_LOG(PRINT_DETAIL_LOG_EN, __LINE__);
                ROS_ERROR("unknown case[ecat_drive_mode[%d](%dd)]", i, ecat_drive_mode[i]);

                return -1;
        }
    }
    thread_ecat_check = boost::thread(ecat_check);
    #define ECAT_HZ 1000
    thread_ecat = boost::thread(ecat_loop, ECAT_HZ);

    pub_ecatMasterState = nh.advertise<ecat::EcatMasterState>("/ecat/ecat_master_state", 100);
    pub_ecatSlaveInfos = nh.advertise<ecat::EcatSlaveInfos>("/ecat/ecat_slave_info", 100);
    pub_ecatState = nh.advertise<ecat::EcatState>("/ecat/ecat_state", 100);

    sub_ecatSlaveCommands = nh.subscribe("/ecat/ecat_slave_command", 100, ecatSlaveCommandsCallBack);
    ros::ServiceServer srv_ecat_control = nh.advertiseService("/ecat/ecat_control", ecatSlaveControlCallback);

    for (int i=0; i<motor_num; i++) {
        pEcatSlaveCommandsMsg[IDX_ECAT_SLV_START+i]->type = (int)EcatSlaveCmdType::INIT;
        ecatSlaveCommandsMsg_enabled[IDX_ECAT_SLV_START+i] = 1;
    }
    printf("waiting motor init\n");

    ros::Rate r(main_hz);

    while(ros::ok()) {
        #if 0
        static uint16_t allMotorInited = 0;
        allMotorInited = 0;
        for (int i=0; i<motor_num; i++) {
            allMotorInited += motorInited[IDX_ECAT_SLV_START+i];
        }
        if (allMotorInited == motor_num) {
            break;
        }
        #else
        if (ecatStateMsg.allMotorInited == motor_num) {
            break;
        }
        #endif

        pub_ecatMasterState.publish(ecatMasterStateMsg);
        pub_ecatSlaveInfos.publish(ecatSlaveInfosMsg);
        pub_ecatState.publish(ecatStateMsg);

        ros::spinOnce();
        r.sleep();
    }
    printf("motor inited\n");

    double time_cur = ros::Time::now().toSec();
    double time_pre = time_cur;
    double time_diff;

    double qmsg_time_pre = time_cur;
    double qmsg_time_diff;

    double pub_time_pre = time_cur;
    double pub_time_diff;

    std::vector<ecat::Info> info = std::vector<ecat::Info>(motor_num);

    while(ros::ok())
    {
        time_cur = ros::Time::now().toSec();

        pub_ecatMasterState.publish(ecatMasterStateMsg);
        pub_ecatSlaveInfos.publish(ecatSlaveInfosMsg);
        pub_ecatState.publish(ecatStateMsg);

        pub_time_diff = time_cur - pub_time_pre;
        if (pub_time_diff > pub_ts_period) {
            pub_time_pre = time_cur;

            for (int i=0; i<motor_num; i++) {
                static double current = 0.0;
                current = pEcatSlaveInfosMsg[i+IDX_ECAT_SLV_START]->txPdo.tor_act * inv_motor_in_arr[i];
                static uint16_t statusword = 0;
                static uint32_t fault = 0;
                statusword = pEcatSlaveInfosMsg[i+IDX_ECAT_SLV_START]->txPdo.statusword;
                fault = (statusword >> STATUSWORD_BIT_FAULT) & 0x01;
                static uint16_t error_code = 0;
                error_code = pEcatSlaveInfosMsg[i+IDX_ECAT_SLV_START]->txPdo.error_code;

                info[i].header.stamp.fromSec(pEcatSlaveInfosMsg[i+IDX_ECAT_SLV_START]->txPdo.ts);
                if (fault && error_code == ENCODER_BATTERY_LOW) {
                    info[i].homing = HOMING_NOT_SET;
                } else {
                    info[i].homing = HOMING_DONE;
                }
                if (fault) {
                    info[i].state = error_code;
                } else {
                    info[i].state = 0;
                }

                static double rad_out = 0.0;
                static double rad_sec_out = 0;
                static double m_out = 0.0;
                static double m_sec_out = 0;
                static double position = 0.0;
                static double speed = 0.0;
                switch (ecat_drive_mode[i]) {
                    case (int)EcatDriveMode::VELOCITY:
                    case (int)EcatDriveMode::PV:
                        switch (slave_arch[i]) {
                            case (int)EcatSlaveArch::ANGULAR_WHEEL_WELCON:
                                info[i].uu = VEL_RAD_TO_VEL_L(VEL_RPM_TO_RAD((pEcatSlaveInfosMsg[i+IDX_ECAT_SLV_START]->info.rpm * inv_motor_in_arr[i]) / gear_ratio[i]), wheel_radius[i]);
                                info[i].uu_per_sec = 0.0;
                                break;
                            case (int)EcatSlaveArch::RPM_LS:
                                info[i].uu = pEcatSlaveInfosMsg[i+IDX_ECAT_SLV_START]->info.rpm * inv_motor_in_arr[i] / gear_ratio[i];
                                info[i].uu_per_sec = 0.0;
                                break;
                            case (int)EcatSlaveArch::RPM_WELCON:
                                info[i].uu = pEcatSlaveInfosMsg[i+IDX_ECAT_SLV_START]->info.rpm * inv_motor_in_arr[i] / gear_ratio[i];
                                info[i].uu_per_sec = 0.0;
                                break;
                            default:
                                break;
                        }
                        break;
                    case (int)EcatDriveMode::PP:
                        switch (slave_arch[i]) {
                            case (int)EcatSlaveArch::ANGULAR_SWIVEL:
                                rad_out = 0.0;
                                position = pEcatSlaveInfosMsg[i+IDX_ECAT_SLV_START]->txPdo.pos_act * inv_motor_in_arr[i];
                                rad_out = REV_TO_RAD(position / motor_tick[i] / gear_ratio[i]);
                                info[i].uu = rad_out + hom_offset[i];
                                rad_sec_out = 0.0;
                                speed = pEcatSlaveInfosMsg[i+IDX_ECAT_SLV_START]->info.rpm * inv_motor_in_arr[i];
                                rad_sec_out = REV_TO_RAD(speed * SEC_TO_MIN(1.0) / gear_ratio[i]);
                                info[i].uu_per_sec = rad_sec_out;
                                #if 0
                                printf("[out] rad: %7.3f, rad_sec: %7.3f\n", rad_out, rad_sec_out);
                                #endif
                                break;
                            case (int)EcatSlaveArch::LINEAR_PULLY:
                                m_out = 0.0;
                                position = pEcatSlaveInfosMsg[i+IDX_ECAT_SLV_START]->txPdo.pos_act * inv_motor_in_arr[i];
                                m_out = (position / motor_tick[i] / gear_ratio[i]) * (pully_pd_mm[i] * M_PI);
                                info[i].uu = m_out + hom_offset[i];
                                m_sec_out = 0.0;
                                speed = pEcatSlaveInfosMsg[i+IDX_ECAT_SLV_START]->info.rpm * inv_motor_in_arr[i];
                                m_sec_out = (speed * SEC_TO_MIN(1.0) / gear_ratio[i]) * (pully_pd_mm[i] * M_PI);
                                info[i].uu_per_sec = m_sec_out;
                                #if 0
                                printf("[out] m: %7.3f, m_sec: %7.3f\n", m_out, m_sec_out);
                                #endif
                                break;
                            case (int)EcatSlaveArch::LINEAR_SCREW:
                                m_out = 0.0;
                                position = pEcatSlaveInfosMsg[i+IDX_ECAT_SLV_START]->txPdo.pos_act * inv_motor_in_arr[i];
                                m_out = (position / motor_tick[i] / gear_ratio[i]) * screw_lead_mm[i];
                                info[i].uu = m_out + hom_offset[i];
                                m_sec_out = 0.0;
                                speed = pEcatSlaveInfosMsg[i+IDX_ECAT_SLV_START]->info.rpm * inv_motor_in_arr[i];
                                m_sec_out = (speed * SEC_TO_MIN(1.0) / gear_ratio[i]) * screw_lead_mm[i];
                                info[i].uu_per_sec = m_sec_out;
                                #if 0
                                printf("[out] m: %7.3f, m_sec: %7.3f\n", m_out, m_sec_out);
                                #endif
                                break;
                            default:
                                break;
                        }
                        break;
                    case (int)EcatDriveMode::TORQUE:
                        #if 0
                        info[i].uu = 789.0;
                        info[i].uu_per_sec = 321.0;
                        #else
                        switch (slave_arch[i]) {
                            case (int)EcatSlaveArch::ANGULAR_WHEEL_WELCON:
                                info[i].uu = VEL_RAD_TO_VEL_L(VEL_RPM_TO_RAD((pEcatSlaveInfosMsg[i+IDX_ECAT_SLV_START]->info.rpm * inv_motor_in_arr[i]) / gear_ratio[i]), wheel_radius[i]);
                                info[i].uu_per_sec = 0.0;
                                break;
                            case (int)EcatSlaveArch::RPM_LS:
                                info[i].uu = pEcatSlaveInfosMsg[i+IDX_ECAT_SLV_START]->info.rpm * inv_motor_in_arr[i] / gear_ratio[i];
                                info[i].uu_per_sec = 0.0;
                                break;
                            case (int)EcatSlaveArch::RPM_WELCON:
                                info[i].uu = pEcatSlaveInfosMsg[i+IDX_ECAT_SLV_START]->info.rpm * inv_motor_in_arr[i] / gear_ratio[i];
                                info[i].uu_per_sec = 0.0;
                                break;
                            case (int)EcatSlaveArch::ANGULAR_SWIVEL:
                                rad_out = 0.0;
                                position = pEcatSlaveInfosMsg[i+IDX_ECAT_SLV_START]->txPdo.pos_act * inv_motor_in_arr[i];
                                rad_out = REV_TO_RAD(position / motor_tick[i] / gear_ratio[i]);
                                info[i].uu = rad_out + hom_offset[i];
                                rad_sec_out = 0.0;
                                speed = pEcatSlaveInfosMsg[i+IDX_ECAT_SLV_START]->info.rpm * inv_motor_in_arr[i];
                                rad_sec_out = REV_TO_RAD(speed * SEC_TO_MIN(1.0) / gear_ratio[i]);
                                info[i].uu_per_sec = rad_sec_out;
                                #if 0
                                printf("[out] rad: %7.3f, rad_sec: %7.3f\n", rad_out, rad_sec_out);
                                #endif
                                break;
                            case (int)EcatSlaveArch::LINEAR_PULLY:
                                m_out = 0.0;
                                position = pEcatSlaveInfosMsg[i+IDX_ECAT_SLV_START]->txPdo.pos_act * inv_motor_in_arr[i];
                                m_out = (position / motor_tick[i] / gear_ratio[i]) * (pully_pd_mm[i] * M_PI);
                                info[i].uu = m_out + hom_offset[i];
                                m_sec_out = 0.0;
                                speed = pEcatSlaveInfosMsg[i+IDX_ECAT_SLV_START]->info.rpm * inv_motor_in_arr[i];
                                m_sec_out = (speed * SEC_TO_MIN(1.0) / gear_ratio[i]) * (pully_pd_mm[i] * M_PI);
                                info[i].uu_per_sec = m_sec_out;
                                #if 0
                                printf("[out] m: %7.3f, m_sec: %7.3f\n", m_out, m_sec_out);
                                #endif
                                break;
                            case (int)EcatSlaveArch::LINEAR_SCREW:
                                m_out = 0.0;
                                position = pEcatSlaveInfosMsg[i+IDX_ECAT_SLV_START]->txPdo.pos_act * inv_motor_in_arr[i];
                                m_out = (position / motor_tick[i] / gear_ratio[i]) * screw_lead_mm[i];
                                info[i].uu = m_out + hom_offset[i];
                                m_sec_out = 0.0;
                                speed = pEcatSlaveInfosMsg[i+IDX_ECAT_SLV_START]->info.rpm * inv_motor_in_arr[i];
                                m_sec_out = (speed * SEC_TO_MIN(1.0) / gear_ratio[i]) * screw_lead_mm[i];
                                info[i].uu_per_sec = m_sec_out;
                                #if 0
                                printf("[out] m: %7.3f, m_sec: %7.3f\n", m_out, m_sec_out);
                                #endif
                                break;
                            default:
                                break;
                        }
                        #endif
                        break;
                    case (int)EcatDriveMode::NO_ECAT:
                    default:
                        info[i].uu = 123456789.0;
                        info[i].uu_per_sec = 987654321.0;
                        break;
                }
            }

            for (int i=0; i<infos.slaves.size(); i++) {
                infos.slaves[i] = info[i];
            }
            pub_info.publish(infos);
        }

        qmsg_time_diff = time_cur - qmsg_time_pre;
        if (qmsg_time_diff > qmsg_ts_period) {
            qmsg_time_pre = time_cur;

            #if 1
            for (int i=0; i<motor_num; i++) {
                switch (ecat_drive_mode[i]) {
                    case (int)EcatDriveMode::NO_ECAT:
                        break;
                    case (int)EcatDriveMode::TORQUE:
                        // processed callback
                        break;
                    case (int)EcatDriveMode::VELOCITY:
                        #if 0
                        pEcatSlaveCommandsMsg[IDX_ECAT_SLV_START+i]->type = (int)EcatSlaveCmdType::RUN;
                        pEcatSlaveCommandsMsg[IDX_ECAT_SLV_START+i]->command.rpm = cmd_rpm[i] * inv_motor_in_arr[i];

                        ecatSlaveCommandsMsg_enabled[IDX_ECAT_SLV_START+i] = 1;
                        #else
                        // processed callback
                        #endif
                        break;
                    case (int)EcatDriveMode::PP:
                        // processed callback
                        break;
                    case (int)EcatDriveMode::PV:
                        // processed callback
                        break;
                    default:
                        PRINT_DETAIL_LOG(PRINT_DETAIL_LOG_EN, __LINE__);
                        ROS_ERROR("unknown case[ecat_drive_mode[%d](%dd)]", i, ecat_drive_mode[i]);
                        break;
                }
            }
            #endif
        }

        ros::spinOnce();
        r.sleep();
    }

    #if 1
    for (int i=0; i<motor_num; i++) {
        pEcatSlaveCommandsMsg[IDX_ECAT_SLV_START+i]->type = (int)EcatSlaveCmdType::RESET;

        ecatSlaveCommandsMsg_enabled[IDX_ECAT_SLV_START+i] = 1;

        ros::Duration(0.010).sleep();
    }
    #endif

    thread_ecat.join();
    thread_ecat_check.join();

    return 0;
}
