#pragma once

#include <ros/ros.h>
#include <math.h>

#include <ecat/Infos.h>

enum class CommandMode {
    ESTOP = -2, STOP = -1, NO_ACTION = 0, VELOCITY = 1, POSITION = 2, HOMING = 3, INIT = 4, RESET = 5
};

extern ecat::Infos infos;

extern int motor_num;
extern std::vector<double> hom_offset;
extern std::vector<double> inv_motor_in_arr;
extern double qmsg_ts_period;
extern double pub_ts_period;
extern std::string ifname;
extern std::vector<int> ecat_drive_mode;
extern std::vector<int> slave_arch;
extern int freerun;
extern double acc_dec_factor;
extern int32_t sdo_ca_support;

extern std::vector<double> gear_ratio;
extern std::vector<double> motor_tick;

#define MIN_TO_SEC(x) ((x) * (60.0))
#define SEC_TO_MIN(x) ((x) / MIN_TO_SEC(1.0))
#define REV_TO_RAD(x) ((x) * (M_PI*2.0))
#define RAD_TO_REV(x) ((x) / REV_TO_RAD(1.0))
#define VEL_L_TO_VEL_RAD(vel, radius) ((vel) / (radius))
#define VEL_RAD_TO_RPM(vel) (MIN_TO_SEC(RAD_TO_REV(vel)))
#define VEL_RAD_TO_VEL_L(vel, radius) ((vel) * (radius))
#define VEL_RPM_TO_RAD(rpm) (SEC_TO_MIN(REV_TO_RAD(rpm)))