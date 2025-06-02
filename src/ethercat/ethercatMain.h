#pragma once

#include <ethercat.h>

#include <ecat/EcatMasterState.h>
#include <ecat/EcatSlaveInfos.h>
#include <ecat/EcatState.h>
#include <ecat/EcatSlaveCommands.h>

#include <ecat/EcatControl.h>

#include "EnumUtilities.h"

#define PRINT_DETAIL_LOG_EN 1
inline void PRINT_DETAIL_LOG(uint32_t print, int line_number, const char* func_name = __builtin_FUNCTION()) {
    if (print) {
        printf("[F:%s,L:%d,T:%lf]", func_name, line_number, ros::Time::now().toSec());
    }
}

// enum class EcatDriveMode {
//     NO_ECAT, TORQUE, VELOCITY, PP
// };
MakeEnumEx(EcatDriveMode,
    ((NO_ECAT,))
    ((TORQUE,))
    ((VELOCITY,))
    ((PP,))
    ((PV,))
);

// enum class EcatSlaveCmdType {
//     NONE, INIT, RUN, STOP, RESET, SDO_RD, SDO_WR, PDO
// };
MakeEnumEx(EcatSlaveCmdType,
    ((NONE,=0))
    ((INIT,=1))
    ((RUN,=2))
    ((STOP,=3))
    ((RESET,=4))
    ((HOME,=5))
    ((SDO_RD,=6))
    ((SDO_WR,=7))
    ((PDO,=8))
);

MakeEnumEx(EcatSlaveArch,
    ((NO_ARCH,=0))
    ((ANGULAR_SWIVEL,=1))
    ((ANGULAR_WHEEL_WELCON,=2))
    ((LINEAR_PULLY,=3))
    ((LINEAR_SCREW,=4))
    ((RPM_LS,=5))
    ((RPM_WELCON,=6))
);

#define IDX_ECAT_MASTER 0
#define IDX_ECAT_SLV_START 1

extern ecat::EcatMasterState ecatMasterStateMsg;
extern ecat::EcatSlaveInfos ecatSlaveInfosMsg;
extern ecat::EcatState ecatStateMsg;
extern ecat::EcatSlaveCommands ecatSlaveCommandsMsg;

extern std::vector<ecat::EcatSlaveInfo*> pEcatSlaveInfosMsg;
extern std::vector<ecat::EcatSlaveCommand*> pEcatSlaveCommandsMsg;

extern uint32_t ecatLoopRun;
extern std::vector<uint32_t> ecatSlaveCommandsMsg_callbacked;
extern std::vector<uint32_t> ecatSlaveCommandsMsg_enabled;
extern std::vector<uint32_t> motorInited;
extern ecat::EcatSlaveCommands srv_ecat_control_msg;

void ecatSlaveCommandsCallBack(const ecat::EcatSlaveCommands ecatSlaveCommands);
bool ecatSlaveControlCallback(ecat::EcatControl::Request &req, ecat::EcatControl::Response &res);
void ecat_loop(int ecat_hz);
void ecat_check();
void ecat_init();

#define STATUSWORD_BIT_FAULT 3
#define STATUSWORD_BIT_WARNING 7

#define LS_MECAPION_ENCODER_BATTERY_CODE 0x35
#define ENCODER_BATTERY_LOW LS_MECAPION_ENCODER_BATTERY_CODE

#define HOMING_NOT_SET 0
#define HOMING_DONE 1

#define MODE_OF_OP_INDEX 0x6060
#define MODE_OF_OP_SUB_INDEX 0x00
#define MODE_OF_OP_DISPLAY_INDEX 0x6061
#define MODE_OF_OP_DISPLAY_SUB_INDEX 0x00
#define HOMING_METHOD_INDEX 0x6098
#define HOMING_METHOD_SUB_INDEX 0x00

#define PP_Mode 1
#define PV_Mode 3
#define PT_Mode 4
#define HM_Mode 6
#define CSP_MODE 8
#define CSV_MODE 9
#define CST_MODE 10

#define HOME_ON_CURRENT_POSITION 35

#define CONTROLWORD_SHUTDOWN 0x0006
#define CONTROLWORD_OP_EN 0x000F
#define CONTROLWORD_OP_PP_START 0x003F
#define CONTROLWORD_OP_HOMING 0x001F
#define CONTROLWORD_Q_STOP 0x0002
#define CONTROLWORD_RESET 0x0080