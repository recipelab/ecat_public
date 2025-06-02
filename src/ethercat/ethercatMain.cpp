#include <ros/ros.h>

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include <boost/format.hpp>

#include <ethercat.h>

#include "ethercatMain.h"
#include "main.hpp"

#define ENUM_TO_STR(ENUM) std::string(#ENUM)

#define DRIVER_ALARM_EXIT 0
#define RET_FAIL -1

typedef struct _SdoData {
    uint16_t index;
    uint8_t subindex;
    uint8_t CA;
    int32_t size;
} SdoData;

#pragma pack(push, 1)
typedef struct _TxPdoValue {
    uint16_t statusword;
    int16_t tor_act;
    int32_t vel_act;
    int32_t pos_act;
    uint16_t error_code;
} TxPdoValue;

typedef struct _RxPdoValue {
    uint16_t controlword;
    int16_t tor_tar;
    int32_t vel_tar;
    int32_t pos_tar;
    int32_t acc_tar;
    int32_t dec_tar;
} RxPdoValue;
#pragma pack(pop)

#include <ecat/EcatMasterState.h>
ecat::EcatMasterState ecatMasterStateMsg;
#include <ecat/EcatSlaveInfos.h>
ecat::EcatSlaveInfos ecatSlaveInfosMsg;
#include <ecat/EcatState.h>
ecat::EcatState ecatStateMsg;
#include <ecat/EcatSlaveCommands.h>
ecat::EcatSlaveCommands ecatSlaveCommandsMsg;

std::vector<ecat::EcatSlaveInfo*> pEcatSlaveInfosMsg;
std::vector<ecat::EcatSlaveCommand*> pEcatSlaveCommandsMsg;
std::vector<TxPdoValue> txPdoValuePre;
std::vector<TxPdoValue> txPdoValue;
std::vector<RxPdoValue> rxPdoValue;
std::vector<SdoData> sdoData;
std::vector<EcatSlaveCmdType> ecatSlaveCmdTypePre;
std::vector<EcatSlaveCmdType> ecatSlaveCmdType;
std::vector<uint32_t> subFsm;
std::vector<uint32_t> subFsmPP;
#define MAX_ERROR_COUNT 3
std::vector<uint32_t> error_count;
#define MAX_RETRY_COUNT 3
std::vector<uint32_t> retry_count;
uint32_t ecatLoopRun;
std::vector<uint32_t> ecatSlaveCommandsMsg_enabled;
std::vector<uint32_t> motorInited;
std::vector<uint8_t> drive_mode;
std::vector<uint16_t> wroteControlword;

#include <ecat/EcatSlaveCommands.h>
std::vector<uint32_t> ecatSlaveCommandsMsg_callbacked;
void baseEcatSlaveCommandsCallBack(const ecat::EcatSlaveCommands ecatSlaveCommands) {
    ecatSlaveCommandsMsg = ecatSlaveCommands;
    for (int i=IDX_ECAT_SLV_START; i<(IDX_ECAT_SLV_START+motor_num); i++) {
        ecatSlaveCommandsMsg_callbacked[i] = 1;
    }

    #if 0
    static double time_cur = ros::Time::now().toSec();
    static double time_pre = time_cur;
    static uint32_t count = 0;

    time_cur = ros::Time::now().toSec();

    PRINT_DETAIL_LOG(PRINT_DETAIL_LOG_EN, __LINE__);
    printf("count: %05d, time_diff(ms): %lf\n", count++, time_cur-time_pre);
    time_pre = time_cur;
    #endif
}
void ecatSlaveCommandsCallBack(const ecat::EcatSlaveCommands ecatSlaveCommands) {
    baseEcatSlaveCommandsCallBack(ecatSlaveCommands);
}

ecat::EcatSlaveCommands srv_ecat_control_msg;
bool ecatSlaveControlCallback(ecat::EcatControl::Request &req, ecat::EcatControl::Response &res) {
    static ecat::EcatSlaveCommand* pSlaveReq = &(req.s1);

    for (int i=0; i<srv_ecat_control_msg.slaves.size(); i++) {
        srv_ecat_control_msg.slaves[i] = pSlaveReq[i];
    }

    baseEcatSlaveCommandsCallBack(srv_ecat_control_msg);

#define SRV_SUCCESS	true
    res.success = SRV_SUCCESS;

    return true;
}

#define PRINT_STATUSWORD 0
#define PRINT_CONTROLWORD 0

void printStatusword(uint16_t statusword, bool en_print);
void printControlword(uint16_t controlword, bool en_print);

#define EC_TIMEOUTMON 500

#define SDO_PRINT_MOTOR_DRIVER 0

#define DEFAULT_DELAY_SEC 0.1
#define DEFAULT_RESET_SEC 0.5

#define SETUP_OD_FAILED RET_FAIL
#define SETUP_OD_NONE 0
#define SETUP_OD_SET 1

char IOmap[4096];
OSAL_THREAD_HANDLE thread1;
OSAL_THREAD_HANDLE thread2;
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;
boolean forceByteAlignment = FALSE;

int SDOwritePseudoCA(uint16_t Slave, uint16_t Index, uint16_t* Values, int Timeout, int subindex_length, int array_to_object_ratio) {
    int retval;
    int retchk;
    uint16_t u16val = 0;

    retval = 0;
    retchk = 0;

    #if 0
    printf("index: 0x%04x, subindex_length: %d\n", Index, subindex_length);
    #endif
    retchk++, retval += ec_SDOwrite(Slave, Index, 0x00, FALSE, sizeof(uint16_t), &u16val, EC_TIMEOUTSAFE);
    for (int i=0; i<subindex_length; i++) {
        retchk++, retval += ec_SDOwrite(Slave, Index, i+1, FALSE, sizeof(uint16_t)*array_to_object_ratio, &Values[(i*array_to_object_ratio)+1], EC_TIMEOUTSAFE);
        #if 0
        printf("index: 0x%04x.%02x, write_byte: %dd, value: 0x%04x, \n", Index, i+1, sizeof(uint16_t)*array_to_object_ratio, Values[(i*array_to_object_ratio)+1]);
        #endif
    }
    retchk++, retval += ec_SDOwrite(Slave, Index, 0x00, FALSE, sizeof(uint16_t), &Values[0], EC_TIMEOUTSAFE);

    // ec_SDOwrite()와 비슷하게 작성함
    // 원래의 리턴값은 Workcounter from last slave response임
    if (retchk == retval) {
        return 1;
    } else {
        return 0;
    }
}

int setupOD(uint16 slave) {
    int retval;
    int retchk;
    uint16 u16val;
    int array_to_object_ratio;

    retval = 0;
    retchk = 0;

    // OD(SINT[8bit, 0x0008], INT[16bit, 0x0010], DINT[32bit, 0x0020])
    // 0x6040(UINT) Controlword
    // 0x6041(UINT) Statusword
    // 0x6077(INT) Torque Actual Value [0.1%]
    // 0x606C(DINT) Velocity Actual Value [UU/s]
    // 0x6064(DINT) Position Actual Value [UU]
    // 0x603F(UINT) Error Code

    // PT
    // 0x6071(INT) Target Torque, CST
    // 0x6087(UDINT) Torque Slope[0.1%/s]

    // PV
    // 0x60FF(DINT) Target Velocity, CSV
    // 0x6083(UDINT) Profile Acceleration[UU/s^2]
    // 0x6084(UDINT) Profile Deceleration[UU/s^2]

    // PP
    // 0x607A(DINT) Target Position, CSP
    // 0x6081(UDINT) Profile Velocity[UU/s]
    // 0x6083(UDINT) Profile Acceleration[UU/s^2]
    // 0x6084(UDINT) Profile Deceleration[UU/s^2]

    // PDO mapping
    // 0x1600 1st receive pdo mapping(0x6040 controlword, 0x60ff target velocity, 0x6083 profile acceleration, 0x6084 profile deceleration)
    // 0x1601 2nd receive pdo mapping(0x6040 controlword, 0x6071 target torque)
    // 0x1602 3rd receive pdo mapping(0x6040 controlword, 0x60ff target velocity)
    // 0x1603 4th receive pdo mapping(0x6040 controlword, 0x607A target position, 0x6081 profile velocity, 0x6083 profile acceleration, 0x6084 profile deceleration)
    // 0x1a01 2nd transmit pdo mapping(0x6041 statusword, 0x6077 torque actual value, 0x606c velocity actual value, 0x6064 position actual value, 0x603f error code)
    uint16 map_1600[] = {0x0004, 0x0010, 0x6040, 0x0020, 0x60ff, 0x0020, 0x6083, 0x0020, 0x6084};
    uint16 map_1601[] = {0x0002, 0x0010, 0x6040, 0x0010, 0x6071};
    uint16 map_1602[] = {0x0002, 0x0010, 0x6040, 0x0020, 0x60ff};
    uint16 map_1603[] = {0x0005, 0x0010, 0x6040, 0x0020, 0x607A, 0x0020, 0x6081, 0x0020, 0x6083, 0x0020, 0x6084};
    uint16 map_1a01[] = {0x0005, 0x0010, 0x6041, 0x0010, 0x6077, 0x0020, 0x606c, 0x0020, 0x6064, 0x0010, 0x603f};
    if (sdo_ca_support) {
        retchk++, retval += ec_SDOwrite(slave, 0x1600, 0x00, TRUE, sizeof(map_1600), &map_1600, EC_TIMEOUTSAFE);
        retchk++, retval += ec_SDOwrite(slave, 0x1601, 0x00, TRUE, sizeof(map_1601), &map_1601, EC_TIMEOUTSAFE);
        retchk++, retval += ec_SDOwrite(slave, 0x1602, 0x00, TRUE, sizeof(map_1602), &map_1602, EC_TIMEOUTSAFE);
        retchk++, retval += ec_SDOwrite(slave, 0x1603, 0x00, TRUE, sizeof(map_1603), &map_1603, EC_TIMEOUTSAFE);
        retchk++, retval += ec_SDOwrite(slave, 0x1a01, 0x00, TRUE, sizeof(map_1a01), &map_1a01, EC_TIMEOUTSAFE);
    } else {
        // 아래의 사항은 welcon사의 모터드라이가 CA를 지원하지 않으므로 인해서 작성하였음
        // Software Version 00.2076 -> 00.2080 으로 업데이트하면서 CA를 지원함
        // 배열의 sizeof를 사용하기 위해서는 변수가 생성된 곳에서 변수의 이름만 넣어서 사용가능
        // ex) uint16 map_1600[] = {0x0004, 0x0010, 0x6040, 0x0020, 0x60ff, 0x0020, 0x6083, 0x0020, 0x6084};
        // ex) sizeof(map_1600) : 18 <-- 실제 바이트
        // ex) sizeof(map_1600 + 1) : 8 <-- 포인터의 크기
        // int SDOwritePseudoCA(uint16_t Slave, uint16_t Index, uint16_t* Values, int Timeout, int subindex_length, int array_to_object_ratio) {
        // subindex_length : 0번 서브인덱스를 제외한 서브인덱스의 개수
        // subindex_length 계산식
        // ex) (sizeof(map_1600)/sizeof(uint16)-1)/2 == (18byte/2byte-1)/2
        // ex) (18byte/2byte) == 9 : 배열 요소의 개수(첫번째 요소를 제외한 나머지가 오브젝트 데이터)
        // ex) -1 : 첫번째 요소 제외
        // ex) 2 : 오브젝트는 배열 요소 2개가 합쳐져서 1개가 됨, array_to_object_ratio
        // array_to_object_ratio : 오브젝트는 배열 요소 2개가 합쳐져서 1개가 됨
        array_to_object_ratio = 2;
        retchk++, retval += SDOwritePseudoCA(slave, 0x1600, map_1600, EC_TIMEOUTSAFE, (sizeof(map_1600)/sizeof(uint16)-1)/array_to_object_ratio, array_to_object_ratio);
        retchk++, retval += SDOwritePseudoCA(slave, 0x1601, map_1601, EC_TIMEOUTSAFE, (sizeof(map_1601)/sizeof(uint16)-1)/array_to_object_ratio, array_to_object_ratio);
        retchk++, retval += SDOwritePseudoCA(slave, 0x1602, map_1602, EC_TIMEOUTSAFE, (sizeof(map_1602)/sizeof(uint16)-1)/array_to_object_ratio, array_to_object_ratio);
        retchk++, retval += SDOwritePseudoCA(slave, 0x1603, map_1603, EC_TIMEOUTSAFE, (sizeof(map_1603)/sizeof(uint16)-1)/array_to_object_ratio, array_to_object_ratio);
        retchk++, retval += SDOwritePseudoCA(slave, 0x1a01, map_1a01, EC_TIMEOUTSAFE, (sizeof(map_1a01)/sizeof(uint16)-1)/array_to_object_ratio, array_to_object_ratio);
    }

    // PDO mapping
    // 0x1c12 sync manager 2 pdo assignment(0x1601 2nd receive pdo mapping)
    // 0x1c13 sync manager 3 pdo assignment(0x1a01 2nd transmit pdo mapping)
    #define SDO_INDEX_IDX 1
    uint16 map_1c12[] = {0x0001, 0x0000};
    switch (ecat_drive_mode[slave-IDX_ECAT_SLV_START]) {
        case (int)EcatDriveMode::NO_ECAT:
            break;
        case (int)EcatDriveMode::TORQUE:
            map_1c12[SDO_INDEX_IDX] = 0x1601;
            break;
        case (int)EcatDriveMode::VELOCITY:
            map_1c12[SDO_INDEX_IDX] = 0x1602;
            break;
        case (int)EcatDriveMode::PP:
            map_1c12[SDO_INDEX_IDX] = 0x1603;
            break;
        case (int)EcatDriveMode::PV:
            map_1c12[SDO_INDEX_IDX] = 0x1600;
            break;
        default:
            PRINT_DETAIL_LOG(PRINT_DETAIL_LOG_EN, __LINE__);
            ROS_ERROR("unknown case[ecat_drive_mode[%d](%dd)]", slave-IDX_ECAT_SLV_START, ecat_drive_mode[slave-IDX_ECAT_SLV_START]);
            break;
    }
    uint16 map_1c13[] = {0x0001, 0x1a01};
    if (sdo_ca_support) {
        retchk++, retval += ec_SDOwrite(slave, 0x1c12, 0x00, TRUE, sizeof(map_1c12), &map_1c12, EC_TIMEOUTSAFE);
        retchk++, retval += ec_SDOwrite(slave, 0x1c13, 0x00, TRUE, sizeof(map_1c13), &map_1c13, EC_TIMEOUTSAFE);
    } else {
        array_to_object_ratio = 1;
        retchk++, retval += SDOwritePseudoCA(slave, 0x1600, map_1c12, EC_TIMEOUTSAFE, (sizeof(map_1c12)/sizeof(uint16)-1)/array_to_object_ratio, array_to_object_ratio);
        retchk++, retval += SDOwritePseudoCA(slave, 0x1601, map_1c13, EC_TIMEOUTSAFE, (sizeof(map_1c13)/sizeof(uint16)-1)/array_to_object_ratio, array_to_object_ratio);
    }

    if (retchk == 0) {
        printf("slave[%02d]'s setupOD is NONE\n", slave);

        return SETUP_OD_NONE;
    } else if (retchk != retval) {
        ROS_ERROR("slave[%02d]'s setupOD is failed !!!", slave);

        #if DRIVER_ALARM_EXIT
        exit(SETUP_OD_FAILED);
        #else
        return SETUP_OD_FAILED;
        #endif
    } else {
    }

    printf("slave[%02d]'s setupOD(%d ea) is set\n", slave, retval);

    return retval;
}

int setupOdPp(uint16 slave) {
    int retval;
    int retchk;
    uint16 u16val;
    int array_to_object_ratio;

    retval = 0;
    retchk = 0;

    // OD(SINT[8bit, 0x0008], INT[16bit, 0x0010], DINT[32bit, 0x0020])
    // 0x6040(UINT) Controlword
    // 0x6041(UINT) Statusword
    // 0x6077(INT) Torque Actual Value [0.1%]
    // 0x606C(DINT) Velocity Actual Value [UU/s]
    // 0x6064(DINT) Position Actual Value [UU]
    // 0x603F(UINT) Error Code

    // PT
    // 0x6071(INT) Target Torque, CST
    // 0x6087(UDINT) Torque Slope[0.1%/s]

    // PV
    // 0x60FF(DINT) Target Velocity, CSV
    // 0x6083(UDINT) Profile Acceleration[UU/s^2]
    // 0x6084(UDINT) Profile Deceleration[UU/s^2]

    // PP
    // 0x607A(DINT) Target Position, CSP
    // 0x6081(UDINT) Profile Velocity[UU/s]
    // 0x6083(UDINT) Profile Acceleration[UU/s^2]
    // 0x6084(UDINT) Profile Deceleration[UU/s^2]

    // PDO mapping
    // 0x1600 1st receive pdo mapping(0x6040 controlword, 0x60ff target velocity, 0x6083 profile acceleration, 0x6084 profile deceleration)
    // 0x1601 2nd receive pdo mapping(0x6040 controlword, 0x6071 target torque)
    // 0x1602 3rd receive pdo mapping(0x6040 controlword, 0x60ff target velocity)
    // 0x1603 4th receive pdo mapping(0x6040 controlword, 0x607A target position, 0x6081 profile velocity, 0x6083 profile acceleration, 0x6084 profile deceleration)
    // 0x1a01 2nd transmit pdo mapping(0x6041 statusword, 0x6077 torque actual value, 0x606c velocity actual value, 0x6064 position actual value, 0x603f error code)
    uint16 map_1600[] = {0x0004, 0x0010, 0x6040, 0x0020, 0x60ff, 0x0020, 0x6083, 0x0020, 0x6084};
    uint16 map_1601[] = {0x0002, 0x0010, 0x6040, 0x0010, 0x6071};
    uint16 map_1602[] = {0x0002, 0x0010, 0x6040, 0x0020, 0x60ff};
    uint16 map_1603[] = {0x0005, 0x0010, 0x6040, 0x0020, 0x607A, 0x0020, 0x6081, 0x0020, 0x6083, 0x0020, 0x6084};
    uint16 map_1a01[] = {0x0005, 0x0010, 0x6041, 0x0010, 0x6077, 0x0020, 0x606c, 0x0020, 0x6064, 0x0010, 0x603f};
    if (sdo_ca_support) {
        retchk++, retval += ec_SDOwrite(slave, 0x1600, 0x00, TRUE, sizeof(map_1600), &map_1600, EC_TIMEOUTSAFE);
        retchk++, retval += ec_SDOwrite(slave, 0x1601, 0x00, TRUE, sizeof(map_1601), &map_1601, EC_TIMEOUTSAFE);
        retchk++, retval += ec_SDOwrite(slave, 0x1602, 0x00, TRUE, sizeof(map_1602), &map_1602, EC_TIMEOUTSAFE);
        retchk++, retval += ec_SDOwrite(slave, 0x1603, 0x00, TRUE, sizeof(map_1603), &map_1603, EC_TIMEOUTSAFE);
        retchk++, retval += ec_SDOwrite(slave, 0x1a01, 0x00, TRUE, sizeof(map_1a01), &map_1a01, EC_TIMEOUTSAFE);
    } else {
        // 아래의 사항은 welcon사의 모터드라이가 CA를 지원하지 않으므로 인해서 작성하였음
        // Software Version 00.2076 -> 00.2080 으로 업데이트하면서 CA를 지원함
        // 배열의 sizeof를 사용하기 위해서는 변수가 생성된 곳에서 변수의 이름만 넣어서 사용가능
        // ex) uint16 map_1600[] = {0x0004, 0x0010, 0x6040, 0x0020, 0x60ff, 0x0020, 0x6083, 0x0020, 0x6084};
        // ex) sizeof(map_1600) : 18 <-- 실제 바이트
        // ex) sizeof(map_1600 + 1) : 8 <-- 포인터의 크기
        // int SDOwritePseudoCA(uint16_t Slave, uint16_t Index, uint16_t* Values, int Timeout, int subindex_length, int array_to_object_ratio) {
        // subindex_length : 0번 서브인덱스를 제외한 서브인덱스의 개수
        // subindex_length 계산식
        // ex) (sizeof(map_1600)/sizeof(uint16)-1)/2 == (18byte/2byte-1)/2
        // ex) (18byte/2byte) == 9 : 배열 요소의 개수(첫번째 요소를 제외한 나머지가 오브젝트 데이터)
        // ex) -1 : 첫번째 요소 제외
        // ex) 2 : 오브젝트는 배열 요소 2개가 합쳐져서 1개가 됨, array_to_object_ratio
        // array_to_object_ratio : 오브젝트는 배열 요소 2개가 합쳐져서 1개가 됨
        array_to_object_ratio = 2;
        retchk++, retval += SDOwritePseudoCA(slave, 0x1600, map_1600, EC_TIMEOUTSAFE, (sizeof(map_1600)/sizeof(uint16)-1)/array_to_object_ratio, array_to_object_ratio);
        retchk++, retval += SDOwritePseudoCA(slave, 0x1601, map_1601, EC_TIMEOUTSAFE, (sizeof(map_1601)/sizeof(uint16)-1)/array_to_object_ratio, array_to_object_ratio);
        retchk++, retval += SDOwritePseudoCA(slave, 0x1602, map_1602, EC_TIMEOUTSAFE, (sizeof(map_1602)/sizeof(uint16)-1)/array_to_object_ratio, array_to_object_ratio);
        retchk++, retval += SDOwritePseudoCA(slave, 0x1603, map_1603, EC_TIMEOUTSAFE, (sizeof(map_1603)/sizeof(uint16)-1)/array_to_object_ratio, array_to_object_ratio);
        retchk++, retval += SDOwritePseudoCA(slave, 0x1a01, map_1a01, EC_TIMEOUTSAFE, (sizeof(map_1a01)/sizeof(uint16)-1)/array_to_object_ratio, array_to_object_ratio);
    }

    // PDO mapping
    // 0x1c12 sync manager 2 pdo assignment(0x1601 2nd receive pdo mapping)
    // 0x1c13 sync manager 3 pdo assignment(0x1a01 2nd transmit pdo mapping)
    #define SDO_INDEX_IDX 1
    uint16 map_1c12[] = {0x0001, 0x0000};
    switch (ecat_drive_mode[slave-IDX_ECAT_SLV_START]) {
        case (int)EcatDriveMode::NO_ECAT:
            break;
        case (int)EcatDriveMode::TORQUE:
            map_1c12[SDO_INDEX_IDX] = 0x1601;
            break;
        case (int)EcatDriveMode::VELOCITY:
            map_1c12[SDO_INDEX_IDX] = 0x1602;
            break;
        case (int)EcatDriveMode::PP:
            map_1c12[SDO_INDEX_IDX] = 0x1603;
            break;
        case (int)EcatDriveMode::PV:
            map_1c12[SDO_INDEX_IDX] = 0x1600;
            break;
        default:
            PRINT_DETAIL_LOG(PRINT_DETAIL_LOG_EN, __LINE__);
            ROS_ERROR("unknown case[ecat_drive_mode[%d](%dd)]", slave-IDX_ECAT_SLV_START, ecat_drive_mode[slave-IDX_ECAT_SLV_START]);
            break;
    }
    uint16 map_1c13[] = {0x0001, 0x1a01};
    if (sdo_ca_support) {
        retchk++, retval += ec_SDOwrite(slave, 0x1c12, 0x00, TRUE, sizeof(map_1c12), &map_1c12, EC_TIMEOUTSAFE);
        retchk++, retval += ec_SDOwrite(slave, 0x1c13, 0x00, TRUE, sizeof(map_1c13), &map_1c13, EC_TIMEOUTSAFE);
    } else {
        array_to_object_ratio = 1;
        retchk++, retval += SDOwritePseudoCA(slave, 0x1600, map_1c12, EC_TIMEOUTSAFE, (sizeof(map_1c12)/sizeof(uint16)-1)/array_to_object_ratio, array_to_object_ratio);
        retchk++, retval += SDOwritePseudoCA(slave, 0x1601, map_1c13, EC_TIMEOUTSAFE, (sizeof(map_1c13)/sizeof(uint16)-1)/array_to_object_ratio, array_to_object_ratio);
    }

    if (retchk == 0) {
        printf("slave[%02d]'s setupOdPp is NONE\n", slave);

        return SETUP_OD_NONE;
    } else if (retchk != retval) {
        ROS_ERROR("slave[%02d]'s setupOdPp is failed !!!", slave);

        #if DRIVER_ALARM_EXIT
        exit(SETUP_OD_FAILED);
        #else
        return SETUP_OD_FAILED;
        #endif
    } else {
    }

    printf("slave[%02d]'s setupOdPp(%d ea) is set\n", slave, retval);

    return retval;
}

int setupOdCsv(uint16 slave) {
    int retval;
    int retchk;
    uint16 u16val;
    int array_to_object_ratio;

    retval = 0;
    retchk = 0;

    // OD(SINT[8bit, 0x0008], INT[16bit, 0x0010], DINT[32bit, 0x0020])
    // 0x6040(UINT) Controlword
    // 0x6041(UINT) Statusword
    // 0x6077(INT) Torque Actual Value [0.1%]
    // 0x606C(DINT) Velocity Actual Value [UU/s]
    // 0x6064(DINT) Position Actual Value [UU]
    // 0x603F(UINT) Error Code

    // PT
    // 0x6071(INT) Target Torque, CST
    // 0x6087(UDINT) Torque Slope[0.1%/s]

    // PV
    // 0x60FF(DINT) Target Velocity, CSV
    // 0x6083(UDINT) Profile Acceleration[UU/s^2]
    // 0x6084(UDINT) Profile Deceleration[UU/s^2]

    // PP
    // 0x607A(DINT) Target Position, CSP
    // 0x6081(UDINT) Profile Velocity[UU/s]
    // 0x6083(UDINT) Profile Acceleration[UU/s^2]
    // 0x6084(UDINT) Profile Deceleration[UU/s^2]

    // PDO mapping
    // 0x1600 1st receive pdo mapping(0x6040 controlword, 0x60ff target velocity, 0x6083 profile acceleration, 0x6084 profile deceleration)
    // 0x1601 2nd receive pdo mapping(0x6040 controlword, 0x6071 target torque)
    // 0x1602 3rd receive pdo mapping(0x6040 controlword, 0x60ff target velocity)
    // 0x1603 4th receive pdo mapping(0x6040 controlword, 0x607A target position, 0x6081 profile velocity, 0x6083 profile acceleration, 0x6084 profile deceleration)
    // 0x1a01 2nd transmit pdo mapping(0x6041 statusword, 0x6077 torque actual value, 0x606c velocity actual value, 0x6064 position actual value, 0x603f error code)
    uint16 map_1600[] = {0x0004, 0x0010, 0x6040, 0x0020, 0x60ff, 0x0020, 0x6083, 0x0020, 0x6084};
    uint16 map_1601[] = {0x0002, 0x0010, 0x6040, 0x0010, 0x6071};
    uint16 map_1602[] = {0x0002, 0x0010, 0x6040, 0x0020, 0x60ff};
    uint16 map_1603[] = {0x0005, 0x0010, 0x6040, 0x0020, 0x607A, 0x0020, 0x6081, 0x0020, 0x6083, 0x0020, 0x6084};
    uint16 map_1a01[] = {0x0005, 0x0010, 0x6041, 0x0010, 0x6077, 0x0020, 0x606c, 0x0020, 0x6064, 0x0010, 0x603f};
    if (sdo_ca_support) {
        retchk++, retval += ec_SDOwrite(slave, 0x1600, 0x00, TRUE, sizeof(map_1600), &map_1600, EC_TIMEOUTSAFE);
        retchk++, retval += ec_SDOwrite(slave, 0x1601, 0x00, TRUE, sizeof(map_1601), &map_1601, EC_TIMEOUTSAFE);
        retchk++, retval += ec_SDOwrite(slave, 0x1602, 0x00, TRUE, sizeof(map_1602), &map_1602, EC_TIMEOUTSAFE);
        retchk++, retval += ec_SDOwrite(slave, 0x1603, 0x00, TRUE, sizeof(map_1603), &map_1603, EC_TIMEOUTSAFE);
        retchk++, retval += ec_SDOwrite(slave, 0x1a01, 0x00, TRUE, sizeof(map_1a01), &map_1a01, EC_TIMEOUTSAFE);
    } else {
        // 아래의 사항은 welcon사의 모터드라이가 CA를 지원하지 않으므로 인해서 작성하였음
        // Software Version 00.2076 -> 00.2080 으로 업데이트하면서 CA를 지원함
        // 배열의 sizeof를 사용하기 위해서는 변수가 생성된 곳에서 변수의 이름만 넣어서 사용가능
        // ex) uint16 map_1600[] = {0x0004, 0x0010, 0x6040, 0x0020, 0x60ff, 0x0020, 0x6083, 0x0020, 0x6084};
        // ex) sizeof(map_1600) : 18 <-- 실제 바이트
        // ex) sizeof(map_1600 + 1) : 8 <-- 포인터의 크기
        // int SDOwritePseudoCA(uint16_t Slave, uint16_t Index, uint16_t* Values, int Timeout, int subindex_length, int array_to_object_ratio) {
        // subindex_length : 0번 서브인덱스를 제외한 서브인덱스의 개수
        // subindex_length 계산식
        // ex) (sizeof(map_1600)/sizeof(uint16)-1)/2 == (18byte/2byte-1)/2
        // ex) (18byte/2byte) == 9 : 배열 요소의 개수(첫번째 요소를 제외한 나머지가 오브젝트 데이터)
        // ex) -1 : 첫번째 요소 제외
        // ex) 2 : 오브젝트는 배열 요소 2개가 합쳐져서 1개가 됨, array_to_object_ratio
        // array_to_object_ratio : 오브젝트는 배열 요소 2개가 합쳐져서 1개가 됨
        array_to_object_ratio = 2;
        retchk++, retval += SDOwritePseudoCA(slave, 0x1600, map_1600, EC_TIMEOUTSAFE, (sizeof(map_1600)/sizeof(uint16)-1)/array_to_object_ratio, array_to_object_ratio);
        retchk++, retval += SDOwritePseudoCA(slave, 0x1601, map_1601, EC_TIMEOUTSAFE, (sizeof(map_1601)/sizeof(uint16)-1)/array_to_object_ratio, array_to_object_ratio);
        retchk++, retval += SDOwritePseudoCA(slave, 0x1602, map_1602, EC_TIMEOUTSAFE, (sizeof(map_1602)/sizeof(uint16)-1)/array_to_object_ratio, array_to_object_ratio);
        retchk++, retval += SDOwritePseudoCA(slave, 0x1603, map_1603, EC_TIMEOUTSAFE, (sizeof(map_1603)/sizeof(uint16)-1)/array_to_object_ratio, array_to_object_ratio);
        retchk++, retval += SDOwritePseudoCA(slave, 0x1a01, map_1a01, EC_TIMEOUTSAFE, (sizeof(map_1a01)/sizeof(uint16)-1)/array_to_object_ratio, array_to_object_ratio);
    }

    // PDO mapping
    // 0x1c12 sync manager 2 pdo assignment(0x1601 2nd receive pdo mapping)
    // 0x1c13 sync manager 3 pdo assignment(0x1a01 2nd transmit pdo mapping)
    #define SDO_INDEX_IDX 1
    uint16 map_1c12[] = {0x0001, 0x0000};
    switch (ecat_drive_mode[slave-IDX_ECAT_SLV_START]) {
        case (int)EcatDriveMode::NO_ECAT:
            break;
        case (int)EcatDriveMode::TORQUE:
            map_1c12[SDO_INDEX_IDX] = 0x1601;
            break;
        case (int)EcatDriveMode::VELOCITY:
            map_1c12[SDO_INDEX_IDX] = 0x1602;
            break;
        case (int)EcatDriveMode::PP:
            map_1c12[SDO_INDEX_IDX] = 0x1603;
            break;
        case (int)EcatDriveMode::PV:
            map_1c12[SDO_INDEX_IDX] = 0x1600;
            break;
        default:
            PRINT_DETAIL_LOG(PRINT_DETAIL_LOG_EN, __LINE__);
            ROS_ERROR("unknown case[ecat_drive_mode[%d](%dd)]", slave-IDX_ECAT_SLV_START, ecat_drive_mode[slave-IDX_ECAT_SLV_START]);
            break;
    }
    uint16 map_1c13[] = {0x0001, 0x1a01};
    if (sdo_ca_support) {
        retchk++, retval += ec_SDOwrite(slave, 0x1c12, 0x00, TRUE, sizeof(map_1c12), &map_1c12, EC_TIMEOUTSAFE);
        retchk++, retval += ec_SDOwrite(slave, 0x1c13, 0x00, TRUE, sizeof(map_1c13), &map_1c13, EC_TIMEOUTSAFE);
    } else {
        array_to_object_ratio = 1;
        retchk++, retval += SDOwritePseudoCA(slave, 0x1c12, map_1c12, EC_TIMEOUTSAFE, (sizeof(map_1c12)/sizeof(uint16)-1)/array_to_object_ratio, array_to_object_ratio);
        retchk++, retval += SDOwritePseudoCA(slave, 0x1c13, map_1c13, EC_TIMEOUTSAFE, (sizeof(map_1c13)/sizeof(uint16)-1)/array_to_object_ratio, array_to_object_ratio);
    }

    if (retchk == 0) {
        printf("slave[%02d]'s setupOdCsv is NONE\n", slave);

        return SETUP_OD_NONE;
    } else if (retchk != retval) {
        ROS_ERROR("slave[%02d]'s setupOdCsv is failed !!!", slave);

        #if DRIVER_ALARM_EXIT
        exit(SETUP_OD_FAILED);
        #else
        return SETUP_OD_FAILED;
        #endif
    } else {
    }

    printf("slave[%02d]'s setupOdCsv(%d ea) is set\n", slave, retval);

    return retval;
}

void ecat_loop(int ecat_hz) {
    int i, oloop, iloop, chk;

    #define BIT32 4
    static uint8_t usdo[BIT32] = {0, };
    int psize = sizeof(usdo);
    needlf = FALSE;
    inOP = FALSE;
    int32_t error = 0;
    uint32_t value = 0;
    int32_t rpm_to_uus = 0;
    int32_t acc_to_uus2 = 0;
    int32_t dec_to_uus2 = 0;
    int32_t uus_to_rpm = 0;
    int32_t position = 0;
    int32_t acc = 0;
    int32_t dec = 0;

    if (ec_init(ifname.c_str())) {
        printf("ec_init on %s succeeded.\n", ifname.c_str());

        if (ec_config_init(FALSE) > 0) {
            printf("%d slaves found and configured.\n", ec_slavecount);
            ecatMasterStateMsg.ec_slavecount = ec_slavecount;

            for (int i=IDX_ECAT_SLV_START; i<(IDX_ECAT_SLV_START+motor_num); i++) {
                #if 0
                ec_slave[i].PO2SOconfig = setupOD;
                #else
                switch (ecat_drive_mode[i-IDX_ECAT_SLV_START]) {
                    case (int)EcatDriveMode::VELOCITY:
                        ec_slave[i].PO2SOconfig = setupOdCsv;
                        break;
                    case (int)EcatDriveMode::PP:
                        ec_slave[i].PO2SOconfig = setupOdPp;
                        break;
                    default:
                        break;
                }
                #endif
            }

            if (forceByteAlignment) {
                #if 0
                ec_config_map_aligned(&IOmap);
                #else
                ROS_ERROR("forceByteAlignment not supported.");
                #endif
            } else {
                ec_config_map(&IOmap);
            }

            ec_configdc();

            #if SDO_PRINT_MOTOR_DRIVER
            printf("SDO List\n");
            for (int i=IDX_ECAT_SLV_START; i<(IDX_ECAT_SLV_START+motor_num); i++) {
                sdo_list(i);
            }
            #endif

            printf("Slaves mapped, state to SAFE_OP.\n");

            ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);

            oloop = ec_slave[0].Obytes;
            if ((oloop == 0) && (ec_slave[0].Obits > 0)) {
                oloop = 1;
            }
            if (oloop > 8) {
                oloop = 8;
            }
            iloop = ec_slave[0].Ibytes;
            if ((iloop == 0) && (ec_slave[0].Ibits > 0)) {
                iloop = 1;
            }
            if (iloop > 8) {
                iloop = 8;
            }

            for (int slave=0; slave<=ec_slavecount; slave++) {
                switch (slave) {
                    case IDX_ECAT_MASTER:
                        ecatMasterStateMsg.slave.master.state = ec_slave[slave].state;
                        ecatMasterStateMsg.slave.master.islost = ec_slave[slave].islost;
                        ecatMasterStateMsg.slave.master.ibytes = ec_slave[slave].Ibytes;
                        ecatMasterStateMsg.slave.master.obytes = ec_slave[slave].Obytes;
                        break;
                    default:
                        ecatMasterStateMsg.slave.slaves[slave-IDX_ECAT_SLV_START].state = ec_slave[slave].state;
                        ecatMasterStateMsg.slave.slaves[slave-IDX_ECAT_SLV_START].islost = ec_slave[slave].islost;
                        ecatMasterStateMsg.slave.slaves[slave-IDX_ECAT_SLV_START].ibytes = ec_slave[slave].Ibytes;
                        ecatMasterStateMsg.slave.slaves[slave-IDX_ECAT_SLV_START].obytes = ec_slave[slave].Obytes;
                        break;
                }
            }

            printf("segments : %d : %d %d %d %d\n", ec_group[0].nsegments, ec_group[0].IOsegment[0], ec_group[0].IOsegment[1], ec_group[0].IOsegment[2], ec_group[0].IOsegment[3]);

            printf("Request operational state for all slaves\n");
            expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            printf("Calculated workcounter %d\n", expectedWKC);
            ec_slave[0].state = EC_STATE_OPERATIONAL;

            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);

            ec_writestate(0);
            chk = 200;

            do {
                ec_send_processdata();
                ec_receive_processdata(EC_TIMEOUTRET);
                ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
            } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

            if (ec_slave[0].state == EC_STATE_OPERATIONAL ) {
                printf("Operational state reached for all slaves.\n");
                inOP = TRUE;

                double delay_ts_now = ros::Time::now().toSec();
                double delay_ts_pre = delay_ts_now;
                double delay_ts_sec = DEFAULT_DELAY_SEC;
                // wait for linux to sync on DC
                usleep(100000);
                #define SYNC0TIME 250000
                for (int i=IDX_ECAT_SLV_START; i<(IDX_ECAT_SLV_START+motor_num); i++) {
                    ec_dcsync0(i, TRUE, SYNC0TIME, 0);
                }
                while (ros::ok()) {
                    ec_send_processdata();
                    wkc = ec_receive_processdata(EC_TIMEOUTRET);

                    if (wkc >= expectedWKC) {
                        #if 0
                        printf("T:%" PRId64", i:%d/%d\n", ec_DCtime, i, MAX_COUNT);
                        #endif

                        #if 0
                        memcpy(&txPdoValue, (TxPdoValue*)ec_slave[IDX_ECAT_SLV_START].inputs, sizeof(TxPdoValue));
                        #else
                        for (int i=IDX_ECAT_SLV_START; i<(IDX_ECAT_SLV_START+motor_num); i++) {
                            #if 0
                            wroteControlword[i] = *((uint16_t*)(ec_slave[i].outputs));
                            txPdoValuePre[i] = txPdoValue[i];
                            memcpy(&txPdoValue[i], (TxPdoValue*)ec_slave[i].inputs, sizeof(TxPdoValue));
                            #else
                            switch (ecat_drive_mode[i-IDX_ECAT_SLV_START]) {
                                case (int)EcatDriveMode::NO_ECAT:
                                    break;
                                case (int)EcatDriveMode::TORQUE:
                                case (int)EcatDriveMode::VELOCITY:
                                case (int)EcatDriveMode::PP:
                                    wroteControlword[i] = *((uint16_t*)(ec_slave[i].outputs));
                                    txPdoValuePre[i] = txPdoValue[i];
                                    memcpy(&txPdoValue[i], (TxPdoValue*)ec_slave[i].inputs, sizeof(TxPdoValue));
                                    break;
                                case (int)EcatDriveMode::PV:
                                    wroteControlword[i] = *((uint16_t*)(ec_slave[i].outputs));
                                    txPdoValuePre[i] = txPdoValue[i];
                                    memcpy(&txPdoValue[i], (TxPdoValue*)ec_slave[i].inputs, sizeof(TxPdoValue));
                                    break;
                                default:
                                    PRINT_DETAIL_LOG(PRINT_DETAIL_LOG_EN, __LINE__);
                                    ROS_ERROR("unknown case[ecat_drive_mode[%d](%dd), node(%dd)]", i-IDX_ECAT_SLV_START, ecat_drive_mode[i-IDX_ECAT_SLV_START], i);
                                    break;
                            }
                            #endif
                        }
                        #endif

                        for (int i=IDX_ECAT_SLV_START; i<txPdoValue.size(); i++) {
                            #if 0
                            PRINT_DETAIL_LOG(PRINT_DETAIL_LOG_EN, __LINE__);
                            printf("[SLV_ID:%2d]PDO(READ): {swrd:0x%04X}, {ator:%5dd}, {avel:%5dd}, {apos:%10dd}, {err:0x%04X}\n", i,
                                txPdoValue[i].statusword, txPdoValue[i].tor_act, txPdoValue[i].vel_act, txPdoValue[i].pos_act, txPdoValue[i].error_code);
                            #else
                            pEcatSlaveInfosMsg[i]->txPdo.ts = ros::Time::now().toSec();
                            pEcatSlaveInfosMsg[i]->txPdo.statusword = txPdoValue[i].statusword;
                            pEcatSlaveInfosMsg[i]->txPdo.tor_act = txPdoValue[i].tor_act;
                            pEcatSlaveInfosMsg[i]->txPdo.vel_act = txPdoValue[i].vel_act;
                            pEcatSlaveInfosMsg[i]->txPdo.pos_act = txPdoValue[i].pos_act;
                            pEcatSlaveInfosMsg[i]->txPdo.error_code = txPdoValue[i].error_code;

                            switch (slave_arch[i-IDX_ECAT_SLV_START]) {
                                case (int)EcatSlaveArch::ANGULAR_SWIVEL:
                                case (int)EcatSlaveArch::LINEAR_PULLY:
                                case (int)EcatSlaveArch::LINEAR_SCREW:
                                case (int)EcatSlaveArch::RPM_LS:
                                    // ls unit : UU
                                    uus_to_rpm = (int32_t)(((double)txPdoValue[i].vel_act / motor_tick[i-IDX_ECAT_SLV_START]) * 60.0);
                                    pEcatSlaveInfosMsg[i]->info.rpm = uus_to_rpm;
                                    break;
                                case (int)EcatSlaveArch::ANGULAR_WHEEL_WELCON:
                                case (int)EcatSlaveArch::RPM_WELCON:
                                    // welcon unit : rpm
                                    pEcatSlaveInfosMsg[i]->info.rpm = txPdoValue[i].vel_act;
                                    break;
                                default:
                                    break;
                            }
                            pEcatSlaveInfosMsg[i]->info.position = txPdoValue[i].pos_act;
                            #endif
                            printStatusword(txPdoValue[i].statusword, PRINT_STATUSWORD);
                        }

                        for (int i=IDX_ECAT_SLV_START; i<(IDX_ECAT_SLV_START+motor_num); i++) {
                            if (ecatSlaveCommandsMsg_callbacked[i] || ecatSlaveCommandsMsg_enabled[i]) {
                                ecatSlaveCommandsMsg_callbacked[i] = 0;

                                #if 1
                                if ((EcatSlaveCmdType)pEcatSlaveCommandsMsg[i]->type == EcatSlaveCmdType::NONE) {
                                    break;
                                }
                                #endif

                                ecatSlaveCmdType[i] = (EcatSlaveCmdType)pEcatSlaveCommandsMsg[i]->type;
                                if (ecatSlaveCmdTypePre[i] != ecatSlaveCmdType[i]) {
                                    ecatSlaveCmdTypePre[i] = ecatSlaveCmdType[i];
                                    subFsm[i] = 0;
                                    subFsmPP[i] = 0;
                                }
                                switch (ecatSlaveCmdType[i]) {
                                    case EcatSlaveCmdType::NONE:
                                        switch (subFsm[i]) {
                                            case 0:
                                                PRINT_DETAIL_LOG(PRINT_DETAIL_LOG_EN, __LINE__);
                                                printf("case[EcatSlaveCmdType::NONE], node(%dd)\n", i);
                                                subFsm[i] = 1;
                                                break;
                                            case 1:
                                                break;
                                            default:
                                                PRINT_DETAIL_LOG(PRINT_DETAIL_LOG_EN, __LINE__);
                                                printf("case[subFsm(%dd)], node(%dd)\n", subFsm[i], i);
                                                break;
                                        }
                                        break;
                                    case EcatSlaveCmdType::INIT:
                                        switch (subFsm[i]) {
                                            case -4:
                                                break;
                                            case -3:
                                                PRINT_DETAIL_LOG(PRINT_DETAIL_LOG_EN, __LINE__);
                                                printf("case[EcatSlaveCmdType::INIT]: MAX_RETRY_COUNT[node(%dd)]\n", i);
                                                subFsm[i] = -4;
                                                break;
                                            case -2:
                                                break;
                                            case -1:
                                                PRINT_DETAIL_LOG(PRINT_DETAIL_LOG_EN, __LINE__);
                                                printf("case[EcatSlaveCmdType::INIT]: MAX_ERROR_COUNT[node(%dd)]\n", i);
                                                subFsm[i] = -2;
                                                break;
                                            case 0:
                                                motorInited[i] = 0;
                                                error_count[i] = 0;
                                                retry_count[i] = 0;
                                                subFsm[i] = 1;
                                                switch (ecat_drive_mode[i-IDX_ECAT_SLV_START]) {
                                                    case (int)EcatDriveMode::NO_ECAT:
                                                        ecatSlaveCommandsMsg_enabled[i] = 0;
                                                        motorInited[i] = 1;
                                                        subFsm[i] = 5;
                                                        break;
                                                    case (int)EcatDriveMode::TORQUE:
                                                        drive_mode[i] = CST_MODE;
                                                        break;
                                                    case (int)EcatDriveMode::VELOCITY:
                                                        drive_mode[i] = CSV_MODE;
                                                        break;
                                                    case (int)EcatDriveMode::PP:
                                                        drive_mode[i] = PP_Mode;
                                                        break;
                                                    case (int)EcatDriveMode::PV:
                                                        drive_mode[i] = PV_Mode;
                                                        break;
                                                    default:
                                                        PRINT_DETAIL_LOG(PRINT_DETAIL_LOG_EN, __LINE__);
                                                        ROS_ERROR("unknown case[ecat_drive_mode[%d](%dd), node(%dd)]", i-IDX_ECAT_SLV_START, ecat_drive_mode[i-IDX_ECAT_SLV_START], i);
                                                        break;
                                                }
                                                break;
                                            case 1:
                                                memset(&usdo, 0, sizeof(usdo));
                                                psize = sizeof(usdo[0]);

                                                // int ec_SDOread(uint16 slave, uint16 index, uint8 subindex, boolean CA, int *psize, void *p, int timeout);
                                                ec_SDOread(i, MODE_OF_OP_DISPLAY_INDEX, MODE_OF_OP_DISPLAY_SUB_INDEX, FALSE, &psize, &usdo, EC_TIMEOUTRXM);
                                                if (EcatError) {
                                                    error_count[i]++;
                                                    #if 1
                                                    PRINT_DETAIL_LOG(PRINT_DETAIL_LOG_EN, __LINE__);
                                                    ROS_ERROR("node(%dd), EcatError: %s", i, ec_elist2string());
                                                    #endif
                                                } else {
                                                    error_count[i] = 0;
                                                }

                                                value = *(uint32_t*)usdo;

                                                if (value == drive_mode[i]) {
                                                    subFsm[i] = 3;
                                                } else {
                                                    subFsm[i] = 2;
                                                }
                                                break;
                                            case 2:
                                                memset(&usdo, 0, sizeof(usdo));
                                                usdo[0] = drive_mode[i];
                                                psize = sizeof(usdo[0]);

                                                // int ec_SDOwrite(uint16 Slave, uint16 Index, uint8 SubIndex, boolean CA, int psize, const void *p, int Timeout);
                                                ec_SDOwrite(i, MODE_OF_OP_INDEX, MODE_OF_OP_SUB_INDEX, FALSE, psize, &usdo, EC_TIMEOUTRXM);
                                                if (EcatError) {
                                                    error_count[i]++;
                                                    #if 1
                                                    PRINT_DETAIL_LOG(PRINT_DETAIL_LOG_EN, __LINE__);
                                                    ROS_ERROR("node(%dd), EcatError: %s", i, ec_elist2string());
                                                    #endif
                                                } else {
                                                    error_count[i] = 0;
                                                    retry_count[i]++;
                                                }

                                                subFsm[i] = 3;
                                                break;
                                            case 3:
                                                *((uint16_t*)(ec_slave[i].outputs)) = CONTROLWORD_RESET;
                                                subFsm[i] = 4;
                                                break;
                                            case 4:
                                                subFsm[i] = 5;
                                                break;
                                            case 5:
                                                switch (ecat_drive_mode[i-IDX_ECAT_SLV_START]) {
                                                    case (int)EcatDriveMode::NO_ECAT:
                                                        break;
                                                    case (int)EcatDriveMode::TORQUE:
                                                        *((uint16_t*)(ec_slave[i].outputs)) = CONTROLWORD_SHUTDOWN;
                                                        *((int16_t*)(ec_slave[i].outputs + sizeof(uint16_t))) = 0;
                                                        subFsm[i] = 6;
                                                    case (int)EcatDriveMode::VELOCITY:
                                                        *((uint16_t*)(ec_slave[i].outputs)) = CONTROLWORD_SHUTDOWN;
                                                        *((int32_t*)(ec_slave[i].outputs + sizeof(uint16_t))) = 0;
                                                        subFsm[i] = 6;
                                                        break;
                                                    case (int)EcatDriveMode::PP:
                                                        switch (subFsmPP[i]) {
                                                            case 0:
                                                                if (txPdoValuePre[i].statusword != txPdoValue[i].statusword) {
                                                                    printf("[%lf][ln:%5d]statusword: 0x%x\n", ros::Time().now().toSec(), __LINE__, txPdoValue[i].statusword);
                                                                }
                                                                *((uint16_t*)(ec_slave[i].outputs)) = CONTROLWORD_SHUTDOWN;
                                                                *((int32_t*)(ec_slave[i].outputs + sizeof(uint16_t))) = 0;
                                                                *((int32_t*)(ec_slave[i].outputs + sizeof(uint16_t) + sizeof(uint32_t))) = 0;
                                                                *((int32_t*)(ec_slave[i].outputs + sizeof(uint16_t) + sizeof(uint32_t) + sizeof(uint32_t))) = 0;
                                                                *((int32_t*)(ec_slave[i].outputs + sizeof(uint16_t) + sizeof(uint32_t) + sizeof(uint32_t) + sizeof(uint32_t))) = 0;
                                                                subFsmPP[i] = 1;
                                                                break;
                                                            case 1:
                                                                if (txPdoValuePre[i].statusword != txPdoValue[i].statusword) {
                                                                    printf("[%lf][ln:%5d]statusword: 0x%x\n", ros::Time().now().toSec(), __LINE__, txPdoValue[i].statusword);
                                                                }
                                                                *((uint16_t*)(ec_slave[i].outputs)) = CONTROLWORD_OP_EN;
                                                                subFsm[i] = 6;
                                                                subFsmPP[i] = 0;
                                                                break;
                                                            default:
                                                                PRINT_DETAIL_LOG(PRINT_DETAIL_LOG_EN, __LINE__);
                                                                ROS_ERROR("unknown case[subFsmPP(%dd), node(%dd)]", subFsmPP[i], i);
                                                                break;
                                                        }
                                                        break;
                                                    case (int)EcatDriveMode::PV:
                                                        switch (subFsmPP[i]) {
                                                            case 0:
                                                                if (txPdoValuePre[i].statusword != txPdoValue[i].statusword) {
                                                                    printf("[%lf][ln:%5d]statusword: 0x%x\n", ros::Time().now().toSec(), __LINE__, txPdoValue[i].statusword);
                                                                }
                                                                *((uint16_t*)(ec_slave[i].outputs)) = CONTROLWORD_SHUTDOWN;
                                                                *((int32_t*)(ec_slave[i].outputs + sizeof(uint16_t))) = 0;
                                                                *((int32_t*)(ec_slave[i].outputs + sizeof(uint16_t) + sizeof(uint32_t))) = 0;
                                                                *((int32_t*)(ec_slave[i].outputs + sizeof(uint16_t) + sizeof(uint32_t) + sizeof(uint32_t))) = 0;
                                                                subFsmPP[i] = 1;
                                                                break;
                                                            case 1:
                                                                if (txPdoValuePre[i].statusword != txPdoValue[i].statusword) {
                                                                    printf("[%lf][ln:%5d]statusword: 0x%x\n", ros::Time().now().toSec(), __LINE__, txPdoValue[i].statusword);
                                                                }
                                                                *((uint16_t*)(ec_slave[i].outputs)) = CONTROLWORD_OP_EN;
                                                                subFsm[i] = 6;
                                                                subFsmPP[i] = 0;
                                                                break;
                                                            default:
                                                                PRINT_DETAIL_LOG(PRINT_DETAIL_LOG_EN, __LINE__);
                                                                ROS_ERROR("unknown case[subFsmPP(%dd), node(%dd)]", subFsmPP[i], i);
                                                                break;
                                                        }
                                                        break;
                                                    default:
                                                        PRINT_DETAIL_LOG(PRINT_DETAIL_LOG_EN, __LINE__);
                                                        ROS_ERROR("unknown case[ecat_drive_mode[%d](%dd), node(%dd)]", i-IDX_ECAT_SLV_START, ecat_drive_mode[i-IDX_ECAT_SLV_START], i);
                                                        break;
                                                }
                                                break;
                                            case 6:
                                                PRINT_DETAIL_LOG(PRINT_DETAIL_LOG_EN, __LINE__);
                                                printf("case[EcatSlaveCmdType::INIT] drive_mode:%dd, node(%dd)\n", drive_mode[i], i);
                                                if (freerun) {
                                                    ROS_WARN("freerun [ecat_drive_mode[%d](%dd), node(%dd)]", i-IDX_ECAT_SLV_START, ecat_drive_mode[i-IDX_ECAT_SLV_START], i);
                                                    pEcatSlaveCommandsMsg[i]->type = (int)EcatSlaveCmdType::RUN;
                                                    ecatSlaveCommandsMsg_enabled[i] = 1;
                                                    motorInited[i] = 1;
                                                    subFsm[i] = 7;
                                                } else {
                                                    ecatSlaveCommandsMsg_enabled[i] = 0;
                                                    motorInited[i] = 1;
                                                    subFsm[i] = 7;
                                                }
                                                break;
                                            case 7:
                                                break;
                                            default:
                                                PRINT_DETAIL_LOG(PRINT_DETAIL_LOG_EN, __LINE__);
                                                printf("case[subFsm(%dd), node(%dd)]\n", subFsm[i], i);
                                                break;
                                        }

                                        if (error_count[i] > MAX_ERROR_COUNT) {
                                            subFsm[i] = -1;
                                        }

                                        if (retry_count[i] > MAX_RETRY_COUNT) {
                                            subFsm[i] = -3;
                                        }
                                        break;
                                    case EcatSlaveCmdType::RUN:
                                        switch (subFsm[i]) {
                                            case 0:
                                                switch (ecat_drive_mode[i-IDX_ECAT_SLV_START]) {
                                                    case (int)EcatDriveMode::NO_ECAT:
                                                        break;
                                                    case (int)EcatDriveMode::TORQUE:
                                                        *((uint16_t*)(ec_slave[i].outputs)) = CONTROLWORD_OP_EN;
                                                        *((int32_t*)(ec_slave[i].outputs + sizeof(uint16_t))) = 0;
                                                        break;
                                                    case (int)EcatDriveMode::VELOCITY:
                                                        *((uint16_t*)(ec_slave[i].outputs)) = CONTROLWORD_OP_EN;

                                                        switch (slave_arch[i-IDX_ECAT_SLV_START]) {
                                                            case (int)EcatSlaveArch::ANGULAR_SWIVEL:
                                                            case (int)EcatSlaveArch::LINEAR_PULLY:
                                                            case (int)EcatSlaveArch::LINEAR_SCREW:
                                                            case (int)EcatSlaveArch::RPM_LS:
                                                                // ls unit : UU
                                                                rpm_to_uus = (pEcatSlaveCommandsMsg[i]->command.rpm / 60.0) * motor_tick[i-IDX_ECAT_SLV_START];
                                                                *((int32_t*)(ec_slave[i].outputs + sizeof(uint16_t))) = rpm_to_uus;
                                                                break;
                                                            case (int)EcatSlaveArch::ANGULAR_WHEEL_WELCON:
                                                            case (int)EcatSlaveArch::RPM_WELCON:
                                                                // welcon unit : rpm
                                                                *((int32_t*)(ec_slave[i].outputs + sizeof(uint16_t))) = pEcatSlaveCommandsMsg[i]->command.rpm;
                                                                break;
                                                            default:
                                                                break;
                                                        }
                                                        break;
                                                    case (int)EcatDriveMode::PP:
                                                        switch (subFsmPP[i]) {
                                                            case 0:
                                                                ecatSlaveCommandsMsg_enabled[i] = 1;
                                                                subFsmPP[i]++;
                                                                break;
                                                            case 1:
                                                                *((uint16_t*)(ec_slave[i].outputs)) = CONTROLWORD_OP_EN;
                                                                *((int32_t*)(ec_slave[i].outputs + sizeof(uint16_t))) = pEcatSlaveCommandsMsg[i]->command.position;
                                                                switch (slave_arch[i-IDX_ECAT_SLV_START]) {
                                                                    case (int)EcatSlaveArch::ANGULAR_SWIVEL:
                                                                    case (int)EcatSlaveArch::LINEAR_PULLY:
                                                                    case (int)EcatSlaveArch::LINEAR_SCREW:
                                                                        // ls unit : UU
                                                                        rpm_to_uus = (pEcatSlaveCommandsMsg[i]->command.rpm / 60.0) * motor_tick[i-IDX_ECAT_SLV_START];
                                                                        *((int32_t*)(ec_slave[i].outputs + sizeof(uint16_t) + sizeof(uint32_t))) = rpm_to_uus;
                                                                        acc_to_uus2 = (pEcatSlaveCommandsMsg[i]->command.acc / 60.0) * motor_tick[i-IDX_ECAT_SLV_START];
                                                                        *((int32_t*)(ec_slave[i].outputs + sizeof(uint16_t) + sizeof(uint32_t) + sizeof(uint32_t))) = acc_to_uus2;
                                                                        dec_to_uus2 = (pEcatSlaveCommandsMsg[i]->command.dec / 60.0) * motor_tick[i-IDX_ECAT_SLV_START];
                                                                        *((int32_t*)(ec_slave[i].outputs + sizeof(uint16_t) + sizeof(uint32_t) + sizeof(uint32_t) + sizeof(uint32_t))) = dec_to_uus2;
                                                                        break;
                                                                    case (int)EcatSlaveArch::ANGULAR_WHEEL_WELCON:
                                                                        // welcon unit : rpm
                                                                        break;
                                                                    case (int)EcatSlaveArch::RPM_LS:
                                                                        // ls unit : UU
                                                                        break;
                                                                    case (int)EcatSlaveArch::RPM_WELCON:
                                                                        // welcon unit : rpm
                                                                        break;
                                                                    default:
                                                                        break;
                                                                }
                                                                subFsmPP[i]++;
                                                                break;
                                                            case 2:
                                                                *((uint16_t*)(ec_slave[i].outputs)) = CONTROLWORD_OP_PP_START;
                                                                subFsmPP[i]++;
                                                                break;
                                                            case 3:
                                                                ecatSlaveCommandsMsg_enabled[i] = 0;
                                                                subFsmPP[i] = 0;
                                                                break;
                                                            default:
                                                                PRINT_DETAIL_LOG(PRINT_DETAIL_LOG_EN, __LINE__);
                                                                ROS_ERROR("unknown case[subFsmPP(%dd), node(%dd)]", subFsmPP[i], i);
                                                                break;
                                                        }
                                                        break;
                                                    case (int)EcatDriveMode::PV:
                                                        *((uint16_t*)(ec_slave[i].outputs)) = CONTROLWORD_OP_EN;

                                                        switch (slave_arch[i-IDX_ECAT_SLV_START]) {
                                                            case (int)EcatSlaveArch::ANGULAR_SWIVEL:
                                                            case (int)EcatSlaveArch::LINEAR_PULLY:
                                                            case (int)EcatSlaveArch::LINEAR_SCREW:
                                                                // ls unit : UU
                                                                break;
                                                            case (int)EcatSlaveArch::ANGULAR_WHEEL_WELCON:
                                                            case (int)EcatSlaveArch::RPM_WELCON:
                                                                // welcon unit : rpm
                                                                *((int32_t*)(ec_slave[i].outputs + sizeof(uint16_t))) = pEcatSlaveCommandsMsg[i]->command.rpm;
                                                                *((int32_t*)(ec_slave[i].outputs + sizeof(uint16_t) + sizeof(uint32_t))) = pEcatSlaveCommandsMsg[i]->command.acc;
                                                                *((int32_t*)(ec_slave[i].outputs + sizeof(uint16_t) + sizeof(uint32_t) + sizeof(uint32_t))) = pEcatSlaveCommandsMsg[i]->command.dec;
                                                                break;
                                                            case (int)EcatSlaveArch::RPM_LS:
                                                                // ls unit : UU
                                                                rpm_to_uus = (pEcatSlaveCommandsMsg[i]->command.rpm / 60.0) * motor_tick[i-IDX_ECAT_SLV_START];
                                                                *((int32_t*)(ec_slave[i].outputs + sizeof(uint16_t) + sizeof(uint32_t))) = rpm_to_uus;
                                                                acc_to_uus2 = (pEcatSlaveCommandsMsg[i]->command.acc / 60.0) * motor_tick[i-IDX_ECAT_SLV_START];
                                                                *((int32_t*)(ec_slave[i].outputs + sizeof(uint16_t) + sizeof(uint32_t) + sizeof(uint32_t))) = acc_to_uus2;
                                                                dec_to_uus2 = (pEcatSlaveCommandsMsg[i]->command.dec / 60.0) * motor_tick[i-IDX_ECAT_SLV_START];
                                                                *((int32_t*)(ec_slave[i].outputs + sizeof(uint16_t) + sizeof(uint32_t) + sizeof(uint32_t) + sizeof(uint32_t))) = dec_to_uus2;
                                                                break;
                                                            default:
                                                                break;
                                                        }
                                                        break;
                                                    default:
                                                        PRINT_DETAIL_LOG(PRINT_DETAIL_LOG_EN, __LINE__);
                                                        ROS_ERROR("unknown case[ecat_drive_mode[%d](%dd), node(%dd)]", i-IDX_ECAT_SLV_START, ecat_drive_mode[i-IDX_ECAT_SLV_START], i);
                                                        break;
                                                }
                                                break;
                                            default:
                                                PRINT_DETAIL_LOG(PRINT_DETAIL_LOG_EN, __LINE__);
                                                printf("case[subFsm(%dd)], node(%dd)\n", subFsm[i], i);
                                                break;
                                        }
                                        break;
                                    case EcatSlaveCmdType::STOP:
                                        switch (subFsm[i]) {
                                            case 0:
                                                switch (ecat_drive_mode[i-IDX_ECAT_SLV_START]) {
                                                    case (int)EcatDriveMode::NO_ECAT:
                                                        break;
                                                    case (int)EcatDriveMode::TORQUE:
                                                        *((uint16_t*)(ec_slave[i].outputs)) = CONTROLWORD_Q_STOP;
                                                        *((int16_t*)(ec_slave[i].outputs + sizeof(uint16_t))) = 0;
                                                        break;
                                                    case (int)EcatDriveMode::VELOCITY:
                                                        *((uint16_t*)(ec_slave[i].outputs)) = CONTROLWORD_Q_STOP;
                                                        *((int32_t*)(ec_slave[i].outputs + sizeof(uint16_t))) = 0;
                                                        break;
                                                    case (int)EcatDriveMode::PP:
                                                        *((uint16_t*)(ec_slave[i].outputs)) = CONTROLWORD_Q_STOP;
                                                        *((int32_t*)(ec_slave[i].outputs + sizeof(uint16_t))) = 0;
                                                        *((int32_t*)(ec_slave[i].outputs + sizeof(uint16_t) + sizeof(uint32_t))) = 0;
                                                        *((int32_t*)(ec_slave[i].outputs + sizeof(uint16_t) + sizeof(uint32_t) + sizeof(uint32_t))) = 0;
                                                        *((int32_t*)(ec_slave[i].outputs + sizeof(uint16_t) + sizeof(uint32_t) + sizeof(uint32_t) + sizeof(uint32_t))) = 0;
                                                        break;
                                                    case (int)EcatDriveMode::PV:
                                                        *((uint16_t*)(ec_slave[i].outputs)) = CONTROLWORD_Q_STOP;
                                                        *((int32_t*)(ec_slave[i].outputs + sizeof(uint16_t))) = 0;
                                                        *((int32_t*)(ec_slave[i].outputs + sizeof(uint16_t) + sizeof(uint32_t))) = 0;
                                                        *((int32_t*)(ec_slave[i].outputs + sizeof(uint16_t) + sizeof(uint32_t) + sizeof(uint32_t))) = 0;
                                                        break;
                                                    default:
                                                        PRINT_DETAIL_LOG(PRINT_DETAIL_LOG_EN, __LINE__);
                                                        ROS_ERROR("unknown case[ecat_drive_mode[%d](%dd), node(%dd)]", i-IDX_ECAT_SLV_START, ecat_drive_mode[i-IDX_ECAT_SLV_START], i);
                                                        break;
                                                }
                                                break;
                                            default:
                                                PRINT_DETAIL_LOG(PRINT_DETAIL_LOG_EN, __LINE__);
                                                printf("case[subFsm(%dd)], node(%dd)\n", subFsm[i], i);
                                                break;
                                        }
                                        break;
                                    case EcatSlaveCmdType::RESET:
                                        switch (subFsm[i]) {
                                            case 0:
                                                switch (ecat_drive_mode[i-IDX_ECAT_SLV_START]) {
                                                    case (int)EcatDriveMode::NO_ECAT:
                                                        break;
                                                    case (int)EcatDriveMode::TORQUE:
                                                        *((uint16_t*)(ec_slave[i].outputs)) = CONTROLWORD_RESET;
                                                        *((int16_t*)(ec_slave[i].outputs + sizeof(uint16_t))) = 0;
                                                        break;
                                                    case (int)EcatDriveMode::VELOCITY:
                                                        *((uint16_t*)(ec_slave[i].outputs)) = CONTROLWORD_RESET;
                                                        *((int32_t*)(ec_slave[i].outputs + sizeof(uint16_t))) = 0;
                                                        break;
                                                    case (int)EcatDriveMode::PP:
                                                        *((uint16_t*)(ec_slave[i].outputs)) = CONTROLWORD_RESET;
                                                        *((int32_t*)(ec_slave[i].outputs + sizeof(uint16_t))) = 0;
                                                        *((int32_t*)(ec_slave[i].outputs + sizeof(uint16_t) + sizeof(uint32_t))) = 0;
                                                        *((int32_t*)(ec_slave[i].outputs + sizeof(uint16_t) + sizeof(uint32_t) + sizeof(uint32_t))) = 0;
                                                        *((int32_t*)(ec_slave[i].outputs + sizeof(uint16_t) + sizeof(uint32_t) + sizeof(uint32_t) + sizeof(uint32_t))) = 0;
                                                        break;
                                                    case (int)EcatDriveMode::PV:
                                                        *((uint16_t*)(ec_slave[i].outputs)) = CONTROLWORD_RESET;
                                                        *((int32_t*)(ec_slave[i].outputs + sizeof(uint16_t))) = 0;
                                                        *((int32_t*)(ec_slave[i].outputs + sizeof(uint16_t) + sizeof(uint32_t))) = 0;
                                                        *((int32_t*)(ec_slave[i].outputs + sizeof(uint16_t) + sizeof(uint32_t) + sizeof(uint32_t))) = 0;
                                                        break;
                                                    default:
                                                        PRINT_DETAIL_LOG(PRINT_DETAIL_LOG_EN, __LINE__);
                                                        ROS_ERROR("unknown case[ecat_drive_mode[%d](%dd), node(%dd)]", i-IDX_ECAT_SLV_START, ecat_drive_mode[i-IDX_ECAT_SLV_START], i);
                                                        break;
                                                }
                                                break;
                                            default:
                                                PRINT_DETAIL_LOG(PRINT_DETAIL_LOG_EN, __LINE__);
                                                printf("case[subFsm(%dd)], node(%dd)\n", subFsm[i], i);
                                                break;
                                        }
                                        break;
                                    case EcatSlaveCmdType::HOME:
                                        switch (subFsm[i]) {
                                            case -3:
                                                break;
                                            case -2:
                                                PRINT_DETAIL_LOG(PRINT_DETAIL_LOG_EN, __LINE__);
                                                printf("case[EcatSlaveCmdType::HOME]: MAX_RETRY_COUNT[node(%dd)]\n", i);
                                                subFsm[i] = -3;
                                                break;
                                            case -1:
                                                PRINT_DETAIL_LOG(PRINT_DETAIL_LOG_EN, __LINE__);
                                                printf("case[EcatSlaveCmdType::HOME]: MAX_ERROR_COUNT[node(%dd)]\n", i);
                                                subFsm[i] = -3;
                                                break;
                                            case 0:
                                                error_count[i] = 0;
                                                retry_count[i] = 0;
                                                switch (ecat_drive_mode[i-IDX_ECAT_SLV_START]) {
                                                    case (int)EcatDriveMode::NO_ECAT:
                                                        break;
                                                    case (int)EcatDriveMode::PP:
                                                        subFsm[i] = 1;
                                                        break;
                                                    default:
                                                        subFsm[i] = -3;
                                                        PRINT_DETAIL_LOG(PRINT_DETAIL_LOG_EN, __LINE__);
                                                        ROS_ERROR("homing not supported. [ecat_drive_mode[%d](%dd), node(%dd)]", i-IDX_ECAT_SLV_START, ecat_drive_mode[i-IDX_ECAT_SLV_START], i);
                                                        break;
                                                }
                                                break;
                                            case 1:
                                                memset(&usdo, 0, sizeof(usdo));
                                                psize = sizeof(usdo[0]);

                                                // int ec_SDOread(uint16 slave, uint16 index, uint8 subindex, boolean CA, int *psize, void *p, int timeout);
                                                ec_SDOread(i, MODE_OF_OP_DISPLAY_INDEX, MODE_OF_OP_DISPLAY_SUB_INDEX, FALSE, &psize, &usdo, EC_TIMEOUTRXM);
                                                if (EcatError) {
                                                    error_count[i]++;
                                                    #if 1
                                                    PRINT_DETAIL_LOG(PRINT_DETAIL_LOG_EN, __LINE__);
                                                    ROS_ERROR("node(%dd), EcatError: %s", i, ec_elist2string());
                                                    #endif
                                                } else {
                                                    error_count[i] = 0;
                                                }

                                                value = *(uint32_t*)usdo;

                                                if (value == HM_Mode) {
                                                    subFsm[i] = 3;
                                                } else {
                                                    subFsm[i] = 2;
                                                }
                                                break;
                                            case 2:
                                                memset(&usdo, 0, sizeof(usdo));
                                                usdo[0] = HM_Mode;
                                                psize = sizeof(usdo[0]);

                                                // int ec_SDOwrite(uint16 Slave, uint16 Index, uint8 SubIndex, boolean CA, int psize, const void *p, int Timeout);
                                                ec_SDOwrite(i, MODE_OF_OP_INDEX, MODE_OF_OP_SUB_INDEX, FALSE, psize, &usdo, EC_TIMEOUTRXM);
                                                if (EcatError) {
                                                    error_count[i]++;
                                                    #if 1
                                                    PRINT_DETAIL_LOG(PRINT_DETAIL_LOG_EN, __LINE__);
                                                    ROS_ERROR("node(%dd), EcatError: %s", i, ec_elist2string());
                                                    #endif
                                                } else {
                                                    error_count[i] = 0;
                                                    retry_count[i]++;
                                                }

                                                subFsm[i] = 3;
                                                break;
                                            case 3:
                                                error_count[i] = 0;
                                                retry_count[i] = 0;
                                                subFsm[i] = 4;
                                                break;
                                            case 4:
                                                memset(&usdo, 0, sizeof(usdo));
                                                psize = sizeof(usdo[0]);

                                                // int ec_SDOread(uint16 slave, uint16 index, uint8 subindex, boolean CA, int *psize, void *p, int timeout);
                                                ec_SDOread(i, HOMING_METHOD_INDEX, HOMING_METHOD_SUB_INDEX, FALSE, &psize, &usdo, EC_TIMEOUTRXM);
                                                if (EcatError) {
                                                    error_count[i]++;
                                                    #if 1
                                                    PRINT_DETAIL_LOG(PRINT_DETAIL_LOG_EN, __LINE__);
                                                    ROS_ERROR("node(%dd), EcatError: %s", i, ec_elist2string());
                                                    #endif
                                                } else {
                                                    error_count[i] = 0;
                                                }

                                                value = *(uint32_t*)usdo;

                                                if (value == HOME_ON_CURRENT_POSITION) {
                                                    subFsm[i] = 6;
                                                } else {
                                                    subFsm[i] = 5;
                                                }
                                                break;
                                            case 5:
                                                memset(&usdo, 0, sizeof(usdo));
                                                usdo[0] = HOME_ON_CURRENT_POSITION;
                                                psize = sizeof(usdo[0]);

                                                // int ec_SDOwrite(uint16 Slave, uint16 Index, uint8 SubIndex, boolean CA, int psize, const void *p, int Timeout);
                                                ec_SDOwrite(i, HOMING_METHOD_INDEX, HOMING_METHOD_SUB_INDEX, FALSE, psize, &usdo, EC_TIMEOUTRXM);
                                                if (EcatError) {
                                                    error_count[i]++;
                                                    #if 1
                                                    PRINT_DETAIL_LOG(PRINT_DETAIL_LOG_EN, __LINE__);
                                                    ROS_ERROR("node(%dd), EcatError: %s", i, ec_elist2string());
                                                    #endif
                                                } else {
                                                    error_count[i] = 0;
                                                    retry_count[i]++;
                                                }

                                                subFsm[i] = 4;
                                                break;
                                            case 6:
                                                switch (ecat_drive_mode[i-IDX_ECAT_SLV_START]) {
                                                    case (int)EcatDriveMode::NO_ECAT:
                                                        break;
                                                    case (int)EcatDriveMode::PP:
                                                        *((uint16_t*)(ec_slave[i].outputs)) = CONTROLWORD_SHUTDOWN;
                                                        *((int32_t*)(ec_slave[i].outputs + sizeof(uint16_t))) = 0;
                                                        *((int32_t*)(ec_slave[i].outputs + sizeof(uint16_t) + sizeof(uint32_t))) = 0;
                                                        *((int32_t*)(ec_slave[i].outputs + sizeof(uint16_t) + sizeof(uint32_t) + sizeof(uint32_t))) = 0;
                                                        *((int32_t*)(ec_slave[i].outputs + sizeof(uint16_t) + sizeof(uint32_t) + sizeof(uint32_t) + sizeof(uint32_t))) = 0;
                                                        subFsm[i] = 7;
                                                        break;
                                                    default:
                                                        subFsm[i] = -3;
                                                        PRINT_DETAIL_LOG(PRINT_DETAIL_LOG_EN, __LINE__);
                                                        ROS_ERROR("homing not supported. [ecat_drive_mode[%d](%dd), node(%dd)]", i-IDX_ECAT_SLV_START, ecat_drive_mode[i-IDX_ECAT_SLV_START], i);
                                                        break;
                                                }
                                                break;
                                            case 7:
                                                switch (ecat_drive_mode[i-IDX_ECAT_SLV_START]) {
                                                    case (int)EcatDriveMode::NO_ECAT:
                                                        break;
                                                    case (int)EcatDriveMode::PP:
                                                        *((uint16_t*)(ec_slave[i].outputs)) = CONTROLWORD_OP_HOMING;
                                                        *((int32_t*)(ec_slave[i].outputs + sizeof(uint16_t))) = 0;
                                                        *((int32_t*)(ec_slave[i].outputs + sizeof(uint16_t) + sizeof(uint32_t))) = 0;
                                                        *((int32_t*)(ec_slave[i].outputs + sizeof(uint16_t) + sizeof(uint32_t) + sizeof(uint32_t))) = 0;
                                                        *((int32_t*)(ec_slave[i].outputs + sizeof(uint16_t) + sizeof(uint32_t) + sizeof(uint32_t) + sizeof(uint32_t))) = 0;
                                                        subFsm[i] = 8;
                                                        break;
                                                    default:
                                                        subFsm[i] = -3;
                                                        PRINT_DETAIL_LOG(PRINT_DETAIL_LOG_EN, __LINE__);
                                                        ROS_ERROR("homing not supported. [ecat_drive_mode[%d](%dd), node(%dd)]", i-IDX_ECAT_SLV_START, ecat_drive_mode[i-IDX_ECAT_SLV_START], i);
                                                        break;
                                                }
                                                break;
                                            case 8:
                                                // homing done check
                                                subFsm[i] = 9;
                                                break;
                                            case 9:
                                                error_count[i] = 0;
                                                retry_count[i] = 0;
                                                subFsm[i] = 10;
                                                break;
                                            case 10:
                                                memset(&usdo, 0, sizeof(usdo));
                                                psize = sizeof(usdo[0]);

                                                // int ec_SDOread(uint16 slave, uint16 index, uint8 subindex, boolean CA, int *psize, void *p, int timeout);
                                                ec_SDOread(i, MODE_OF_OP_DISPLAY_INDEX, MODE_OF_OP_DISPLAY_SUB_INDEX, FALSE, &psize, &usdo, EC_TIMEOUTRXM);
                                                if (EcatError) {
                                                    error_count[i]++;
                                                    #if 1
                                                    PRINT_DETAIL_LOG(PRINT_DETAIL_LOG_EN, __LINE__);
                                                    ROS_ERROR("node(%dd), EcatError: %s", i, ec_elist2string());
                                                    #endif
                                                } else {
                                                    error_count[i] = 0;
                                                }

                                                value = *(uint32_t*)usdo;

                                                if (value == drive_mode[i]) {
                                                    subFsm[i] = 12;
                                                } else {
                                                    subFsm[i] = 11;
                                                }
                                                break;
                                            case 11:
                                                memset(&usdo, 0, sizeof(usdo));
                                                usdo[0] = drive_mode[i];
                                                psize = sizeof(usdo[0]);

                                                // int ec_SDOwrite(uint16 Slave, uint16 Index, uint8 SubIndex, boolean CA, int psize, const void *p, int Timeout);
                                                ec_SDOwrite(i, MODE_OF_OP_INDEX, MODE_OF_OP_SUB_INDEX, FALSE, psize, &usdo, EC_TIMEOUTRXM);
                                                if (EcatError) {
                                                    error_count[i]++;
                                                    #if 1
                                                    PRINT_DETAIL_LOG(PRINT_DETAIL_LOG_EN, __LINE__);
                                                    ROS_ERROR("node(%dd), EcatError: %s", i, ec_elist2string());
                                                    #endif
                                                } else {
                                                    error_count[i] = 0;
                                                    retry_count[i]++;
                                                }

                                                subFsm[i] = 10;
                                                break;
                                            case 12:
                                                PRINT_DETAIL_LOG(PRINT_DETAIL_LOG_EN, __LINE__);
                                                printf("case[EcatSlaveCmdType::HOME] drive_mode:%dd, node(%dd)\n", drive_mode[i], i);
                                                ecatSlaveCommandsMsg_enabled[i] = 0;
                                                subFsm[i] = 13;
                                                break;
                                            case 13:
                                                break;
                                            default:
                                                PRINT_DETAIL_LOG(PRINT_DETAIL_LOG_EN, __LINE__);
                                                printf("case[subFsm(%dd), node(%dd)]\n", subFsm[i], i);
                                                break;
                                        }

                                        if (error_count[i] > MAX_ERROR_COUNT) {
                                            subFsm[i] = -1;
                                        }

                                        if (retry_count[i] > MAX_RETRY_COUNT) {
                                            subFsm[i] = -2;
                                        }
                                        break;
                                    case EcatSlaveCmdType::SDO_RD:
                                        switch (subFsm[i]) {
                                            case 0:
                                                sdoData[i].index = pEcatSlaveCommandsMsg[i]->sdo.index;
                                                sdoData[i].subindex = pEcatSlaveCommandsMsg[i]->sdo.subindex;
                                                sdoData[i].CA = pEcatSlaveCommandsMsg[i]->sdo.CA;
                                                sdoData[i].size = pEcatSlaveCommandsMsg[i]->sdo.size;

                                                memset(&usdo, 0, sizeof(usdo));
                                                // int ec_SDOread(uint16 slave, uint16 index, uint8 subindex, boolean CA, int *psize, void *p, int timeout);
                                                psize = sdoData[i].size;
                                                ec_SDOread(i, sdoData[i].index, sdoData[i].subindex, sdoData[i].CA, &psize, &usdo, EC_TIMEOUTRXM);
                                                if (EcatError) {
                                                    error = TRUE;
                                                    PRINT_DETAIL_LOG(PRINT_DETAIL_LOG_EN, __LINE__);
                                                    ROS_ERROR("node(%dd), EcatError: %s", i, ec_elist2string());
                                                } else {
                                                    error = FALSE;
                                                    PRINT_DETAIL_LOG(PRINT_DETAIL_LOG_EN, __LINE__);
                                                    printf("node(%dd), sdo read success: ", i);
                                                    for (int i=0; i<(int)sizeof(usdo); i++) {
                                                        printf("[%02x]", usdo[i]);
                                                    }
                                                    printf("\n");
                                                }

                                                sdoData[i].size = psize;
                                                value = *(uint32_t*)usdo;
                                                pEcatSlaveInfosMsg[i]->sdo.ts = ros::Time::now().toSec();
                                                pEcatSlaveInfosMsg[i]->sdo.index = sdoData[i].index;
                                                pEcatSlaveInfosMsg[i]->sdo.subindex = sdoData[i].subindex;
                                                pEcatSlaveInfosMsg[i]->sdo.CA = sdoData[i].CA;
                                                pEcatSlaveInfosMsg[i]->sdo.size = sdoData[i].size;
                                                pEcatSlaveInfosMsg[i]->sdo.error = error;
                                                pEcatSlaveInfosMsg[i]->sdo.value = value;

                                                PRINT_DETAIL_LOG(PRINT_DETAIL_LOG_EN, __LINE__);
                                                printf("case[EcatSlaveCmdType::SDO_RD], node(%dd)\n", i);
                                                subFsm[i] = 1;
                                                break;
                                            case 1:
                                                break;
                                            default:
                                                PRINT_DETAIL_LOG(PRINT_DETAIL_LOG_EN, __LINE__);
                                                printf("case[subFsm(%dd)], node(%dd)\n", subFsm[i], i);
                                                break;
                                        }
                                        break;
                                    case EcatSlaveCmdType::SDO_WR:
                                        switch (subFsm[i]) {
                                            case 0:
                                                sdoData[i].index = pEcatSlaveCommandsMsg[i]->sdo.index;
                                                sdoData[i].subindex = pEcatSlaveCommandsMsg[i]->sdo.subindex;
                                                sdoData[i].CA = pEcatSlaveCommandsMsg[i]->sdo.CA;
                                                sdoData[i].size = pEcatSlaveCommandsMsg[i]->sdo.size;
                                                value = pEcatSlaveCommandsMsg[i]->sdo.value;

                                                memcpy(&usdo, (uint8_t*)&value, sizeof(usdo));
                                                // int ec_SDOwrite(uint16 Slave, uint16 Index, uint8 SubIndex, boolean CA, int psize, const void *p, int Timeout);
                                                psize = sdoData[i].size;
                                                ec_SDOwrite(i, sdoData[i].index, sdoData[i].subindex, sdoData[i].CA, psize, &usdo, EC_TIMEOUTRXM);
                                                if (EcatError) {
                                                    error = TRUE;
                                                    PRINT_DETAIL_LOG(PRINT_DETAIL_LOG_EN, __LINE__);
                                                    ROS_ERROR("node(%dd), EcatError: %s", i, ec_elist2string());
                                                } else {
                                                    error = FALSE;
                                                    PRINT_DETAIL_LOG(PRINT_DETAIL_LOG_EN, __LINE__);
                                                    printf("node(%dd), sdo write success: ", i);
                                                    for (int i=0; i<(int)sizeof(usdo); i++) {
                                                        printf("[%02x]", usdo[i]);
                                                    }
                                                    printf("\n");
                                                }

                                                sdoData[i].size = psize;
                                                value = *(uint32_t*)usdo;
                                                pEcatSlaveInfosMsg[i]->sdo.ts = ros::Time::now().toSec();
                                                pEcatSlaveInfosMsg[i]->sdo.index = sdoData[i].index;
                                                pEcatSlaveInfosMsg[i]->sdo.subindex = sdoData[i].subindex;
                                                pEcatSlaveInfosMsg[i]->sdo.CA = sdoData[i].CA;
                                                pEcatSlaveInfosMsg[i]->sdo.size = sdoData[i].size;
                                                pEcatSlaveInfosMsg[i]->sdo.error = error;
                                                pEcatSlaveInfosMsg[i]->sdo.value = value;

                                                PRINT_DETAIL_LOG(PRINT_DETAIL_LOG_EN, __LINE__);
                                                printf("case[EcatSlaveCmdType::SDO_WR], node(%dd)\n", i);
                                                subFsm[i] = 1;
                                                break;
                                            case 1:
                                                break;
                                            default:
                                                PRINT_DETAIL_LOG(PRINT_DETAIL_LOG_EN, __LINE__);
                                                printf("case[subFsm(%dd)], node(%dd)\n", subFsm[i], i);
                                                break;
                                        }
                                        break;
                                    case EcatSlaveCmdType::PDO:
                                        switch (subFsm[i]) {
                                            case 0:
                                                switch (ecat_drive_mode[i-IDX_ECAT_SLV_START]) {
                                                    case (int)EcatDriveMode::NO_ECAT:
                                                        break;
                                                    case (int)EcatDriveMode::TORQUE:
                                                        rxPdoValue[i].controlword = pEcatSlaveCommandsMsg[i]->rxPdo.controlword;
                                                        rxPdoValue[i].tor_tar = pEcatSlaveCommandsMsg[i]->rxPdo.tor_tar;

                                                        *((uint16_t*)(ec_slave[i].outputs)) = rxPdoValue[i].controlword;
                                                        *((int16_t*)(ec_slave[i].outputs + sizeof(uint16_t))) = rxPdoValue[i].tor_tar;
                                                        break;
                                                    case (int)EcatDriveMode::VELOCITY:
                                                        rxPdoValue[i].controlword = pEcatSlaveCommandsMsg[i]->rxPdo.controlword;
                                                        rxPdoValue[i].vel_tar = pEcatSlaveCommandsMsg[i]->rxPdo.vel_tar;

                                                        *((uint16_t*)(ec_slave[i].outputs)) = rxPdoValue[i].controlword;
                                                        *((int32_t*)(ec_slave[i].outputs + sizeof(uint16_t))) = rxPdoValue[i].vel_tar;
                                                        break;
                                                    case (int)EcatDriveMode::PP:
                                                        rxPdoValue[i].controlword = pEcatSlaveCommandsMsg[i]->rxPdo.controlword;
                                                        rxPdoValue[i].pos_tar = pEcatSlaveCommandsMsg[i]->rxPdo.pos_tar;
                                                        rxPdoValue[i].vel_tar = pEcatSlaveCommandsMsg[i]->rxPdo.vel_tar;
                                                        rxPdoValue[i].acc_tar = pEcatSlaveCommandsMsg[i]->rxPdo.acc_tar;
                                                        rxPdoValue[i].dec_tar = pEcatSlaveCommandsMsg[i]->rxPdo.dec_tar;

                                                        *((uint16_t*)(ec_slave[i].outputs)) = rxPdoValue[i].controlword;
                                                        *((int32_t*)(ec_slave[i].outputs + sizeof(uint16_t))) = rxPdoValue[i].pos_tar;
                                                        *((int32_t*)(ec_slave[i].outputs + sizeof(uint16_t) + sizeof(uint32_t))) = rxPdoValue[i].vel_tar;
                                                        *((int32_t*)(ec_slave[i].outputs + sizeof(uint16_t) + sizeof(uint32_t) + sizeof(uint32_t))) = rxPdoValue[i].acc_tar;
                                                        *((int32_t*)(ec_slave[i].outputs + sizeof(uint16_t) + sizeof(uint32_t) + sizeof(uint32_t) + sizeof(uint32_t))) = rxPdoValue[i].dec_tar;
                                                        break;
                                                    case (int)EcatDriveMode::PV:
                                                        rxPdoValue[i].controlword = pEcatSlaveCommandsMsg[i]->rxPdo.controlword;
                                                        rxPdoValue[i].pos_tar = pEcatSlaveCommandsMsg[i]->rxPdo.pos_tar;
                                                        rxPdoValue[i].vel_tar = pEcatSlaveCommandsMsg[i]->rxPdo.vel_tar;
                                                        rxPdoValue[i].acc_tar = pEcatSlaveCommandsMsg[i]->rxPdo.acc_tar;
                                                        rxPdoValue[i].dec_tar = pEcatSlaveCommandsMsg[i]->rxPdo.dec_tar;

                                                        *((uint16_t*)(ec_slave[i].outputs)) = rxPdoValue[i].controlword;
                                                        *((int32_t*)(ec_slave[i].outputs + sizeof(uint16_t))) = rxPdoValue[i].vel_tar;
                                                        *((int32_t*)(ec_slave[i].outputs + sizeof(uint16_t) + sizeof(uint32_t))) = rxPdoValue[i].acc_tar;
                                                        *((int32_t*)(ec_slave[i].outputs + sizeof(uint16_t) + sizeof(uint32_t) + sizeof(uint32_t))) = rxPdoValue[i].dec_tar;
                                                        break;
                                                    default:
                                                        PRINT_DETAIL_LOG(PRINT_DETAIL_LOG_EN, __LINE__);
                                                        ROS_ERROR("unknown case[ecat_drive_mode[%d](%dd), node(%dd)]", i-IDX_ECAT_SLV_START, ecat_drive_mode[i-IDX_ECAT_SLV_START], i);
                                                        break;
                                                }

                                                PRINT_DETAIL_LOG(PRINT_DETAIL_LOG_EN, __LINE__);
                                                printf("case[EcatSlaveCmdType::PDO], node(%dd)\n", i);
                                                break;
                                            default:
                                                PRINT_DETAIL_LOG(PRINT_DETAIL_LOG_EN, __LINE__);
                                                printf("case[subFsm(%dd)], node(%dd)\n", subFsm[i], i);
                                                break;
                                        }
                                        break;
                                    default:
                                        PRINT_DETAIL_LOG(PRINT_DETAIL_LOG_EN, __LINE__);
                                        ROS_ERROR("unknown case(%dd, 0x%x), node(%dd)", pEcatSlaveCommandsMsg[i]->type, pEcatSlaveCommandsMsg[i]->type, i);
                                        break;
                                }
                            }
                        }
                        ecatLoopRun = TRUE;
                        needlf = TRUE;
                    }
                    osal_usleep(5000);

                    ecatStateMsg.ts = ros::Time::now().toSec();
                    ecatStateMsg.allMotorInited = 0;
                    for (int i=IDX_ECAT_SLV_START; i<(IDX_ECAT_SLV_START+motor_num); i++) {
                        ecatStateMsg.allMotorInited += motorInited[i];
                        ecatStateMsg.motorInited[i-IDX_ECAT_SLV_START] = motorInited[i];
                        ecatStateMsg.ecatSlaveCommandsMsg_callbacked[i-IDX_ECAT_SLV_START] = ecatSlaveCommandsMsg_callbacked[i];
                        ecatStateMsg.ecatSlaveCommandsMsg_enabled[i-IDX_ECAT_SLV_START] = ecatSlaveCommandsMsg_enabled[i];
                        ecatStateMsg.ecatSlaveCmdType[i-IDX_ECAT_SLV_START] = EcatSlaveCmdType2String(ecatSlaveCmdType[i]);
                        ecatStateMsg.ecatSlaveCmdTypePre[i-IDX_ECAT_SLV_START] = EcatSlaveCmdType2String(ecatSlaveCmdTypePre[i]);
                        ecatStateMsg.subFsm[i-IDX_ECAT_SLV_START] = subFsm[i];
                        ecatStateMsg.subFsmPP[i-IDX_ECAT_SLV_START] = subFsmPP[i];
                        ecatStateMsg.statusword[i-IDX_ECAT_SLV_START] = (boost::format("0x%04X") % txPdoValue[i].statusword).str();
                        ecatStateMsg.controlword[i-IDX_ECAT_SLV_START] = (boost::format("0x%04X") % wroteControlword[i]).str();
                        ecatStateMsg.fault[i-IDX_ECAT_SLV_START] = (txPdoValue[i].statusword >> STATUSWORD_BIT_FAULT) & 0x01;
                        ecatStateMsg.error_code[i-IDX_ECAT_SLV_START] = (boost::format("0x%04X") % txPdoValue[i].error_code).str();
                    }
                }
                for (int i=IDX_ECAT_SLV_START; i<(IDX_ECAT_SLV_START+motor_num); i++) {
                    ec_dcsync0(i, FALSE, SYNC0TIME, 0);
                }
                inOP = FALSE;
            } else {
                ROS_ERROR("Not all slaves reached operational state.");
                ec_readstate();
                for (i = 1; i<=ec_slavecount ; i++) {
                    if (ec_slave[i].state != EC_STATE_OPERATIONAL) {
                        printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                            i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                    }
                }
            }

            printf("\nRequest init state for all slaves\n");
            ec_slave[0].state = EC_STATE_INIT;

            ec_writestate(0);
        } else {
            ROS_ERROR("No slaves found!");
        }

        ec_close();
    } else {
        ROS_ERROR("No socket connection on %s", ifname.c_str());
        ROS_ERROR("Execute as root");
    }
}

void ecat_check() {
    int slave;

    while (ros::ok()) {
        if (inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate)) {
            if (needlf) {
                needlf = FALSE;
                printf("\n");
            }

            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();
            for (slave = 1; slave <= ec_slavecount; slave++) {
                if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL)) {
                    ec_group[currentgroup].docheckstate = TRUE;
                    if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR)) {
                        ROS_ERROR("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.", slave);
                        ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                        ec_writestate(slave);
                    } else if (ec_slave[slave].state == EC_STATE_SAFE_OP) {
                        ROS_WARN("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.", slave);
                        ec_slave[slave].state = EC_STATE_OPERATIONAL;
                        ec_writestate(slave);
                    } else if (ec_slave[slave].state > EC_STATE_NONE) {
                        if (ec_reconfig_slave(slave, EC_TIMEOUTMON)) {
                            ec_slave[slave].islost = FALSE;
                            printf("MESSAGE : slave %d reconfigured\n", slave);
                        }
                    } else if (!ec_slave[slave].islost) {
                        ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                        if (ec_slave[slave].state == EC_STATE_NONE) {
                            ec_slave[slave].islost = TRUE;
                            ROS_ERROR("ERROR : slave %d lost", slave);
                        }
                    }
                }
                if (ec_slave[slave].islost) {
                    if (ec_slave[slave].state == EC_STATE_NONE) {
                        if (ec_recover_slave(slave, EC_TIMEOUTMON)) {
                            ec_slave[slave].islost = FALSE;
                            printf("MESSAGE : slave %d recovered\n", slave);
                        }
                    } else {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d found\n", slave);
                    }
                }
            }

            if (!ec_group[currentgroup].docheckstate) {
                printf("OK : all slaves resumed OPERATIONAL.\n");
            }
        }

        ecatMasterStateMsg.ts = ros::Time::now().toSec();
        ecatMasterStateMsg.inOP = inOP;
        ecatMasterStateMsg.needlf = needlf;
        ecatMasterStateMsg.wkc = wkc;
        ecatMasterStateMsg.expectedWKC = expectedWKC;
        ecatMasterStateMsg.currentgroup = currentgroup;
        ecatMasterStateMsg.docheckstate = ec_group[currentgroup].docheckstate;
        if (ecatMasterStateMsg.init == TRUE) {
            for (slave=0; slave<=ec_slavecount; slave++) {
                switch (slave) {
                    case IDX_ECAT_MASTER:
                        ecatMasterStateMsg.slave.master.state = ec_slave[slave].state;
                        ecatMasterStateMsg.slave.master.islost = ec_slave[slave].islost;
                        ecatMasterStateMsg.slave.master.ibytes = ec_slave[slave].Ibytes;
                        ecatMasterStateMsg.slave.master.obytes = ec_slave[slave].Obytes;
                        break;
                    default:
                        ecatMasterStateMsg.slave.slaves[slave-IDX_ECAT_SLV_START].state = ec_slave[slave].state;
                        ecatMasterStateMsg.slave.slaves[slave-IDX_ECAT_SLV_START].islost = ec_slave[slave].islost;
                        ecatMasterStateMsg.slave.slaves[slave-IDX_ECAT_SLV_START].ibytes = ec_slave[slave].Ibytes;
                        ecatMasterStateMsg.slave.slaves[slave-IDX_ECAT_SLV_START].obytes = ec_slave[slave].Obytes;
                        break;
                }
            }
        }

        osal_usleep(10000);
    }
}

void ecat_init() {
    pEcatSlaveInfosMsg = std::vector<ecat::EcatSlaveInfo*>(IDX_ECAT_SLV_START+motor_num);
    for (int i=0; i<ecatSlaveCommandsMsg.slaves.size(); i++) {
        pEcatSlaveInfosMsg[IDX_ECAT_SLV_START+i] = &(ecatSlaveInfosMsg.slaves[i]);
    }

    pEcatSlaveCommandsMsg = std::vector<ecat::EcatSlaveCommand*>(IDX_ECAT_SLV_START+motor_num);
    for (int i=0; i<ecatSlaveCommandsMsg.slaves.size(); i++) {
        pEcatSlaveCommandsMsg[IDX_ECAT_SLV_START+i] = &(ecatSlaveCommandsMsg.slaves[i]);
    }

    txPdoValuePre = std::vector<TxPdoValue>(IDX_ECAT_SLV_START+motor_num);
    txPdoValue = std::vector<TxPdoValue>(IDX_ECAT_SLV_START+motor_num);
    rxPdoValue = std::vector<RxPdoValue>(IDX_ECAT_SLV_START+motor_num);
    sdoData = std::vector<SdoData>(IDX_ECAT_SLV_START+motor_num);
    ecatSlaveCmdTypePre = std::vector<EcatSlaveCmdType>(IDX_ECAT_SLV_START+motor_num);
    ecatSlaveCmdType = std::vector<EcatSlaveCmdType>(IDX_ECAT_SLV_START+motor_num);
    subFsm = std::vector<uint32_t>(IDX_ECAT_SLV_START+motor_num);
    subFsmPP = std::vector<uint32_t>(IDX_ECAT_SLV_START+motor_num);
    error_count = std::vector<uint32_t>(IDX_ECAT_SLV_START+motor_num);
    retry_count = std::vector<uint32_t>(IDX_ECAT_SLV_START+motor_num);
    ecatSlaveCommandsMsg_callbacked = std::vector<uint32_t>(IDX_ECAT_SLV_START+motor_num);
    ecatSlaveCommandsMsg_enabled = std::vector<uint32_t>(IDX_ECAT_SLV_START+motor_num);
    motorInited = std::vector<uint32_t>(IDX_ECAT_SLV_START+motor_num);
    drive_mode = std::vector<uint8_t>(IDX_ECAT_SLV_START+motor_num);
    wroteControlword = std::vector<uint16_t>(IDX_ECAT_SLV_START+motor_num);
}

void printStatusword(uint16_t statusword, bool en_print) {
    #define STATUSWORD_BIT0_RDY_SW_ON 0
    #define STATUSWORD_BIT1_SW_ON 1
    #define STATUSWORD_BIT2_OP_EN 2
    #define STATUSWORD_BIT3_FAULT 3
    #define STATUSWORD_BIT4_VOL_EN 4
    #define STATUSWORD_BIT5_Q_STOP 5
    #define STATUSWORD_BIT6_SW_DIS 6
    #define STATUSWORD_BIT7_WARN 7
    #define STATUSWORD_BIT8_RES8 8
    #define STATUSWORD_BIT9_REMOTE 9
    #define STATUSWORD_BIT10_OMS10 10
    #define STATUSWORD_BIT11_INT_LIM 11
    #define STATUSWORD_BIT12_OMS12 12
    #define STATUSWORD_BIT13_OMS13 13
    #define STATUSWORD_BIT14_ABS 14
    #define STATUSWORD_BIT15_RES15 15
    #define STATUSWORD_BIT_LEN (STATUSWORD_BIT15_RES15+1)
    const static char* STATUSWORD_FUNC[] = {
        "RDY_SW_ON", "SW_ON", "OP_EN", "FAULT", "VOL_EN", "Q_STOP", "SW_DIS", "WARN", "RES8", "REMOTE", "OMS10", "INT_LIM", "OMS12", "OMS13", "ABS", "RES15"
    };

    if (en_print) {
        PRINT_DETAIL_LOG(PRINT_DETAIL_LOG_EN, __LINE__);
        printf("statusword: 0x%04x\n", statusword);
        printf("statusword list\n");
        for (int i=0; i<STATUSWORD_BIT_LEN; i++) {
            if ((statusword >> i) & 0x01) {
                printf("%s, ", STATUSWORD_FUNC[i]);
            }
        }
        printf("\n");
    }
}

void printControlword(uint16_t controlword, bool en_print) {
    #define CONTROLWORD_BIT0_SW_ON 0
    #define CONTROLWORD_BIT1_EN_VOL 1
    #define CONTROLWORD_BIT2_Q_STOP 2
    #define CONTROLWORD_BIT3_EN_OP 3
    #define CONTROLWORD_BIT4_OMS4 4
    #define CONTROLWORD_BIT5_OMS5 5
    #define CONTROLWORD_BIT6_OMS6 6
    #define CONTROLWORD_BIT7_FAULT_RST 7
    #define CONTROLWORD_BIT8_HALT 8
    #define CONTROLWORD_BIT9_OMS9 9
    #define CONTROLWORD_BIT10_RES10 10
    #define CONTROLWORD_BIT11_RES11 11
    #define CONTROLWORD_BIT12_RES12 12
    #define CONTROLWORD_BIT13_RES13 13
    #define CONTROLWORD_BIT14_RES14 14
    #define CONTROLWORD_BIT15_RES15 15
    #define CONTROLWORD_BIT_LEN (CONTROLWORD_BIT15_RES15+1)
    const static char* CONTROLWORD_FUNC[] = {
        "SW_ON", "EN_VOL", "Q_STOP", "EN_OP", "OMS4", "OMS5", "OMS6", "FAULT_RST", "HALT", "OMS9", "RES10", "RES11", "RES12", "RES13", "RES14", "RES15"
    };

    if (en_print) {
        PRINT_DETAIL_LOG(PRINT_DETAIL_LOG_EN, __LINE__);
        printf("controlword: 0x%04x\n", controlword);
        printf("controlword list\n");
        for (int i=0; i<CONTROLWORD_BIT_LEN; i++) {
            if ((controlword >> i) & 0x01) {
                printf("%s, ", CONTROLWORD_FUNC[i]);
            }
        }
        printf("\n");
    }
}

static ec_ODlistt ODlist;
static ec_OElistt OElist;

#define OTYPE_VAR 0x0007
#define OTYPE_ARRAY 0x0008
#define OTYPE_RECORD 0x0009

#define ATYPE_Rpre 0x01
#define ATYPE_Rsafe 0x02
#define ATYPE_Rop 0x04
#define ATYPE_Wpre 0x08
#define ATYPE_Wsafe 0x10
#define ATYPE_Wop 0x20

char* dtype2string(uint16 dtype, uint16 bitlen) {
    static char str[32] = {0, };

    switch (dtype) {
        case ECT_BOOLEAN:
            sprintf(str, "BOOLEAN");
            break;
        case ECT_INTEGER8:
            sprintf(str, "INTEGER8");
            break;
        case ECT_INTEGER16:
            sprintf(str, "INTEGER16");
            break;
        case ECT_INTEGER32:
            sprintf(str, "INTEGER32");
            break;
        case ECT_INTEGER24:
            sprintf(str, "INTEGER24");
            break;
        case ECT_INTEGER64:
            sprintf(str, "INTEGER64");
            break;
        case ECT_UNSIGNED8:
            sprintf(str, "UNSIGNED8");
            break;
        case ECT_UNSIGNED16:
            sprintf(str, "UNSIGNED16");
            break;
        case ECT_UNSIGNED32:
            sprintf(str, "UNSIGNED32");
            break;
        case ECT_UNSIGNED24:
            sprintf(str, "UNSIGNED24");
            break;
        case ECT_UNSIGNED64:
            sprintf(str, "UNSIGNED64");
            break;
        case ECT_REAL32:
            sprintf(str, "REAL32");
            break;
        case ECT_REAL64:
            sprintf(str, "REAL64");
            break;
        case ECT_BIT1:
            sprintf(str, "BIT1");
            break;
        case ECT_BIT2:
            sprintf(str, "BIT2");
            break;
        case ECT_BIT3:
            sprintf(str, "BIT3");
            break;
        case ECT_BIT4:
            sprintf(str, "BIT4");
            break;
        case ECT_BIT5:
            sprintf(str, "BIT5");
            break;
        case ECT_BIT6:
            sprintf(str, "BIT6");
            break;
        case ECT_BIT7:
            sprintf(str, "BIT7");
            break;
        case ECT_BIT8:
            sprintf(str, "BIT8");
            break;
        case ECT_VISIBLE_STRING:
            sprintf(str, "VISIBLE_STR(%d)", bitlen);
            break;
        case ECT_OCTET_STRING:
            sprintf(str, "OCTET_STR(%d)", bitlen);
            break;
        default:
            sprintf(str, "dt:0x%4.4X (%d)", dtype, bitlen);
    }

    return str;
}

char* otype2string(uint16 otype) {
    static char str[32] = {0, };

    switch (otype) {
        case OTYPE_VAR:
            sprintf(str, "VAR");
            break;
        case OTYPE_ARRAY:
            sprintf(str, "ARRAY");
            break;
        case OTYPE_RECORD:
            sprintf(str, "RECORD");
            break;
        default:
            sprintf(str, "ot:0x%4.4X", otype);
    }

    return str;
}

char* access2string(uint16 access) {
    static char str[32] = {0, };

    sprintf(str, "%s%s%s%s%s%s",
        ((access & ATYPE_Rpre) != 0 ? "R" : "_"),
        ((access & ATYPE_Wpre) != 0 ? "W" : "_"),
        ((access & ATYPE_Rsafe) != 0 ? "R" : "_"),
        ((access & ATYPE_Wsafe) != 0 ? "W" : "_"),
        ((access & ATYPE_Rop) != 0 ? "R" : "_"),
        ((access & ATYPE_Wop) != 0 ? "W" : "_"));

    return str;
}

char* SDO2string(uint16 slave, uint16 index, uint8 subidx, uint16 dtype) {
    static char usdo[128];
    int l = sizeof(usdo) - 1;
    int i;
    uint8* u8;
    int8* i8;
    uint16* u16;
    int16* i16;
    uint32* u32;
    int32* i32;
    uint64* u64;
    int64* i64;
    float* sr;
    double* dr;
    char es[32];

    memset(&usdo, 0, 128);
    ec_SDOread(slave, index, subidx, FALSE, &l, &usdo, EC_TIMEOUTRXM);

    if (EcatError) {
        return ec_elist2string();
    } else {
        static char str[64] = {0, };

        switch (dtype) {
            case ECT_BOOLEAN:
                u8 = (uint8*)&usdo[0];
                if (*u8) sprintf(str, "TRUE");
                else sprintf(str, "FALSE");
                break;
            case ECT_INTEGER8:
                i8 = (int8*)&usdo[0];
                sprintf(str, "0x%2.2x / %d", *i8, *i8);
                break;
            case ECT_INTEGER16:
                i16 = (int16*)&usdo[0];
                sprintf(str, "0x%4.4x / %d", *i16, *i16);
                break;
            case ECT_INTEGER32:
            case ECT_INTEGER24:
                i32 = (int32*)&usdo[0];
                sprintf(str, "0x%8.8x / %d", *i32, *i32);
                break;
            case ECT_INTEGER64:
                i64 = (int64*)&usdo[0];
                sprintf(str, "0x%16.16" PRIx64 " / %" PRId64, *i64, *i64);
                break;
            case ECT_UNSIGNED8:
                u8 = (uint8*)&usdo[0];
                sprintf(str, "0x%2.2x / %u", *u8, *u8);
                break;
            case ECT_UNSIGNED16:
                u16 = (uint16*)&usdo[0];
                sprintf(str, "0x%4.4x / %u", *u16, *u16);
                break;
            case ECT_UNSIGNED32:
            case ECT_UNSIGNED24:
                u32 = (uint32*)&usdo[0];
                sprintf(str, "0x%8.8x / %u", *u32, *u32);
                break;
            case ECT_UNSIGNED64:
                u64 = (uint64*)&usdo[0];
                sprintf(str, "0x%16.16" PRIx64" / %" PRIu64, *u64, *u64);
                break;
            case ECT_REAL32:
                sr = (float*)&usdo[0];
                sprintf(str, "%f", *sr);
                break;
            case ECT_REAL64:
                dr = (double*)&usdo[0];
                sprintf(str, "%f", *dr);
                break;
            case ECT_BIT1:
            case ECT_BIT2:
            case ECT_BIT3:
            case ECT_BIT4:
            case ECT_BIT5:
            case ECT_BIT6:
            case ECT_BIT7:
            case ECT_BIT8:
                u8 = (uint8*)&usdo[0];
                sprintf(str, "0x%x / %u", *u8, *u8);
                break;
            case ECT_VISIBLE_STRING:
                strcpy(str, "\"");
                strcat(str, usdo);
                strcat(str, "\"");
                break;
            case ECT_OCTET_STRING:
                str[0] = 0x00;
                for (i = 0; i < l; i++) {
                    sprintf(es, "0x%2.2x ", usdo[i]);
                    strcat(str, es);
                }
                break;
            default:
                sprintf(str, "Unknown type");
        }

        return str;
    }
}

void sdo_list(int slave_num) {
    int index_i, j;

    ODlist.Entries = 0;
    memset(&ODlist, 0, sizeof(ODlist));
    if (ec_readODlist(slave_num, &ODlist)) {
        printf("CoE Object Description found, %d entries.\n", ODlist.Entries);
        for (index_i=0; index_i<ODlist.Entries; index_i++) {
            uint8_t max_sub;
            char name[128] = {0, };

            ec_readODdescription(index_i, &ODlist);
            while (EcatError) printf(" - %s\n", ec_elist2string());
            snprintf(name, sizeof(name) - 1, "\"%s\"", ODlist.Name[index_i]);
            if (ODlist.ObjectCode[index_i] == OTYPE_VAR) {
                printf("0x%04x, %-40s, [%s]\n",
                    ODlist.Index[index_i], name, otype2string(ODlist.ObjectCode[index_i]));
            } else {
                printf("0x%04x, %-40s, [%-6s  maxsub(0x%02x/%02d)]\n",
                    ODlist.Index[index_i], name, otype2string(ODlist.ObjectCode[index_i]),
                    ODlist.MaxSub[index_i], ODlist.MaxSub[index_i]);
            }
            memset(&OElist, 0, sizeof(OElist));
            ec_readOE(index_i, &ODlist, &OElist);
            while (EcatError) printf("- %s\n", ec_elist2string());

            if (ODlist.ObjectCode[index_i] != OTYPE_VAR) {
                int l = sizeof(max_sub);
                ec_SDOread(slave_num, ODlist.Index[index_i], 0, FALSE, &l, &max_sub, EC_TIMEOUTRXM);
            } else {
                max_sub = ODlist.MaxSub[index_i];
            }

            for (j=0; j<max_sub+1; j++) {
                if ((OElist.DataType[j] > 0) && (OElist.BitLength[j] > 0)) {
                    snprintf(name, sizeof(name) - 1, "\"%s\"", OElist.Name[j]);
                    printf("  0x%02x, %-40s, [%-16s %6s], ", j, name,
                        dtype2string(OElist.DataType[j], OElist.BitLength[j]),
                        access2string(OElist.ObjAccess[j]));
                    if ((OElist.ObjAccess[j] & 0x0007)) {
                        printf("%s", SDO2string(slave_num, ODlist.Index[index_i], j, OElist.DataType[j]));
                    }
                    printf("\n");
                }
            }
        }
    } else {
        while (EcatError) printf("%s", ec_elist2string());
    }
}