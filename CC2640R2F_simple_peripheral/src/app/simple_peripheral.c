/******************************************************************************

 @file       simple_peripheral.c

 @brief This file contains the Simple Peripheral sample application for use
        with the CC2650 Bluetooth Low Energy Protocol Stack.

 Group: CMCU, SCS
 Target Device: CC2640R2

 ******************************************************************************
 
 Copyright (c) 2013-2017, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 Release Name: simplelink_cc2640r2_sdk_1_40_00_45
 Release Date: 2017-07-20 17:16:59
 *****************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/display/Display.h>

#if defined( USE_FPGA ) || defined( DEBUG_SW_TRACE )
#include <driverlib/ioc.h>
#endif // USE_FPGA | DEBUG_SW_TRACE

#include <icall.h>
#include "util.h"
/* This Header file contains all BLE API and icall structure definition */
#include "icall_ble_api.h"

#include "devinfoservice.h"
#include "simple_gatt_profile.h"

#if defined(FEATURE_OAD) || defined(IMAGE_INVALIDATE)
#include "oad_target.h"
#include "oad.h"
#endif //FEATURE_OAD || IMAGE_INVALIDATE

#include "peripheral.h"

#ifdef USE_RCOSC
#include "rcosc_calibration.h"
#endif //USE_RCOSC


#include "board.h"

#if !defined(Display_DISABLE_ALL)
#include "board_key.h"
#include <menu/two_btn_menu.h>

#include "simple_peripheral_menu.h"
#endif  // !Display_DISABLE_ALL

#include "simple_peripheral.h"

#include "bmp280.h"



/*********************************************************************
 * CONSTANTS
 */

// Advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160

// General discoverable mode: advertise indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

#ifndef FEATURE_OAD
// Minimum connection interval (units of 1.25ms, 80=100ms) for automatic
// parameter update request
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     80

// Maximum connection interval (units of 1.25ms, 800=1000ms) for automatic
// parameter update request
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     800

#else // FEATURE_OAD
// Increase the the connection interval to allow for higher throughput for OAD

// Minimum connection interval (units of 1.25ms, 8=10ms) for automatic
// parameter update request
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     8

// Maximum connection interval (units of 1.25ms, 8=10ms) for automatic
// parameter update request
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     8
#endif // FEATURE_OAD

// Slave latency to use for automatic parameter update request
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) for automatic parameter
// update request
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000

// After the connection is formed, the peripheral waits until the central
// device asks for its preferred connection parameters
#define DEFAULT_ENABLE_UPDATE_REQUEST         GAPROLE_LINK_PARAM_UPDATE_WAIT_REMOTE_PARAMS

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         6

// How often to perform periodic event (in msec)
#define SBP_PERIODIC_EVT_PERIOD               5000

// Application specific event ID for HCI Connection Event End Events
#define SBP_HCI_CONN_EVT_END_EVT              0x0001

// Type of Display to open
#if !defined(Display_DISABLE_ALL)
  #if defined(BOARD_DISPLAY_USE_LCD) && (BOARD_DISPLAY_USE_LCD!=0)
    #define SBP_DISPLAY_TYPE Display_Type_LCD
  #elif defined (BOARD_DISPLAY_USE_UART) && (BOARD_DISPLAY_USE_UART!=0)
    #define SBP_DISPLAY_TYPE Display_Type_UART
  #else // !BOARD_DISPLAY_USE_LCD && !BOARD_DISPLAY_USE_UART
    #define SBP_DISPLAY_TYPE 0 // Option not supported
  #endif // BOARD_DISPLAY_USE_LCD && BOARD_DISPLAY_USE_UART
#else // BOARD_DISPLAY_USE_LCD && BOARD_DISPLAY_USE_UART
  #define SBP_DISPLAY_TYPE 0 // No Display
#endif // !Display_DISABLE_ALL

#ifdef FEATURE_OAD
// The size of an OAD packet.
#define OAD_PACKET_SIZE                       ((OAD_BLOCK_SIZE) + 2)
#endif // FEATURE_OAD

// Task configuration
#define SBP_TASK_PRIORITY                     1

#ifndef SBP_TASK_STACK_SIZE
#define SBP_TASK_STACK_SIZE                   644
#endif

// Application events
#define SBP_STATE_CHANGE_EVT                  0x0001
#define SBP_CHAR_CHANGE_EVT                   0x0002
#define SBP_KEY_CHANGE_EVT                    0x0004

// Internal Events for RTOS application
#define SBP_ICALL_EVT                         ICALL_MSG_EVENT_ID // Event_Id_31
#define SBP_QUEUE_EVT                         UTIL_QUEUE_EVENT_ID // Event_Id_30
#define SBP_PERIODIC_EVT                      Event_Id_00

#ifdef FEATURE_OAD
// Additional Application Events for OAD
#define SBP_QUEUE_PING_EVT                    Event_Id_01

// Bitwise OR of all events to pend on with OAD
#define SBP_ALL_EVENTS                        (SBP_ICALL_EVT        | \
                                               SBP_QUEUE_EVT        | \
                                               SBP_PERIODIC_EVT     | \
                                               SBP_QUEUE_PING_EVT)
#else
// Bitwise OR of all events to pend on
#define SBP_ALL_EVENTS                        (SBP_ICALL_EVT        | \
                                               SBP_QUEUE_EVT        | \
                                               SBP_PERIODIC_EVT)
#endif /* FEATURE_OAD */

// Row numbers for two-button menu
#define SBP_ROW_RESULT        (0)
#define SBP_ROW_BDADDR        (1)
#define SBP_ROW_ROLESTATE     (2)
#define SBP_ROW_STATUS_1      (3)
#define SBP_ROW_STATUS_2      (3)

/*********************************************************************
 * TYPEDEFS
 */

// App event passed from profiles.
typedef struct
{
  appEvtHdr_t hdr;  // event header.
} sbpEvt_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Display Interface
Display_Handle dispHandle = NULL;

/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Event globally used to post local events and pend on system and
// local events.
static ICall_SyncHandle syncEvent;

// Clock instances for internal periodic events.
static Clock_Struct periodicClock;

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

#if defined(FEATURE_OAD)
// Event data from OAD profile.
static Queue_Struct oadQ;
static Queue_Handle hOadQ;
#endif //FEATURE_OAD

// Task configuration
Task_Struct sbpTask;
Char sbpTaskStack[SBP_TASK_STACK_SIZE];

// 扫描回应 (max size = 31 bytes)
static uint8_t scanRspData[] =
{
  0x14,   // 长度为0x14
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,         // 表示后面紧跟的数据为蓝牙名称
  'S',
  'i',
  'm',
  'p',
  'l',
  'e',
  'B',
  'L',
  'E',
  'P',
  'e',
  'r',
  'i',
  'p',
  'h',
  'e',
  'r',
  'a',
  'l',

  0x05,   // 长度为5
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,           // 连接参数
  LO_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),   // 100ms
  HI_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),
  LO_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),   // 1s
  HI_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),

  0x02,   // 长度为2
  GAP_ADTYPE_POWER_LEVEL,                         // 发射功率
  0       // 0dBm
};

// Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertising)

/* 广播内容 */
static uint8_t advertData[] =
{
  0x02,                      // 字段长度为2
  GAP_ADTYPE_FLAGS,          // 表示后面紧跟的数据为广播类型
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

#if !defined(FEATURE_OAD) || defined(FEATURE_OAD_ONCHIP)
  0x03,   // 长度为3
#else    
  0x05, 
#endif //FEATURE_OAD
  GAP_ADTYPE_16BIT_MORE,      // 表示后面紧跟的数据为UUID
#ifdef FEATURE_OAD
  LO_UINT16(OAD_SERVICE_UUID),
  HI_UINT16(OAD_SERVICE_UUID),
#endif //FEATURE_OAD
#ifndef FEATURE_OAD_ONCHIP
  LO_UINT16(SIMPLEPROFILE_SERV_UUID),
  HI_UINT16(SIMPLEPROFILE_SERV_UUID)
#endif //FEATURE_OAD_ONCHIP
};

// GAP GATT Attributes
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "Simple Peripheral";

// Globals used for ATT Response retransmission
static gattMsgEvent_t *pAttRsp = NULL;
static uint8_t rspTxRetry = 0;
static uint8_t ownAddress[B_ADDR_LEN];

//float pressure;
//float temperature; 
//float asl;
static char tempi=0;

float pressure;
float temperature; 
float asl;  
UART_Handle uart;
UART_Params uartParams;
char  input;
const char  echoPrompt[] = "Echoing characters:\r\n";
const char  success[] = "success\r\n";
const char  failure[] = "failure\r\n";
const char  pre[] = "pre:";
const char  tempe[] = "tempe:";
const char  high[] = "high:";
PIN_Config ledPinTable[] = {
    Board_PIN_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_PIN_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW  | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    IOID_23        | PIN_INPUT_EN  | PIN_PULLUP | PIN_HYSTERESIS,
    PIN_TERMINATE
};
static PIN_Handle ledPinHandle;
static PIN_State ledPinState;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void SimpleBLEPeripheral_init( void );
static void SimpleBLEPeripheral_taskFxn(UArg a0, UArg a1);

static uint8_t SimpleBLEPeripheral_processStackMsg(ICall_Hdr *pMsg);
static uint8_t SimpleBLEPeripheral_processGATTMsg(gattMsgEvent_t *pMsg);
static void SimpleBLEPeripheral_processAppMsg(sbpEvt_t *pMsg);
static void SimpleBLEPeripheral_processStateChangeEvt(gaprole_States_t newState);
static void SimpleBLEPeripheral_processCharValueChangeEvt(uint8_t paramID);
static void SimpleBLEPeripheral_performPeriodicTask(void);
static void SimpleBLEPeripheral_clockHandler(UArg arg);

static void SimpleBLEPeripheral_sendAttRsp(void);
static void SimpleBLEPeripheral_freeAttRsp(uint8_t status);

static void SimpleBLEPeripheral_stateChangeCB(gaprole_States_t newState);
#ifndef FEATURE_OAD_ONCHIP
static void SimpleBLEPeripheral_charValueChangeCB(uint8_t paramID);
#endif //!FEATURE_OAD_ONCHIP
static void SimpleBLEPeripheral_enqueueMsg(uint8_t event, uint8_t state);

#ifdef FEATURE_OAD
void SimpleBLEPeripheral_processOadWriteCB(uint8_t event, uint16_t connHandle,
                                           uint8_t *pData);
#endif //FEATURE_OAD

#if !defined(Display_DISABLE_ALL)
void SimpleBLEPeripheral_keyChangeHandler(uint8 keys);
static void SimpleBLEPeripheral_handleKeys(uint8_t keys);
#endif  // !Display_DISABLE_ALL

/*********************************************************************
 * EXTERN FUNCTIONS
 */
extern void AssertHandler(uint8 assertCause, uint8 assertSubcause);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// Peripheral GAPRole Callbacks
static gapRolesCBs_t SimpleBLEPeripheral_gapRoleCBs =
{
  SimpleBLEPeripheral_stateChangeCB     // GAPRole State Change Callbacks
};

// GAP Bond Manager Callbacks
// These are set to NULL since they are not needed. The application
// is set up to only perform justworks pairing.
static gapBondCBs_t simpleBLEPeripheral_BondMgrCBs =
{
  NULL, // Passcode callback
  NULL  // Pairing / Bonding state Callback
};

// Simple GATT Profile Callbacks
#ifndef FEATURE_OAD_ONCHIP
static simpleProfileCBs_t SimpleBLEPeripheral_simpleProfileCBs =
{
  SimpleBLEPeripheral_charValueChangeCB // Simple GATT Characteristic value change callback
};
#endif //!FEATURE_OAD_ONCHIP

#ifdef FEATURE_OAD
static oadTargetCBs_t simpleBLEPeripheral_oadCBs =
{
  SimpleBLEPeripheral_processOadWriteCB // OAD Profile Characteristic value change callback.
};
#endif //FEATURE_OAD

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleBLEPeripheral_createTask
 *
 * @brief   Task creation function for the Simple Peripheral.
 *
 * @param   None.
 *
 * @return  None.
 */

void SimpleBLEPeripheral_createTask(void)
{
  Task_Params taskParams;

  /* 初始化任务参数 */
  Task_Params_init(&taskParams);
  taskParams.stack = sbpTaskStack;
  taskParams.stackSize = SBP_TASK_STACK_SIZE;
  taskParams.priority = SBP_TASK_PRIORITY;

  /* 创建任务 */
  Task_construct(&sbpTask, SimpleBLEPeripheral_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_init
 *
 * @brief   Called during initialization and contains application
 *          specific initialization (ie. hardware initialization/setup,
 *          table initialization, power up notification, etc), and
 *          profile initialization/setup.
 *
 * @param   None.
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_init(void)
{
  /* 注册当前线程到ICall，以便APP任务可以与ICall任务进行收发消息 */
  ICall_registerApp(&selfEntity, &syncEvent);

#ifdef USE_RCOSC
  RCOSC_enableCalibration();
#endif // USE_RCOSC

#if defined( USE_FPGA )
  // configure RF Core SMI Data Link
  IOCPortConfigureSet(IOID_12, IOC_PORT_RFC_GPO0, IOC_STD_OUTPUT);
  IOCPortConfigureSet(IOID_11, IOC_PORT_RFC_GPI0, IOC_STD_INPUT);

  // configure RF Core SMI Command Link
  IOCPortConfigureSet(IOID_10, IOC_IOCFG0_PORT_ID_RFC_SMI_CL_OUT, IOC_STD_OUTPUT);
  IOCPortConfigureSet(IOID_9, IOC_IOCFG0_PORT_ID_RFC_SMI_CL_IN, IOC_STD_INPUT);

  // configure RF Core tracer IO
  IOCPortConfigureSet(IOID_8, IOC_PORT_RFC_TRC, IOC_STD_OUTPUT);
#else // !USE_FPGA
  #if defined( DEBUG_SW_TRACE )
    // configure RF Core tracer IO
    IOCPortConfigureSet(IOID_8, IOC_PORT_RFC_TRC, IOC_STD_OUTPUT | IOC_CURRENT_4MA | IOC_SLEW_ENABLE);
  #endif // DEBUG_SW_TRACE
#endif // USE_FPGA

  /* 创建RTOS消息队列以便接收profile发送过来的消息*/
  appMsgQueue = Util_constructQueue(&appMsg);

  /* 为周期性事件创建一次性的定时 */
  Util_constructClock(&periodicClock, SimpleBLEPeripheral_clockHandler,
                      SBP_PERIODIC_EVT_PERIOD, 0, false, SBP_PERIODIC_EVT);
  
  Display_print0(dispHandle, 0, 0, "*BLE Peripheral");
//  {  
//      UART_init();   
//      UART_Params_init(&uartParams);
//      uartParams.writeDataMode = UART_DATA_BINARY;
//      uartParams.readDataMode = UART_DATA_BINARY;
//      uartParams.readReturnMode = UART_RETURN_FULL;
//      uartParams.readEcho = UART_ECHO_OFF;
//      uartParams.baudRate = 115200;
//      uart = UART_open(Board_UART0, &uartParams);
//      if (uart == NULL) {
//          /* UART_open() 打开失败时掉到此处死循环 */
//          while (1);
//      }
//      UART_write(uart, echoPrompt, sizeof(echoPrompt));
//      
//      ledPinHandle = PIN_open(&ledPinState, ledPinTable);
//      if(!ledPinHandle) {
//          /* LED打开错误时掉到此处死循环 */
//          while(1)
//          {
//            UART_write(uart, echoPrompt, sizeof(echoPrompt));
//          }
//      }
//
//      if(bmp280Init()==true)
//      {
//          UART_write(uart, success, sizeof(success));
//      }
//      else
//      {
//          UART_write(uart, failure, sizeof(failure));
//      }
//  }
  
  

  /* 初始化显示模块，如OLED和UART部分 */
  dispHandle = Display_open(SBP_DISPLAY_TYPE, NULL);

  /* 设置连接建立后到从机发送更新参数请求的延迟时间，单位是秒,需要注意的是，如果连接参数更新模式为
     GAPROLE_LINK_PARAM_UPDATE_INITIATE_BOTH_PARAMS或GAPROLE_LINK_PARAM_UPDATE_INITIATE_APP_PARAMS
     则该语句不起作用 */
  GAP_SetParamValue(TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL);

  
  {
    uint8_t initialAdvertEnable = TRUE;

    /* 设置为0，则表示广播30.72秒后将自动停止广播 */
    uint16_t advertOffTime = 0;

    uint8_t enableUpdateRequest = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16_t desiredMinInterval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16_t desiredMaxInterval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16_t desiredSlaveLatency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16_t desiredConnTimeout = DEFAULT_DESIRED_CONN_TIMEOUT;

    /* 设置广播使能 */
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                         &initialAdvertEnable);
    /* 设置广播广播时间，设置为0，则表示广播30.72秒后将自动停止广播 */
    GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16_t),
                         &advertOffTime);
    /* 设置扫描回应参数 */
    GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData),
                         scanRspData);
    /* 设置广播内容 */
    GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData);

    /* 设置连接参数更新类型 */
    GAPRole_SetParameter(GAPROLE_PARAM_UPDATE_ENABLE, sizeof(uint8_t),
                         &enableUpdateRequest);
    /* 设置最小连接间隙 */
    GAPRole_SetParameter(GAPROLE_MIN_CONN_INTERVAL, sizeof(uint16_t),
                         &desiredMinInterval);
    /* 设置最大连接间隙 */
    GAPRole_SetParameter(GAPROLE_MAX_CONN_INTERVAL, sizeof(uint16_t),
                         &desiredMaxInterval);
    /* 从机潜伏期 */
    GAPRole_SetParameter(GAPROLE_SLAVE_LATENCY, sizeof(uint16_t),
                         &desiredSlaveLatency);
    /* 设置监督时间 */
    GAPRole_SetParameter(GAPROLE_TIMEOUT_MULTIPLIER, sizeof(uint16_t),
                         &desiredConnTimeout);
  }

  /* 设置设备名称参数 */
  GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);


  {
    /* 默认广播间隙为DEFAULT_ADVERTISING_INTERVAL */
    uint16_t advInt = DEFAULT_ADVERTISING_INTERVAL;

    /* 设置广播间隙参数 */
    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt);
  }


  {
    uint32_t passkey = 0;   // 配对PIN码 "000000"
    uint8_t pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;  // 等待对方发起配对请求
    uint8_t mitm = TRUE;    // 需要配对
    uint8_t ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;  // IO能力为只有显示屏
    uint8_t bonding = TRUE; // 需要绑定

    /* 设置绑定管理器参数 */
    GAPBondMgr_SetParameter(GAPBOND_DEFAULT_PASSCODE, sizeof(uint32_t), &passkey);
    GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);
    GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
    GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
    GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);
  }

  /* 添加相关服务 */
  GGS_AddService(GATT_ALL_SERVICES);           // GAP GATT服务
  GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT服务
  DevInfo_AddService();                        // 设备信息服务

#ifndef FEATURE_OAD_ONCHIP
  SimpleProfile_AddService(GATT_ALL_SERVICES); // Simple GATT Profile
#endif //!FEATURE_OAD_ONCHIP

#ifdef FEATURE_OAD
  VOID OAD_addService();                       // OAD Profile
  OAD_register((oadTargetCBs_t *)&simpleBLEPeripheral_oadCBs);
  hOadQ = Util_constructQueue(&oadQ);
#endif //FEATURE_OAD

#ifdef IMAGE_INVALIDATE
  Reset_addService();
#endif //IMAGE_INVALIDATE


#ifndef FEATURE_OAD_ONCHIP

  /* 设置特征值参数 */
  {
    uint8_t charValue1 = 1;
    uint8_t charValue2 = 2;
    uint8_t charValue3 = 3;
    uint8_t charValue4 = 4;
    uint8_t charValue5[SIMPLEPROFILE_CHAR5_LEN] = { 1, 2, 3, 4, 5 };

    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR1, sizeof(uint8_t),
                               &charValue1);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR2, sizeof(uint8_t),
                               &charValue2);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR3, sizeof(uint8_t),
                               &charValue3);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4, sizeof(uint8_t),
                               &charValue4);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR5, SIMPLEPROFILE_CHAR5_LEN,
                               charValue5);
  }

  /* 注册simpleProfile回调函数 */
  SimpleProfile_RegisterAppCBs(&SimpleBLEPeripheral_simpleProfileCBs);
#endif //!FEATURE_OAD_ONCHIP

  /* 注册绑定器回调函数 */
  VOID GAPBondMgr_Register(&simpleBLEPeripheral_BondMgrCBs);
  
  /* 注册当前线程到GAP以接收HCI/HOST的消息 */
  GAP_RegisterForMsgs(selfEntity);
  
  /* 注册当前线程到GATT以接收GATT相关事件和ATT响应 */
  GATT_RegisterForMsgs(selfEntity);

  // 下面这项仅在开取数据扩展功能才需要配置
  {
    //Set initial values to maximum, RX is set to max. by default(251 octets, 2120us)
    #define APP_SUGGESTED_PDU_SIZE 251 //default is 27 octets(TX)
    #define APP_SUGGESTED_TX_TIME 2120 //default is 328us(TX)
  }

#if defined (BLE_V42_FEATURES) && (BLE_V42_FEATURES & PRIVACY_1_2_CFG)
  // 初始化GATT客户端
  GATT_InitClient();

  /* 取消RPAO特征，注意，如果希望设备通过私有模式跟其他的蓝牙5设备通信，则必须注释掉本语句 */
  GGS_SetParamValue(GGS_DISABLE_RPAO_CHARACTERISTIC);
#endif // BLE_V42_FEATURES & PRIVACY_1_2_CFG

#if !defined(Display_DISABLE_ALL)
  // Set the title of the main menu
  #if defined FEATURE_OAD
    #if defined (HAL_IMAGE_A)
      TBM_SET_TITLE(&sbpMenuMain, "BLE Peripheral A");
    #else
      TBM_SET_TITLE(&sbpMenuMain, "BLE Peripheral B");
    #endif // HAL_IMAGE_A
  #else
    TBM_SET_TITLE(&sbpMenuMain, "BLE Peripheral");
  #endif // FEATURE_OAD

  // 初始化菜单模块，该部分不需要用到，所以注释掉
  //tbm_setItemStatus(&sbpMenuMain, TBM_ITEM_NONE, TBM_ITEM_ALL);
  //tbm_initTwoBtnMenu(dispHandle, &sbpMenuMain, 3, NULL);

  // 按键初始化
  Board_initKeys(SimpleBLEPeripheral_keyChangeHandler);
#endif  // !Display_DISABLE_ALL

#if !defined (USE_LL_CONN_PARAM_UPDATE)
  /* 获取BLE 特性，获取结果将触发HCI_LE_READ_LOCAL_SUPPORTED_FEATURES事件，
  然后通过在在main任务的循环处理中处理该事件以获取BLE特性 */
  HCI_LE_ReadLocalSupportedFeaturesCmd();
#endif // !defined (USE_LL_CONN_PARAM_UPDATE)

  // 启动设备
  VOID GAPRole_StartDevice(&SimpleBLEPeripheral_gapRoleCBs);
  
 
  
  


}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_taskFxn
 *
 * @brief   Application task entry point for the Simple Peripheral.
 *
 * @param   a0, a1 - not used.
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_taskFxn(UArg a0, UArg a1)
{
  /* APP任务初始化 */
  SimpleBLEPeripheral_init();

  // APP主循环
  for (;;)
  {
    uint32_t events;
//    bmp280GetData(&pressure, &temperature, &asl);

    /* 获取事件 */
    events = Event_pend(syncEvent, Event_Id_NONE, SBP_ALL_EVENTS,
                        ICALL_TIMEOUT_FOREVER);

    if (events)
    {
      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;

      /* 获取stack协议栈消息进行处理 */
      if (ICall_fetchServiceMsg(&src, &dest,
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
      {
        uint8 safeToDealloc = TRUE;

        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          ICall_Stack_Event *pEvt = (ICall_Stack_Event *)pMsg;

          // Check for BLE stack events first
          if (pEvt->signature == 0xffff)
          {
            if (pEvt->event_flag & SBP_HCI_CONN_EVT_END_EVT)
            {
              // Try to retransmit pending ATT Response (if any)
              SimpleBLEPeripheral_sendAttRsp();
            }
          }
          else
          {
            // Process inter-task message
            safeToDealloc = SimpleBLEPeripheral_processStackMsg((ICall_Hdr *)pMsg);
          }
        }

        if (pMsg && safeToDealloc)
        {
          ICall_freeMsg(pMsg);
        }
      }

      /* 处理RTOS队列消息 */
      if (events & SBP_QUEUE_EVT)
      {
        while (!Queue_empty(appMsgQueue))
        {
          sbpEvt_t *pMsg = (sbpEvt_t *)Util_dequeueMsg(appMsgQueue);
          if (pMsg)
          {
            // Process message.
            SimpleBLEPeripheral_processAppMsg(pMsg);

            // Free the space from the message.
            ICall_free(pMsg);
          }
        }
      }

      /* 处理周期性事件 */
      if (events & SBP_PERIODIC_EVT)
      {
        Util_startClock(&periodicClock);

        // Perform periodic application task
        SimpleBLEPeripheral_performPeriodicTask();
      }

#ifdef FEATURE_OAD
      if (events & SBP_QUEUE_PING_EVT)
      {
        while (!Queue_empty(hOadQ))
        {
          oadTargetWrite_t *oadWriteEvt = Queue_get(hOadQ);

          // Identify new image.
          if (oadWriteEvt->event == OAD_WRITE_IDENTIFY_REQ)
          {
            OAD_imgIdentifyWrite(oadWriteEvt->connHandle, oadWriteEvt->pData);
          }
          // Write a next block request.
          else if (oadWriteEvt->event == OAD_WRITE_BLOCK_REQ)
          {
            OAD_imgBlockWrite(oadWriteEvt->connHandle, oadWriteEvt->pData);
          }

          // Free buffer.
          ICall_free(oadWriteEvt);
        }
      }
#endif //FEATURE_OAD
    }
  }
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_processStackMsg
 *
 * @brief   Process an incoming stack message.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t SimpleBLEPeripheral_processStackMsg(ICall_Hdr *pMsg)
{
  uint8_t safeToDealloc = TRUE;

  switch (pMsg->event)
  {
    // 处理GATT消息事件
    case GATT_MSG_EVENT:
      // Process GATT message
      safeToDealloc = SimpleBLEPeripheral_processGATTMsg((gattMsgEvent_t *)pMsg);
      break;

    // 处理GAP事件
    case HCI_GAP_EVENT_EVENT:
      {

        // Process HCI message
        switch(pMsg->status)
        {
          case HCI_COMMAND_COMPLETE_EVENT_CODE:
            // Process HCI Command Complete Event
            {

#if !defined (USE_LL_CONN_PARAM_UPDATE)
              // This code will disable the use of the LL_CONNECTION_PARAM_REQ
              // control procedure (for connection parameter updates, the
              // L2CAP Connection Parameter Update procedure will be used
              // instead). To re-enable the LL_CONNECTION_PARAM_REQ control
              // procedures, define the symbol USE_LL_CONN_PARAM_UPDATE
              // The L2CAP Connection Parameter Update procedure is used to
              // support a delta between the minimum and maximum connection
              // intervals required by some iOS devices.

              // Parse Command Complete Event for opcode and status
              hciEvt_CmdComplete_t* command_complete = (hciEvt_CmdComplete_t*) pMsg;
              uint8_t   pktStatus = command_complete->pReturnParam[0];

              //find which command this command complete is for
              switch (command_complete->cmdOpcode)
              {
                case HCI_LE_READ_LOCAL_SUPPORTED_FEATURES:
                  {
                    if (pktStatus == SUCCESS)
                    {
                      uint8_t featSet[8];

                      // Get current feature set from received event (bits 1-9
                      // of the returned data
                      memcpy( featSet, &command_complete->pReturnParam[1], 8 );

                      // Clear bit 1 of byte 0 of feature set to disable LL
                      // Connection Parameter Updates
                      CLR_FEATURE_FLAG( featSet[0], LL_FEATURE_CONN_PARAMS_REQ );

                      // Update controller with modified features
                      HCI_EXT_SetLocalSupportedFeaturesCmd( featSet );
                    }
                  }
                  break;

                default:
                  //do nothing
                  break;
              }
#endif // !defined (USE_LL_CONN_PARAM_UPDATE)

            }
            break;

          // 处理硬件错误事件
          case HCI_BLE_HARDWARE_ERROR_EVENT_CODE:
            AssertHandler(HAL_ASSERT_CAUSE_HARDWARE_ERROR,0);
            break;

          // LE Events
          case HCI_LE_EVENT_CODE:
            {
              hciEvt_BLEPhyUpdateComplete_t *pPUC
                = (hciEvt_BLEPhyUpdateComplete_t*) pMsg;

              // A Phy Update Has Completed or Failed
              if (pPUC->BLEEventCode == HCI_BLE_PHY_UPDATE_COMPLETE_EVENT)
              {
                if (pPUC->status != SUCCESS)
                {
                  Display_print0(dispHandle, SBP_ROW_STATUS_1, 0,
                                 "PHY Change failure");
                }
                else
                {
                  Display_print0(dispHandle, SBP_ROW_STATUS_1, 0,
                                 "PHY Update Complete");
                  // Only symmetrical PHY is supported.
                  // rxPhy should be equal to txPhy.
                  Display_print1(dispHandle, SBP_ROW_STATUS_2, 0,
                                 "Current PHY: %s",
                                 (pPUC->rxPhy == HCI_PHY_1_MBPS) ? "1 Mbps" :

// Note: BLE_V50_FEATURES is always defined and long range phy (PHY_LR_CFG) is
//       defined in build_config.opt
#if (BLE_V50_FEATURES & PHY_LR_CFG)
                                   ((pPUC->rxPhy == HCI_PHY_2_MBPS) ? "2 Mbps" :
                                       "Coded:S2"));
#else  // !PHY_LR_CFG
                                   "2 Mbps");
#endif // PHY_LR_CFG
                }
              }
            }
            break;

          default:
            break;
        }
      }
      break;

      default:
        // do nothing
        break;

    }

  return (safeToDealloc);
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t SimpleBLEPeripheral_processGATTMsg(gattMsgEvent_t *pMsg)
{
  // See if GATT server was unable to transmit an ATT response
  if (pMsg->hdr.status == blePending)
  {
    // No HCI buffer was available. Let's try to retransmit the response
    // on the next connection event.
    if (HCI_EXT_ConnEventNoticeCmd(pMsg->connHandle, selfEntity,
                                   SBP_HCI_CONN_EVT_END_EVT) == SUCCESS)
    {
      // First free any pending response
      SimpleBLEPeripheral_freeAttRsp(FAILURE);

      // Hold on to the response message for retransmission
      pAttRsp = pMsg;

      // Don't free the response message yet
      return (FALSE);
    }
  }
  else if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
  {
    // ATT request-response or indication-confirmation flow control is
    // violated. All subsequent ATT requests or indications will be dropped.
    // The app is informed in case it wants to drop the connection.

    // Display the opcode of the message that caused the violation.
    Display_print1(dispHandle, SBP_ROW_ROLESTATE, 0, "FC Violated: %d", pMsg->msg.flowCtrlEvt.opcode);
  }
  else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
  {
    // MTU size updated
    Display_print1(dispHandle, SBP_ROW_ROLESTATE, 0, "MTU Size: %d", pMsg->msg.mtuEvt.MTU);
  }

  // Free message payload. Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);

  // It's safe to free the incoming message
  return (TRUE);
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_sendAttRsp
 *
 * @brief   Send a pending ATT response message.
 *
 * @param   none
 *
 * @return  none
 */
static void SimpleBLEPeripheral_sendAttRsp(void)
{
  // See if there's a pending ATT Response to be transmitted
  if (pAttRsp != NULL)
  {
    uint8_t status;

    // Increment retransmission count
    rspTxRetry++;

    // Try to retransmit ATT response till either we're successful or
    // the ATT Client times out (after 30s) and drops the connection.
    status = GATT_SendRsp(pAttRsp->connHandle, pAttRsp->method, &(pAttRsp->msg));
    if ((status != blePending) && (status != MSG_BUFFER_NOT_AVAIL))
    {
      // Disable connection event end notice
      HCI_EXT_ConnEventNoticeCmd(pAttRsp->connHandle, selfEntity, 0);

      // We're done with the response message
      SimpleBLEPeripheral_freeAttRsp(status);
    }
    else
    {
      // Continue retrying
      Display_print1(dispHandle, SBP_ROW_STATUS_1, 0, "Rsp send retry: %d", rspTxRetry);
    }
  }
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_freeAttRsp
 *
 * @brief   Free ATT response message.
 *
 * @param   status - response transmit status
 *
 * @return  none
 */
static void SimpleBLEPeripheral_freeAttRsp(uint8_t status)
{
  // See if there's a pending ATT response message
  if (pAttRsp != NULL)
  {
    // See if the response was sent out successfully
    if (status == SUCCESS)
    {
      Display_print1(dispHandle, SBP_ROW_STATUS_1, 0, "Rsp sent retry: %d", rspTxRetry);
    }
    else
    {
      // Free response payload
      GATT_bm_free(&pAttRsp->msg, pAttRsp->method);

      Display_print1(dispHandle, SBP_ROW_STATUS_1, 0, "Rsp retry failed: %d", rspTxRetry);
    }

    // Free response message
    ICall_freeMsg(pAttRsp);

    // Reset our globals
    pAttRsp = NULL;
    rspTxRetry = 0;
  }
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_processAppMsg
 *
 * @brief   Process an incoming callback from a profile.
 *
 * @param   pMsg - message to process
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_processAppMsg(sbpEvt_t *pMsg)
{
  switch (pMsg->hdr.event)
  {
    case SBP_STATE_CHANGE_EVT:
      SimpleBLEPeripheral_processStateChangeEvt((gaprole_States_t)pMsg->
                                                hdr.state);
      break;

    case SBP_CHAR_CHANGE_EVT:
      SimpleBLEPeripheral_processCharValueChangeEvt(pMsg->hdr.state);
      break;

#if !defined(Display_DISABLE_ALL)
    case SBP_KEY_CHANGE_EVT:
      SimpleBLEPeripheral_handleKeys(pMsg->hdr.state);
      break;
#endif  // !Display_DISABLE_ALL

    default:
      // Do nothing.
      break;
  }
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_stateChangeCB
 *
 * @brief   Callback from GAP Role indicating a role state change.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_stateChangeCB(gaprole_States_t newState)
{
  SimpleBLEPeripheral_enqueueMsg(SBP_STATE_CHANGE_EVT, newState);
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_processStateChangeEvt
 *
 * @brief   Process a pending GAP Role state change event.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_processStateChangeEvt(gaprole_States_t newState)
{
#ifdef PLUS_BROADCASTER
  static bool firstConnFlag = false;
#endif // PLUS_BROADCASTER

  switch ( newState )
  {
    // 初始化完成，开始工作
    case GAPROLE_STARTED:
      {
        uint8_t systemId[DEVINFO_SYSTEM_ID_LEN];

        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);

        // 使用设备地址来产生系统ID
        systemId[0] = ownAddress[0];
        systemId[1] = ownAddress[1];
        systemId[2] = ownAddress[2];

        systemId[4] = 0x00;
        systemId[3] = 0x00;

        systemId[7] = ownAddress[5];
        systemId[6] = ownAddress[4];
        systemId[5] = ownAddress[3];

        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);

        // 打印设备地址
        Display_print0(dispHandle, SBP_ROW_BDADDR, 0, Util_convertBdAddr2Str(ownAddress));
        Display_print0(dispHandle, SBP_ROW_ROLESTATE, 0, "Initialized");
      }
      break;

    // 开始广播
    case GAPROLE_ADVERTISING:
      Display_print0(dispHandle, 0, 0, "*BLE Peripheral");
      Display_print0(dispHandle, SBP_ROW_BDADDR, 0, Util_convertBdAddr2Str(ownAddress));
      Display_print0(dispHandle, SBP_ROW_ROLESTATE, 0, "Advertising...");
      break;

#ifdef PLUS_BROADCASTER
    /* 当连接被断开后，不可连接广播将被转成可连接中广播 */
    case GAPROLE_ADVERTISING_NONCONN:
      {
        // 关闭不可连接广播
        uint8_t advertEnabled = FALSE;

        GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t),
                           &advertEnabled);

        // 打开可连接广播
        advertEnabled = TRUE;

        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                             &advertEnabled);

        // 设置标志
        firstConnFlag = false;

        SimpleBLEPeripheral_freeAttRsp(bleNotConnected);
      }
      break;
#endif //PLUS_BROADCASTER

    // 完成链接
    case GAPROLE_CONNECTED:
      {
        linkDBInfo_t linkInfo;
        uint8_t numActive = 0;

        Util_startClock(&periodicClock);

        numActive = linkDB_NumActive();

        // 获取连接信息
        if ( linkDB_GetInfo( numActive - 1, &linkInfo ) == SUCCESS )
        {
          Display_print1(dispHandle, SBP_ROW_ROLESTATE, 0, "Num Conns: %d", (uint16_t)numActive);
          Display_print0(dispHandle, SBP_ROW_STATUS_1, 0, Util_convertBdAddr2Str(linkInfo.addr));
        }
        else
        {
          uint8_t peerAddress[B_ADDR_LEN];

          GAPRole_GetParameter(GAPROLE_CONN_BD_ADDR, peerAddress);

          Display_print0(dispHandle, SBP_ROW_ROLESTATE, 0, "Connected");
          Display_print0(dispHandle, SBP_ROW_STATUS_1, 0, Util_convertBdAddr2Str(peerAddress));
        }

#if !defined(Display_DISABLE_ALL)
        tbm_setItemStatus(&sbpMenuMain, TBM_ITEM_ALL, TBM_ITEM_NONE);
#endif
        #ifdef PLUS_BROADCASTER

          if (firstConnFlag == false)
          {
            // 广播可连接广播
            uint8_t advertEnabled = FALSE; 

            GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                                 &advertEnabled);

            // 打开不可连接广播
            advertEnabled = TRUE;
            
            GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t),
                                 &advertEnabled);
            firstConnFlag = true;
          }
        #endif
      }
      break;

    // 连接中广播
    case GAPROLE_CONNECTED_ADV:
      Display_print0(dispHandle, SBP_ROW_ROLESTATE, 0, "ConAdvertising");
      break;

    // 断开连接
    case GAPROLE_WAITING:
      Util_stopClock(&periodicClock);
      SimpleBLEPeripheral_freeAttRsp(bleNotConnected);

      Display_print0(dispHandle, SBP_ROW_ROLESTATE, 0, "Disconnected");

#if !defined(Display_DISABLE_ALL)
      
      tbm_setItemStatus(&sbpMenuMain, TBM_ITEM_NONE, TBM_ITEM_ALL);
#endif

      Display_clearLines(dispHandle, SBP_ROW_RESULT, SBP_ROW_STATUS_2);
      break;

    // 超时处理
    case GAPROLE_WAITING_AFTER_TIMEOUT:
      SimpleBLEPeripheral_freeAttRsp(bleNotConnected);

      Display_print0(dispHandle, SBP_ROW_RESULT, 0, "Timed Out");

#if !defined(Display_DISABLE_ALL)
      tbm_setItemStatus(&sbpMenuMain, TBM_ITEM_NONE, TBM_ITEM_ALL);
#endif

      Display_clearLines(dispHandle, SBP_ROW_STATUS_1, SBP_ROW_STATUS_2);

      #ifdef PLUS_BROADCASTER
        firstConnFlag = false;
      #endif
      break;

    // 错误处理
    case GAPROLE_ERROR:
      Display_print0(dispHandle, SBP_ROW_RESULT, 0, "Error");
      break;

    default:
      Display_clearLines(dispHandle, SBP_ROW_RESULT, SBP_ROW_STATUS_2);
      break;
  }
}

#ifndef FEATURE_OAD_ONCHIP
/*********************************************************************
 * @fn      SimpleBLEPeripheral_charValueChangeCB
 *
 * @brief   Callback from Simple Profile indicating a characteristic
 *          value change.
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_charValueChangeCB(uint8_t paramID)
{
  /* 添加到消息队列 */
  SimpleBLEPeripheral_enqueueMsg(SBP_CHAR_CHANGE_EVT, paramID);
}
#endif //!FEATURE_OAD_ONCHIP

/*********************************************************************
 * @fn      SimpleBLEPeripheral_processCharValueChangeEvt
 *
 * @brief   Process a pending Simple Profile characteristic value change
 *          event.
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_processCharValueChangeEvt(uint8_t paramID)
{
#ifndef FEATURE_OAD_ONCHIP
  uint8_t newValue;

  switch(paramID)
  {
    case SIMPLEPROFILE_CHAR1:
      SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR1, &newValue);

      Display_print1(dispHandle, SBP_ROW_STATUS_1, 0, "Char 1: %d", (uint16_t)newValue);
      break;

    case SIMPLEPROFILE_CHAR3:
//      bmp280GetData(&pressure, &temperature, &asl);
      newValue=temperature;
      SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR3, &newValue);

      Display_print1(dispHandle, SBP_ROW_STATUS_1, 0, "Char 3: %d", (uint16_t)newValue);
      break;

    default:
      // should not reach here!
      break;
  }
#endif //!FEATURE_OAD_ONCHIP
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_performPeriodicTask
 *
 * @brief   Perform a periodic application task. This function gets called
 *          every five seconds (SBP_PERIODIC_EVT_PERIOD). In this example,
 *          the value of the third characteristic in the SimpleGATTProfile
 *          service is retrieved from the profile, and then copied into the
 *          value of the the fourth characteristic.
 *
 * @param   None.
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_performPeriodicTask(void)
{
#ifndef FEATURE_OAD_ONCHIP
  uint8_t valueToCopy;
  uint8_t data[6];
  int32_t temp;
  uint32_t press;

  // Call to retrieve the value of the third characteristic in the profile
//  if (SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR3, &valueToCopy) == SUCCESS)
//  {
//    // Call to set that value of the fourth characteristic in the profile.
//    // Note that if notifications of the fourth characteristic have been
//    // enabled by a GATT client device, then a notification will be sent
//    // every time this function is called.
//    
//   SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4, sizeof(uint8_t),
//                               &valueToCopy);
//  }
  GPIO_toggle(Board_GPIO_LED1);
  tempi++;
  SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4, sizeof(uint8_t),
                               &tempi);
//    SensorBmp280_read(data);
//    SensorBmp280_convert(data, &temp, &press);
//    
//    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4, sizeof(uint32_t),
//                                &press);
    
#endif //!FEATURE_OAD_ONCHIP
}


#ifdef FEATURE_OAD
/*********************************************************************
 * @fn      SimpleBLEPeripheral_processOadWriteCB
 *
 * @brief   Process a write request to the OAD profile.
 *
 * @param   event      - event type:
 *                       OAD_WRITE_IDENTIFY_REQ
 *                       OAD_WRITE_BLOCK_REQ
 * @param   connHandle - the connection Handle this request is from.
 * @param   pData      - pointer to data for processing and/or storing.
 *
 * @return  None.
 */
void SimpleBLEPeripheral_processOadWriteCB(uint8_t event, uint16_t connHandle,
                                           uint8_t *pData)
{
  oadTargetWrite_t *oadWriteEvt = ICall_malloc( sizeof(oadTargetWrite_t) + \
                                             sizeof(uint8_t) * OAD_PACKET_SIZE);

  if ( oadWriteEvt != NULL )
  {
    oadWriteEvt->event = event;
    oadWriteEvt->connHandle = connHandle;

    oadWriteEvt->pData = (uint8_t *)(&oadWriteEvt->pData + 1);
    memcpy(oadWriteEvt->pData, pData, OAD_PACKET_SIZE);

    Queue_put(hOadQ, (Queue_Elem *)oadWriteEvt);

    // Post the application's event.  For OAD, no event flag is used.
    Event_post(syncEvent, SBP_QUEUE_PING_EVT);
  }
  else
  {
    // Fail silently.
  }
}
#endif //FEATURE_OAD

/*********************************************************************
 * @fn      SimpleBLEPeripheral_clockHandler
 *
 * @brief   Handler function for clock timeouts.
 *
 * @param   arg - event type
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_clockHandler(UArg arg)
{
  // Wake up the application.
  Event_post(syncEvent, arg);
}

#if !defined(Display_DISABLE_ALL)
/*********************************************************************
 * @fn      SimpleBLEPeripheral_keyChangeHandler
 *
 * @brief   Key event handler function
 *
 * @param   keys - bitmap of pressed keys
 *
 * @return  none
 */
void SimpleBLEPeripheral_keyChangeHandler(uint8 keys)
{
  SimpleBLEPeripheral_enqueueMsg(SBP_KEY_CHANGE_EVT, keys);
}
#endif  // !Display_DISABLE_ALL

/*********************************************************************
 * @fn      SimpleBLEPeripheral_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_enqueueMsg(uint8_t event, uint8_t state)
{
  sbpEvt_t *pMsg;

  // Create dynamic pointer to message.
  if ((pMsg = ICall_malloc(sizeof(sbpEvt_t))))
  {
    pMsg->hdr.event = event;
    pMsg->hdr.state = state;

    // Enqueue the message.
    Util_enqueueMsg(appMsgQueue, syncEvent, (uint8*)pMsg);
  }
}

#if !defined(Display_DISABLE_ALL)
/*********************************************************************
 * @fn      SimpleBLEPeripheral_handleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   keys - bit field for key events. Valid entries:
 *                 KEY_LEFT
 *                 KEY_RIGHT
 *
 * @return  none
 */
static void SimpleBLEPeripheral_handleKeys(uint8_t keys)
{
  if (keys & KEY_LEFT)
  {
    // Check if the key is still pressed. WA for possible bouncing.
#if defined(CC2650DK_7ID)
    if (PIN_getInputValue(Board_KEY_LEFT) == 0)
#elif defined(CC2650_LAUNCHXL) || defined(CC2640R2_LAUNCHXL)
    if (PIN_getInputValue(Board_PIN_BUTTON0) == 0)
#endif // CC2650DK_7ID, CC2650_LAUNCHXL, CC2640R2_LAUNCHXL
    {
      tbm_buttonLeft();
    }
  }
  else if (keys & KEY_RIGHT)
  {
    // Check if the key is still pressed. WA for possible bouncing.
#if defined(CC2650DK_7ID)
    if (PIN_getInputValue(Board_KEY_RIGHT) == 0)
#elif defined(CC2650_LAUNCHXL) || defined(CC2640R2_LAUNCHXL)
    if (PIN_getInputValue(Board_PIN_BUTTON1) == 0)
#endif // CC2650DK_7ID, CC2650_LAUNCHXL, CC2640R2_LAUNCHXL
    {
      tbm_buttonRight();
    }
  }
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_doSetPhy
 *
 * @brief   Set PHY preference.
 *
 * @param   index - 0: 1M PHY
 *                  1: 2M PHY
 *                  2: 1M + 2M PHY
 *                  3: CODED PHY (Long range) (when PHY_LR_CFG is defined)
 *                  4: 1M + 2M + CODED PHY (when PHY_LR_CFG is defined)
 *
 * @return  always true
 */
bool SimpleBLEPeripheral_doSetPhy(uint8 index)
{
  uint8_t gapRoleState;
  uint16_t connectionHandle;
  static uint8_t phy[] = {
    HCI_PHY_1_MBPS, HCI_PHY_2_MBPS, HCI_PHY_1_MBPS | HCI_PHY_2_MBPS,

  // Note: BLE_V50_FEATURES is always defined and long range phy (PHY_LR_CFG) is
  //       defined in build_config.opt
  // To use the long range phy, HCI_PHY_CODED needs to be included
  #if (BLE_V50_FEATURES & PHY_LR_CFG)
    HCI_PHY_CODED, HCI_PHY_1_MBPS | HCI_PHY_2_MBPS | HCI_PHY_CODED,
  #endif  // PHY_LR_CFG
  };

  GAPRole_GetParameter(GAPROLE_STATE, &gapRoleState);
  GAPRole_GetParameter(GAPROLE_CONNHANDLE, &connectionHandle);

  // Set Phy Preference on the current connection. Apply the same value
  // for RX and TX.
  HCI_LE_SetPhyCmd(connectionHandle, 0, phy[index], phy[index], 0);

  Display_print1(dispHandle, SBP_ROW_RESULT, 0, "PHY preference: %s",
                 TBM_GET_ACTION_DESC(&sbpMenuMain, index));

  Display_clearLine(dispHandle, SBP_ROW_STATUS_1);

  return true;
}
#endif  // !Display_DISABLE_ALL

/*********************************************************************
*********************************************************************/
