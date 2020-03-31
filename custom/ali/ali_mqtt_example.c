/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of Guangzhou Yoglink Imformation Techology Co.,Ltd 2019
*
*****************************************************************************/
/*****************************************************************************
 *
 * Filename:
 * ---------
 *   ali_mqtt_example.c
 *
 * Project:
 * --------
 *   OpenCPU
 *
 * Description:
 * ------------
 *   This example demonstrates how to use MQTT function with APIs in OpenCPU to connect ali iot cloud.
 *
 * Usage:
 * ------
 *   Compile & Run:
 *
 *     Set "C_PREDEF=-D __ALI_MQTT_EXAMPLE__" in gcc_makefile file. And compile the
 *     app using "make clean/new".
 *     Download image bin to module to run.
 * 
 * note:
 *     add run_through function.
 *============================================================================
 *             HISTORY
 *----------------------------------------------------------------------------
 * 
 ****************************************************************************/
#ifdef __ALI_MQTT_EXAMPLE__
#include "custom_feature_def.h"
#include "ql_stdlib.h"
#include "ql_common.h"
#include "ql_type.h"
#include "ql_trace.h"
#include "ql_error.h"
#include "ql_uart.h"
#include "ql_timer.h"
#include "ql_socket.h"
#include "ril_network.h"
#include "ril_mqtt.h"
#include "ril.h"
#include "ril_util.h"
#include "ril_system.h"
#include "ql_fs.h"

#include "dev_sign_api.h"
#include "data_process.h"

#include "cJSON.h"


#define DEBUG_ENABLE 1
#if DEBUG_ENABLE > 0
#define DEBUG_PORT  UART_PORT1
#define DBG_BUF_LEN   1024
static char DBG_BUFFER[DBG_BUF_LEN];
#define APP_DEBUG(FORMAT,...) {\
    Ql_memset(DBG_BUFFER, 0, DBG_BUF_LEN);\
    Ql_sprintf(DBG_BUFFER,FORMAT,##__VA_ARGS__); \
    if (UART_PORT2 == (DEBUG_PORT)) \
    {\
        Ql_Debug_Trace(DBG_BUFFER);\
    } else {\
        Ql_UART_Write((Enum_SerialPort)(DEBUG_PORT), (u8*)(DBG_BUFFER), Ql_strlen((const char *)(DBG_BUFFER)));\
    }\
}
#else
#define APP_DEBUG(FORMAT,...) 
#endif

/****************************************************************************
* Definition for APN
****************************************************************************/
#define APN      "CMNET\0"
#define USERID   ""
#define PASSWD   ""

/*****************************************************************
* pre-defined topic define and payload format
******************************************************************/
#define PROPERTY_POST_TOPIC         "/sys/%s/%s/thing/event/property/post"    // format: /sys/{productKey}/{deviceName}/thing/event/property/post
#define EVENT_POST_TOPIC            "/sys/%s/%s/thing/event/%s/post"          // format: /sys/{productKey}/{deviceName}/thing/event/{eventname}/post
#define PROPERTY_SET_TOPIC          "/sys/%s/%s/thing/service/property/set"   // format: /sys/{productKey}/{deviceName}/thing/service/property/set
#define DEV_INFO_UPDATE_TOPIC       "/sys/%s/%s/thing/deviceinfo/update"      // format: /sys/{productKey}/{deviceName}/thing/deviceinfo/update
#define FIRMWARE_VERSION_TOPIC      "/ota/device/inform/%s/%s"                // format: /ota/device/inform/{productKey}/{deviceName}
#define OTA_UPGRADE_SUB_TOPIC       "/ota/device/upgrade/%s/%s"               // format: /ota/device/upgrade/{productKey}/{deviceName}
#define OTA_PROGRESS_PUB_TOPIC      "/ota/device/progress/%s/%s"              // format: /ota/device/progress/{productKey}/{deviceName}


#define PROPERTY_METHOD             "thing.event.property.post"
#define EVENT_METHOD                "thing.event.post"

#define PROPERTY_PAYLOAD_FMT        "{'id':'%d','version':'1.0.0','params':%s,'method':'%s'}"
#define FIRMWARE_VERSION_MSG_FMT    "{'id':'%d','params':{'version':'%s'}}"

const char *DEVICE_INFO_UPDATE_FMT = "{'id':'%d','version':'1.0.0','params':["
                                     "{'attrKey':'SYS_LP_SDK_VERSION','attrValue':'%s','domain':'SYSTEM'},"
                                     "{'attrKey':'SYS_SDK_LANGUAGE','attrValue':'C','domain':'SYSTEM'}"
									 "{\"attrKey\":\"SYS_MODULE_ID\",\"attrValue\":\"M25\",\"domain\":\"SYSTEM\"},"
                                     "{\"attrKey\":\"SYS_PARTNER_ID\",\"attrValue\":\"Quectel\",\"domain\":\"SYSTEM\"}"
                                     "],'method':'thing.deviceinfo.update'}";

/*****************************************************************
* command define
******************************************************************/
#define CMD_PROPERTY_HEAD               "AT+SENDJS=property"
#define CMD_EVENT_HEAD                  "AT+SENDJS=event"
#define CMD_DEV_INFO_SET_HEAD           "AT+PRODEVICE="
#define CMD_DEV_INFO_GET_HEAD           "AT+PRODEVICE?"
#define CMD_PRODUCT_SECRET_SET_HEAD     "AT+PROSECRET="
#define CMD_PRODUCT_SECRET_GET_HEAD     "AT+PROSECRET?"
#define CMD_NET_STATUS_GET_HEAD         "AT+ILOPCONNECT"
#define CMD_APP_VERSION_GET_HEAD        "AT+APPVER"
#define CMD_RESET_HEAD                  "AT+RESET"
#define CMD_RUN_THROUGH_MODE			"AT+SENDPASS"
#define AT_CMD_END_FLAG                 "\r\n"

#define APP_FIRMWARE_VERSION            "1.0.0"

/*****************************************************************
* response define
******************************************************************/
#define OK_ASK                          "+ok\r\n"
#define FAIL_ASK                        "+error\r\n"

#define NET_DISCONNECT_ASK              "+ok=NET_DISCONNECT\r\n"
#define NET_CONNECT_ASK                 "+ok=NET_CONNECT\r\n"
#define SERVER_DISCONNECT_ASK           "+ok=SERVER_DISCONNECT\r\n"
#define SERVER_CONNECT_ASK              "+ok=SERVER_CONNECT\r\n"

#define NET_DISCONNECT_RESP             "+ILOPCONNECT=NET_DISCONNECT\r\n"
#define NET_CONNECT_RESP                "+ILOPCONNECT=NET_CONNECT\r\n"
#define SERVER_DISCONNECT_RESP          "+ILOPCONNECT=SERVER_DISCONNECT\r\n"
#define SERVER_CONNECT_RESP             "+ILOPCONNECT=SERVER_CONNECT\r\n"

/*****************************************************************
* filesystem define
******************************************************************/
#define PATH_ROOT    		            ((u8 *)"myroot")
#define DEV_INFO_DIR                    "dev_info"
#define DEV_INFO_FILE                   "devinfo.txt"
#define PRODUCT_SECRET_FILE             "prosecret.txt"
#define FILE_CONTENT_LEN                256
#define RUN_THROUGH_FLAG_DIR            "run_through_flag"
#define RUN_THROUGH_FLAG_FILE			"run_through_flag.txt"
/*****************************************************************
* MQTT topic define
******************************************************************/
typedef enum {
	PROPERTY_SET,
	SERVICE_DOWN,
	OTA_UPGRADE,
	TOPIC_MAX_ID,
	RAW_DOWN_ID,
} MQTT_TOPIC_ID;

/*****************************************************************
* command type define
******************************************************************/
typedef enum {
	CMD_PROPERTY_POST,
	CMD_EVENT_POST,
	CMD_DEV_INFO_SET,
	CMD_DEV_INFO_GET,
	CMD_PRODUCT_SECRET_SET,
	CMD_PRODUCT_SECRET_GET,
	CMD_NET_STATUS_GET,
	CMD_APP_VERSION_GET,
	CMD_RESET,
	CMD_INVALID
} Enum_CMD_TYPE;


/*****************************************************************
* define process state
******************************************************************/
typedef enum{
    STATE_NW_QUERY_STATE,
    STATE_MQTT_CFG,
    STATE_MQTT_OPEN,
    STATE_MQTT_CONN,
    STATE_MQTT_SUB,
    STATE_MQTT_PUB,
    STATE_MQTT_TUNS,
    STATE_MQTT_CLOSE,
    STATE_MQTT_DISC,
    STATE_TOTAL_NUM
}Enum_ONENETSTATE;
static u8 m_mqtt_state = STATE_NW_QUERY_STATE;

#define MSG_ID_USER_DATA                (MSG_ID_USER_START+0x100)

// Define the UART port and the receive data buffer
#define SERIAL_RX_BUFFER_LEN  512
static Enum_SerialPort m_myUartPort  = UART_PORT1;
static u8 m_RxBuf_Uart[SERIAL_RX_BUFFER_LEN];
static u8 uart_buff[SERIAL_RX_BUFFER_LEN];
static u32 RxBuff_totalBytes = 0, lastTotalBytes = 0;

u8 m_net_state = STATE_NET_DISCONNECT;


/*****************************************************************
* run_through_mode param
******************************************************************/

//Select the communication mode of UART
#define AT_CMD_MODE   			0
#define RUN_THROUGH_MODE  		1
#define EXIT_RUN_THROUGH_MODE   "+++"
static u8 m2m_comm_mode = AT_CMD_MODE ;



/*****************************************************************
* MQTT  timer param
******************************************************************/
#define MQTT_TIMER_ID           TIMER_ID_USER_START
#define MQTT_TIMER_PERIOD       500

#define MQTT_DISCONN_MAX_TIME   30000 // 30s
#define MQTT_RETRY_MAX_COUNT    10

#define HEARTBEAT_TIMER_ID      (TIMER_ID_USER_START + 1)
#define HEARTBEAT_TIMER_PERIOD  1000

#define WTD_FEED_TIMER_ID       (TIMER_ID_USER_START + 2)
#define WTD_FEED_TIMER_PERIOD   800

#define SIM_CARD_ERROR_MAX_TIME 60000

/*****************************************************************
*  MQTT Param
******************************************************************/
MQTT_Urc_Param_t*    mqtt_urc_param_ptr = NULL;
ST_MQTT_topic_info_t  mqtt_topic_info_t;

#define DEFAULT_KEEPALIVE_TIME    60  // 60 second
#define PKT_TIMEOUT_SECOND        10
#define RETRY_TIMES               1

#if 0
/**
 * MQTT version to connect with: 3.1
 */
#define MQTTVERSION_3_1     0
/**
 * MQTT version to connect with: 3.1.1
 */
#define MQTTVERSION_3_1_1   1
#endif

/*****************************************************************
* GPIO Param
******************************************************************/
Enum_PinName  wtd_feedpin = PINNAME_PCM_OUT;

/*****************************************************************
*  TEST Param
******************************************************************/
u8 connect_id = ConnectID_0;
u16 pub_message_id = 0;
u16 sub_message_id = 0;
static u8 count = 0;

#define PAYLOAD_MAX_LEN     512
#define DEV_INFO_MAX_LEN    (IOTX_PRODUCT_KEY_LEN + IOTX_PRODUCT_SECRET_LEN + IOTX_DEVICE_NAME_LEN + IOTX_DEVICE_SECRET_LEN)

u8 productKey[IOTX_PRODUCT_KEY_LEN + 1];
u8 productSecret[IOTX_PRODUCT_SECRET_LEN + 1];
u8 deviceName[IOTX_DEVICE_NAME_LEN + 1];
u8 deviceSecret[IOTX_DEVICE_SECRET_LEN + 1];

iotx_sign_mqtt_t sign_mqtt;
static unsigned int g_report_id = 0;
static s8 dev_info_flag = 0;

u8 sim_card_status = 1; //ƒ¨»œsimø® «’˝≥£≤Â»Îµƒ
u16 sim_card_error_time = 0;
char strImei[30];
u8 imei_valid_flag = 0;
u8 location_valid_flag = 0;
u8 imei_update_flag = 0;
u8 location_update_flag = 0;

u8 net_status_report = 0;
u16 pdp_open_count = 0;
u8 mqtt_conn_flag = 0;
u16 mqtt_disconn_time = 0;
u8 mqtt_open_retry_count = 0;
u8 mqtt_conn_retry_count = 0;
u8 mqtt_sub_retry_count = 0;
u8 mqtt_pub_retry_count = 0;
u8 mqtt_server_conn = 0;



/*****************************************************************
* uart callback function
******************************************************************/
static void CallBack_UART_Hdlr(Enum_SerialPort port, Enum_UARTEventType msg, bool level, void* customizedPara);

/*****************************************************************
* AT CMD Response function
******************************************************************/
static s32 ATResponse_Handler(char* line, u32 len, void* userData);

/*****************************************************************
* timer callback function
******************************************************************/
static void Callback_Timer(u32 timerId, void* param);
static void callback_WatchDogTimer(u32 timerId, void* param);

/*****************************************************************
* mqtt recv callback function
******************************************************************/
static void callback_mqtt_recv(u8* buffer,u32 length);

/*****************************************************************
* other subroutines
******************************************************************/
static s32 ReadSerialPort(Enum_SerialPort port, /*[in]*/u32 lastLen, /*[out]*/u8* pBuffer, /*[in]*/u32 bufLen);
static void proc_handle(u8 *pData,s32 len);

/* report device information */
int iotx_report_devinfo(u8 tcp_id, u32 msg_id);
/* report Firmware version */
int iotx_report_firmware_version(u8 tcp_id, u32 msg_id);
/* report ModuleID */
int iotx_report_mid(u8 tcp_id, u32 msg_id);
/* get report id value */
int iotx_report_id(void);
/* get firmware version */
int GetFirmwareVersion(char *version);
static void uart_write_data(char port, const char *buf);
int example_publish(u8 tcp_id, u32 msg_id, char data_type, char *event_id, char *payload);
int example_user_post_property(u8 tcp_id, u32 msg_id, char *params_str);
int example_user_post_event(u8 tcp_id, u32 msg_id, char *event_id, char *params_str);

u8 get_mqtt_topic_id(const char *topic_name);
void printf_mqtt_conn_result(u8 code);

/*****************************************************************
* device information function
******************************************************************/
int iotx_report_device_location_info(void);
int iotx_report_device_imei_info(void);

/*****************************************************************
* filesystem function
******************************************************************/
static s32 read_fromfs(/*IN*/const char *filepath,/*OUT*/char *buf, /*OUT*/s32 *len);
static s32 write_tofs(/*IN*/const char *filepath, /*IN*/const char *buf, /*OUT*/s32 *len);
static s32 read_device_info_fromfs(/*OUT*/char *buf, /*OUT*/s32 *len);
static s32 write_device_info_tofs(/*IN*/const char *buf, /*OUT*/s32 *len);
static s32 read_productsecret_fromfs(/*OUT*/char *buf, /*OUT*/s32 *len);
static s32 write_productsecret_tofs(/*IN*/const char *buf, /*OUT*/s32 *len);
static s32 write_run_through_flag_tofs(const char *buf, s32 *len);
static s32 read_run_through_flag_fromfs(const char *buf, s32 *len);



s32 RIL_MQTT_QMTCFG_VERSION(u8 tcp_id, u8 version);
s32 RIL_MQTT_QMTCFG_KEEPALIVE(u8 tcp_id, u16 keep_alive_second);
s32 RIL_MQTT_QMTCFG_TIMEOUT(u8 tcp_id, u8 pkt_timeout, u8 retry_times, u8 timeout_notice_flag);

void proc_main_task(s32 taskId)
{
    ST_MSG msg;
    int i;
    iotx_mqtt_region_types_t region = IOTX_CLOUD_REGION_SHANGHAI;
    iotx_dev_meta_info_t meta;
    u8 buff[DEV_INFO_MAX_LEN+4];
    u8 dev_info[DEV_INFO_MAX_LEN+4];
    char payload[PAYLOAD_MAX_LEN];
    char event_name[32];
    char app_ver[32];
    char *buff_ptr = NULL;
    char buff_array[DEV_INFO_MAX_LEN+4];
    u32 writeLen, readLen;
    s32 ret;
	
    // Register & open UART port
    Ql_UART_Register(m_myUartPort, CallBack_UART_Hdlr, NULL);
    Ql_UART_Open(m_myUartPort, 115200, FC_NONE);

    APP_DEBUG("<--OpenCPU: MQTT Client.-->\r\n");

	//run_through_flag information
	//Ê®°ÂùóÈó¥ÈÄö‰ø°Ê®°ÂºèÊñá‰ª∂ËØªÂèñ‰∏éÊ®°ÂºèËÆæÁΩÆ
	ret = read_run_through_flag_fromfs(buff_array, &readLen);
	if(ret == 0)
	{
		m2m_comm_mode = buff_array[0];
		if(m2m_comm_mode=='0')
		{
			m2m_comm_mode=AT_CMD_MODE;
			APP_DEBUG("<--m2m_comm_mode:AT_CMD_MODE-->\r\n");
		}
		else if(m2m_comm_mode=='1')
		{
			m2m_comm_mode=RUN_THROUGH_MODE;
			APP_DEBUG("<--m2m_comm_mode:RUN_THROUGH_MODE-->\r\n");	
		}
		else
		{
			APP_DEBUG("<--m2m_comm_mode:mode read error-->\r\n");
		}
	}
	else
	APP_DEBUG("<--m2m_comm_mode:mode read error>\r\n");
	
    //register 
    ret = Ql_Timer_Register(MQTT_TIMER_ID, Callback_Timer, NULL);
	APP_DEBUG("<--Ql_Timer_Register MQTT_TIMER_ID ret:%d-->\r\n", ret);

	ret = Ql_Timer_Register(HEARTBEAT_TIMER_ID, Callback_Timer, NULL);
	APP_DEBUG("<--Ql_Timer_Register HEARTBEAT_TIMER_ID ret:%d-->\r\n", ret);

	Ql_Timer_Start(HEARTBEAT_TIMER_ID, HEARTBEAT_TIMER_PERIOD, TRUE);

	Ql_GPIO_Init(PINNAME_PCM_OUT, PINDIRECTION_OUT, PINLEVEL_LOW, PINPULLSEL_PULLUP);

	ret = Ql_Timer_Register(WTD_FEED_TIMER_ID, callback_WatchDogTimer, NULL);
	if (ret < 0)
	{
		Ql_Debug_Trace("<--register WTD_FEED_TIMER_ID fail ret=%d-->\r\n", ret);
		return;
	}

	ret = Ql_Timer_Start(WTD_FEED_TIMER_ID, WTD_FEED_TIMER_PERIOD, TRUE);
	if (ret < 0)
	{
		Ql_Debug_Trace("<--start WTD_FEED_TIMER_ID timer fail ret=%d-->\r\n", ret);
		return;
	}

    //register
    ret = Ql_Mqtt_Recv_Register(callback_mqtt_recv);
	APP_DEBUG("<--register recv callback successful ret:%d-->\r\n", ret);

    Ql_memset(productKey, 0, sizeof(productKey));
    Ql_memset(deviceName, 0, sizeof(deviceName));
    Ql_memset(deviceSecret, 0, sizeof(deviceSecret));
    Ql_memset(productSecret, 0, sizeof(productSecret));

    Ql_memset(buff_array, 0, sizeof(buff_array));

    /* get device informaion */
	ret = read_device_info_fromfs(buff_array, &readLen);
	if (ret == 0)
	{
		ret = get_device_info(buff_array, productKey, deviceName, deviceSecret);
		if (ret == 0)
		{
			dev_info_flag = 1;
		}
		else
		{
			dev_info_flag = 0;
		}
	}

	ret = read_productsecret_fromfs(buff_array, &readLen);
	if (ret == 0)
	{
		ret = get_product_secret(buff_array, productSecret);
	}

    APP_DEBUG("productKey: %s deviceName: %s deviceSecret: %s\r\n", productKey, deviceName, deviceSecret);
    APP_DEBUG("productSecret: %s\r\n", productSecret);

    Ql_memset(&meta, 0, sizeof(iotx_dev_meta_info_t));
    Ql_memcpy(meta.product_key, productKey, Ql_strlen(productKey));
    Ql_memcpy(meta.product_secret, productSecret, Ql_strlen(productSecret));
    Ql_memcpy(meta.device_name, deviceName, Ql_strlen(deviceName));
    Ql_memcpy(meta.device_secret, deviceSecret, Ql_strlen(deviceSecret));

    if (IOT_Sign_MQTT(region, &meta, &sign_mqtt) < 0)
    {
        return -1;
    }

    APP_DEBUG("sign_mqtt.hostname: %s\r\n", sign_mqtt.hostname);
    APP_DEBUG("sign_mqtt.port    : %d\r\n", sign_mqtt.port);
    APP_DEBUG("sign_mqtt.username: %s\r\n", sign_mqtt.username);
    APP_DEBUG("sign_mqtt.password: %s\r\n", sign_mqtt.password);
    APP_DEBUG("sign_mqtt.clientid: %s\r\n", sign_mqtt.clientid);

	
    while(TRUE)
    {
        Ql_OS_GetMessage(&msg);
        switch(msg.message)
        {
#ifdef __OCPU_RIL_SUPPORT__
        case MSG_ID_RIL_READY:
            APP_DEBUG("<-- RIL is ready -->\r\n");
            Ql_RIL_Initialize();

			uart_write_data(m_myUartPort, "AT=OK!\r\n");

			Ql_memset(strImei, 0x0, sizeof(strImei));
			ret = RIL_GetIMEI(strImei);
			APP_DEBUG("<-- IMEI:%s, ret=%d -->\r\n", strImei, ret);
			if (ret == RIL_AT_SUCCESS)
			{
				imei_valid_flag = 1;
				imei_update_flag = 0;
			}

            break;
#endif
		case MSG_ID_URC_INDICATION:
		{     
			switch (msg.param1)
            {
    		    case URC_SIM_CARD_STATE_IND:
    			APP_DEBUG("<-- SIM Card Status:%d -->\r\n", msg.param2);
				if(SIM_STAT_READY == msg.param2)
				{
					Ql_Timer_Start(MQTT_TIMER_ID, MQTT_TIMER_PERIOD, TRUE);
					APP_DEBUG("<--Ql_Timer_Start MQTT_TIMER_ID ret:%d-->\r\n",ret);
					sim_card_status = 1;
				}
				else if (SIM_STAT_NOT_INSERTED == msg.param2)
				{
					/* …Ë÷√¡À»˝‘™◊È */
					if (dev_info_flag)
					{
						sim_card_status = 0;
						APP_DEBUG("<-- sim_card_status:%d -->\r\n", sim_card_status);
					}
				}
    			break;
				case URC_GPRS_NW_STATE_IND:
				{
					s32 cgreg = 0;
					ret = RIL_NW_GetGPRSState(&cgreg);
					APP_DEBUG("<--Network State:cgreg=%d-->\r\n",cgreg);
					if((cgreg == NW_STAT_REGISTERED)||(cgreg == NW_STAT_REGISTERED_ROAMING))
					{
						if (mqtt_conn_flag == 0)
						{
							m_net_state = STATE_NET_CONNECT;
							uart_write_data(m_myUartPort, NET_CONNECT_RESP);
						}
						else
						{
							m_net_state = STATE_SERVER_CONNECT;
							uart_write_data(m_myUartPort, SERVER_CONNECT_RESP);
						}
					}
					else
					{
						m_net_state = STATE_NET_DISCONNECT;
						uart_write_data(m_myUartPort, NET_DISCONNECT_RESP);
					}
				}
				break;
				case URC_MQTT_OPEN:
				{
					mqtt_urc_param_ptr = msg.param2;
					if(0 == mqtt_urc_param_ptr->result)
					{
     					APP_DEBUG("<--open a MQTT client successfully-->\r\n");
                        m_mqtt_state = STATE_MQTT_CONN;
					}
					else
					{
						APP_DEBUG("<--open a MQTT client failure,error=%d.-->\r\n",mqtt_urc_param_ptr->result);
						m_mqtt_state = STATE_MQTT_OPEN;
						mqtt_open_retry_count++;
						if (mqtt_open_retry_count > MQTT_RETRY_MAX_COUNT)
						{
							APP_DEBUG("<--open a MQTT client failure count > %d, must reboot-->\r\n", MQTT_RETRY_MAX_COUNT);
							Ql_Reset(0);
						}
					}
				}
				break;
    		    case URC_MQTT_CONN:
				{
					mqtt_urc_param_ptr = msg.param2;
					if(0 == mqtt_urc_param_ptr->result)
					{
        		        APP_DEBUG("<--connection to MQTT server successfully->\r\n");
						APP_DEBUG("mqtt connect_code: %d\r\n", mqtt_urc_param_ptr->connect_code);
						printf_mqtt_conn_result(mqtt_urc_param_ptr->connect_code);
						m_mqtt_state = STATE_MQTT_SUB;
						m_net_state = STATE_SERVER_CONNECT;
						mqtt_conn_flag = 1;
						mqtt_server_conn = 1;
						uart_write_data(m_myUartPort, SERVER_CONNECT_RESP);
					}
					else
					{
						APP_DEBUG("<--connection to MQTT failure,error=%d.-->\r\n",mqtt_urc_param_ptr->result);
						APP_DEBUG("mqtt connect_code: %d\r\n", mqtt_urc_param_ptr->connect_code);
						printf_mqtt_conn_result(mqtt_urc_param_ptr->connect_code);

						APP_DEBUG("Reopen MQTT connection and try to send CONNECT packet to server again.\r\n");
						m_mqtt_state = STATE_MQTT_OPEN;
						mqtt_conn_retry_count++;
						if (mqtt_conn_retry_count > MQTT_RETRY_MAX_COUNT)
						{
							APP_DEBUG("<--connection to MQTT server failure count > %d, must reboot-->\r\n", MQTT_RETRY_MAX_COUNT);
							Ql_Reset(0);
						}
					}
    		    }
    			break;
                case URC_MQTT_SUB:
				{
					mqtt_urc_param_ptr = msg.param2;
					if((0 == mqtt_urc_param_ptr->result)&&(128 != mqtt_urc_param_ptr->sub_value[0]))
					{
        		        APP_DEBUG("<--subscribe topics successfully->\r\n");
						m_mqtt_state = STATE_MQTT_PUB;
					}
					else
					{
						APP_DEBUG("<--subscribe topics failure,error=%d.-->\r\n",mqtt_urc_param_ptr->result);
						m_mqtt_state = URC_MQTT_SUB;
					}
    		    }
    			break;
				case URC_MQTT_PUB:
				{
					mqtt_urc_param_ptr = msg.param2;
					if(0 == mqtt_urc_param_ptr->result)
					{
        		        APP_DEBUG("<--publish messages to ali server successfully->\r\n");
                        if (pub_message_id > 3)
                        {
                            m_mqtt_state = STATE_TOTAL_NUM;
                            uart_write_data(m_myUartPort, OK_ASK);
                        }
                        else
                        {
                            m_mqtt_state = STATE_MQTT_PUB;
                        }
					}
					else
					{
						APP_DEBUG("<--publish messages to ali server failure,error=%d.-->\r\n",mqtt_urc_param_ptr->result);

                        if (pub_message_id > 3)
                        {
                            uart_write_data(m_myUartPort, FAIL_ASK);
                        }
					}
    		    }
    			break;
			    case URC_MQTT_CLOSE:
				{
					mqtt_urc_param_ptr = msg.param2;
					if(0 == mqtt_urc_param_ptr->result)
					{
        		        APP_DEBUG("<--closed mqtt socket successfully-->\r\n");
						m_mqtt_state = STATE_TOTAL_NUM;
                        m_net_state = STATE_SERVER_DISCONNECT;
						mqtt_conn_flag = 0;
                        uart_write_data(m_myUartPort, SERVER_DISCONNECT_RESP);
					}
					else
					{
						APP_DEBUG("<--closed mqtt socket failure,error=%d.-->\r\n",mqtt_urc_param_ptr->result);
						m_mqtt_state = STATE_MQTT_CLOSE;
					}
    		    }
    			break;
				case URC_MQTT_DISC:
				{
					mqtt_urc_param_ptr = msg.param2;
					if(0 == mqtt_urc_param_ptr->result)
					{
						APP_DEBUG("<--Disconnect MQTT successfully-->\r\n");
						m_mqtt_state = STATE_TOTAL_NUM;
						m_net_state = STATE_SERVER_DISCONNECT;
						mqtt_conn_flag = 0;
						uart_write_data(m_myUartPort, SERVER_DISCONNECT_RESP);
					}
					else
					{
						APP_DEBUG("<--Disconnect MQTT failure,error = %d-->\r\n",mqtt_urc_param_ptr->result);
						m_mqtt_state = STATE_MQTT_DISC;
					}
				}
				break;
				case URC_MQTT_STATE:
				{
					mqtt_urc_param_ptr = msg.param2;
					APP_DEBUG("mqtt_state: %d result: %d\r\n", mqtt_urc_param_ptr->mqtt_state, mqtt_urc_param_ptr->result);
					switch (mqtt_urc_param_ptr->mqtt_state)
					{
						case 1:
						{
							APP_DEBUG("MQTT Connection is closed or reset by peer.\r\n");
							m_net_state = STATE_SERVER_DISCONNECT;
							uart_write_data(m_myUartPort, SERVER_DISCONNECT_RESP);
							APP_DEBUG("<--must reboot-->\r\n");
							Ql_Sleep(20);
							Ql_Reset(0);
						}
						break;
						case 2:
						{
							APP_DEBUG("Sending PINGREQ packet timed out or failed.\r\n");
							ret = RIL_NW_ClosePDPContext();
							APP_DEBUG("<-- Close PDP context, ret=%d -->\r\n", ret);
							APP_DEBUG("<--must reboot-->\r\n");
							Ql_Sleep(20);
							Ql_Reset(0);
						}
						break;
						case 3:
						{
							APP_DEBUG("Sending CONNECT packet timed out or failed.\r\n");
							APP_DEBUG("Reopen MQTT connection and try to send CONNECT packet to server again.\r\n");
							/* Reopen MQTT connection and try to send CONNECT packet to server again */
							m_mqtt_state = STATE_MQTT_OPEN;
						}
						break;
						case 4:
						{
							APP_DEBUG("Receiving CONNACK packet timed out or failed.\r\n");
							APP_DEBUG("Reopen MQTT connection and try to send CONNECT packet to server again.\r\n");
							/* Reopen MQTT connection and try to send CONNECT packet to server again */
							m_mqtt_state = STATE_MQTT_OPEN;
						}
						break;
						case 5:
						{
							APP_DEBUG("Client sends DISCONNECT packet to sever but server is initiative to close MQTT connection.\r\n");
						}
						break;
						case 6:
						{
							APP_DEBUG("Client is initiative to close MQTT connection due to packet sending failure all the time.\r\n");
						}
						break;
						case 7:
						{
							APP_DEBUG("The link is not alive or the server is unavailable.\r\n");
						}
						break;
						default:
						break;
					}
				}
				break;
		        default:
    		    //APP_DEBUG("<-- Other URC: type=%d\r\n", msg.param1);
    	        break;
			}
		}
		break;
	    case MSG_ID_USER_DATA:
	    {
            APP_DEBUG("\r\n<--Sub task 1 recv MSG: SrcId = %d, MsgID = %d Data1 = %d, Data2 = %d-->\r\n", \
                        msg.srcTaskId, \
                        msg.message,\
                        msg.param1, \
                        msg.param2);

            switch (msg.param1)
            {
                case CMD_PROPERTY_POST:
                {
                    Ql_memset(payload, 0, PAYLOAD_MAX_LEN);
                    get_paylaod(PROPERTY_DATA, uart_buff, payload);

                    for (i = 0; i < Ql_strlen(payload); i++)
                    {
                        if (payload[i]  == '\"')
                        {
                            payload[i] = '\'';
                        }
                    }

                    APP_DEBUG("\r\n<--uart_buff: %s-->\r\n", uart_buff);
                    APP_DEBUG("\r\n<--payload: %s-->\r\n", payload);

                    ret = example_user_post_property(connect_id, pub_message_id++, payload);
                }
                break;

                case CMD_EVENT_POST:
                {
                    Ql_memset(event_name, 0, 32);
                    Ql_memset(payload, 0, PAYLOAD_MAX_LEN);
                    get_event_name(uart_buff, event_name);
                    get_paylaod(EVENT_DATA, uart_buff, payload);

                    for (i = 0; i < Ql_strlen(payload); i++)
                    {
                        if (payload[i] == '\"')
                        {
                            payload[i] = '\'';
                        }
                    }

                    APP_DEBUG("\r\n<--uart_buff: %s-->\r\n", uart_buff);
                    APP_DEBUG("\r\n<--event_name: %s-->\r\n", event_name);
                    APP_DEBUG("\r\n<--payload: %s-->\r\n", payload);

                    ret = example_user_post_event(connect_id, pub_message_id++, event_name, payload);
                }
                break;

                case CMD_DEV_INFO_SET:
                {
                    buff_ptr = strstr(uart_buff, "=");
                    buff_ptr++;
                    ret = write_device_info_tofs(buff_ptr, &writeLen);
                    if (ret == 0)
                    {
                        uart_write_data(m_myUartPort, OK_ASK);
                    }
                    else
                    {
                        uart_write_data(m_myUartPort, FAIL_ASK);
                    }
                }
                break;

                case CMD_DEV_INFO_GET:
                {
                    Ql_memset(buff_array, 0, 128);
                    ret = read_device_info_fromfs(buff_array, &readLen);
                    if (ret == 0)
                    {
                        uart_write_data(m_myUartPort, "+ok=");
                        uart_write_data(m_myUartPort, buff_array);
                        uart_write_data(m_myUartPort, "\r\n");
                    }
                    else
                    {
                        uart_write_data(m_myUartPort, FAIL_ASK);
                    }
                }
                break;

                case CMD_PRODUCT_SECRET_SET:
                {
                    buff_ptr = strstr(uart_buff, "=");
                    buff_ptr++;
                    ret = write_productsecret_tofs(buff_ptr, &writeLen);
                    if (ret == 0)
                    {
                        uart_write_data(m_myUartPort, OK_ASK);
                    }
                    else
                    {
                        uart_write_data(m_myUartPort, FAIL_ASK);
                    }
                }
				break;

                case CMD_PRODUCT_SECRET_GET:
                {
                    Ql_memset(buff_array, 0, 128);
                    ret = read_productsecret_fromfs(buff_array, &readLen);
                    if (ret == 0)
                    {
                        if (ret == 0)
                        {
                            uart_write_data(m_myUartPort, "+ok=");
                            uart_write_data(m_myUartPort, productSecret);
                            uart_write_data(m_myUartPort, "\r\n");
                        }
                        else
                        {
                            uart_write_data(m_myUartPort, FAIL_ASK);
                        }
                    }
                    else
                    {
                        uart_write_data(m_myUartPort, FAIL_ASK);
                    }
                }
                break;

                case CMD_NET_STATUS_GET:
                {
                    switch(m_net_state)
                    {
                        case STATE_NET_DISCONNECT:
                            uart_write_data(m_myUartPort, NET_DISCONNECT_ASK);
                            break;

                        case STATE_NET_CONNECT:
                            uart_write_data(m_myUartPort, NET_CONNECT_ASK);
                            break;

                        case STATE_SERVER_DISCONNECT:
                            uart_write_data(m_myUartPort, SERVER_DISCONNECT_ASK);
                            break;

                        case STATE_SERVER_CONNECT:
                            uart_write_data(m_myUartPort, SERVER_CONNECT_ASK);
                            break;

                        default:
                            break;
                    }
                }
                break;

                case CMD_APP_VERSION_GET:
                {
                    Ql_memset(app_ver, 0, 32);
                    GetFirmwareVersion(app_ver);
                    uart_write_data(m_myUartPort, "+ok=");
                    uart_write_data(m_myUartPort, app_ver);
                    uart_write_data(m_myUartPort, "\r\n");
                }
                break;

                case CMD_RESET:
                {
                    uart_write_data(m_myUartPort, OK_ASK);
                    Ql_Sleep(100);
                    Ql_Reset(0);
                }
                break;

                default:
                    break;
            }
	    }
	    break;

	    default:
            break;
        }
    }
}

static s32 ReadSerialPort(Enum_SerialPort port, /*[in]*/u32 lastLen, /*[out]*/u8* pBuffer, /*[in]*/u32 bufLen)
{
    s32 rdLen = 0;
    s32 rdTotalLen = 0;
	s32 cnt = 0;

    if (NULL == pBuffer || 0 == bufLen)
    {
        return -1;
    }

    /* new frame start */
    if (lastLen == 0)
    {
        Ql_memset(pBuffer, 0x0, bufLen);
    }

    while (1)
    {
        rdLen = Ql_UART_Read(port, pBuffer + rdTotalLen + lastLen, bufLen - rdTotalLen - lastLen);


		if (rdLen <= 0)  // All data is read out, or Serial Port Error!
        {

			
            break;
        }
        rdTotalLen += rdLen;
	
		
				
        // Continue to read...
    }

    if (rdLen < 0) // Serial Port Error!
    {
        APP_DEBUG("Fail to read from port[%d]\r\n", port);
        return -99;
    }

    return rdTotalLen;
}

static void CallBack_UART_Hdlr(Enum_SerialPort port, Enum_UARTEventType msg, bool level, void* customizedPara)
{
	s32 writeLen; 
    switch (msg)
    {
    case EVENT_UART_READY_TO_READ:
        {
           if (m_myUartPort == port)
           {
                s32 totalBytes = ReadSerialPort(port, lastTotalBytes, m_RxBuf_Uart, sizeof(m_RxBuf_Uart));
                if (totalBytes <= 0)
                {
                    APP_DEBUG("<-- No data in UART buffer! -->\r\n");
					lastTotalBytes = 0;
					RxBuff_totalBytes = 0;
                    return;
                }
                {// Read data from UART
                    s32 ret;
                    char* pCh = NULL;

                    // Echo
                    Ql_UART_Write(m_myUartPort, m_RxBuf_Uart, totalBytes);

                    Ql_memset(uart_buff, 0, SERIAL_RX_BUFFER_LEN);
                    Ql_strcpy(uart_buff, m_RxBuf_Uart);

					//ÈÄè‰º†Ê®°Âºè
					if(m2m_comm_mode == 1)			//M2M is in run_through_mode
					{
						char rawsend_topic_name[80] ={0} ;
						char payload[50] = {0} ;

						//ÈÄÄÂá∫ÈÄè‰º†Ê®°Âºè
						if(!Ql_strncmp(m_RxBuf_Uart,EXIT_RUN_THROUGH_MODE, Ql_strlen(EXIT_RUN_THROUGH_MODE)))
						{
							write_run_through_flag_tofs("0",&writeLen);
							m2m_comm_mode = AT_CMD_MODE;
							APP_DEBUG("\r\n");
							return;
						}

						Ql_sprintf(rawsend_topic_name,
								"/sys/%s/%s/thing/model/up_raw",
								productKey,
           		 				deviceName);
						Ql_strcpy(payload, m_RxBuf_Uart);
						APP_DEBUG("topic_name: %s\r\n", rawsend_topic_name);	
						APP_DEBUG("payload: %s\r\n",payload);
						APP_DEBUG("\r\n");
						
						//‰∏äÊä•Ëá™ÂÆö‰πâÂçèËÆÆ
						ret = RIL_MQTT_QMTPUB(connect_id, pub_message_id, QOS1_AT_LEASET_ONCE, 0, rawsend_topic_name, totalBytes, m_RxBuf_Uart);
					    if (RIL_AT_SUCCESS == ret)
					    {
					        APP_DEBUG("<--Start publish messages to ali server-->\r\n");

					        ret = SUCCESS_RETURN;
							return;
					      
					    }
					    else
					    {
					        APP_DEBUG("<--publish messages to ali server failure, error = %d.-->\r\n", ret);
					        ret = FAIL_RETURN;
							return;
					    }

					

					}

					else
					{
						pCh = Ql_strstr((char*)m_RxBuf_Uart, AT_CMD_END_FLAG);
	                    if (pCh)
	                    {
	                        lastTotalBytes = 0;
	                        RxBuff_totalBytes += totalBytes;

	                        *(pCh + 0) = '\0';
	                        *(pCh + 1) = '\0';
	                    }
	                    else // keep read until we get "\r\n", one complete frame
	                    {
	                        lastTotalBytes += totalBytes;
	                        RxBuff_totalBytes += totalBytes;
	                        return;
	                    }
	                    if (!Ql_strncmp(m_RxBuf_Uart, CMD_PROPERTY_HEAD, Ql_strlen(CMD_PROPERTY_HEAD)))
	                    {
	                        Ql_OS_SendMessage(main_task_id, MSG_ID_USER_DATA, CMD_PROPERTY_POST, 0);
	                        return;
	                    }
	                    else if (!Ql_strncmp(m_RxBuf_Uart, CMD_EVENT_HEAD, Ql_strlen(CMD_EVENT_HEAD)))
	                    {
	                        Ql_OS_SendMessage(main_task_id, MSG_ID_USER_DATA, CMD_EVENT_POST, 0);
	                        return;
	                    }
	                    else if (!Ql_strncmp(m_RxBuf_Uart, CMD_DEV_INFO_SET_HEAD, Ql_strlen(CMD_DEV_INFO_SET_HEAD)))
	                    {
	                        if (Ql_strlen(m_RxBuf_Uart) > Ql_strlen(CMD_DEV_INFO_SET_HEAD))
	                        {
	                            Ql_OS_SendMessage(main_task_id, MSG_ID_USER_DATA, CMD_DEV_INFO_SET, 0);
	                        }
	                        else
	                        {
	                            Ql_OS_SendMessage(main_task_id, MSG_ID_USER_DATA, CMD_DEV_INFO_GET, 0);
	                        }
	                        return;
	                    }
	                    else if (!Ql_strncmp(m_RxBuf_Uart, CMD_DEV_INFO_GET_HEAD, Ql_strlen(CMD_DEV_INFO_GET_HEAD)))
	                    {
	                        Ql_OS_SendMessage(main_task_id, MSG_ID_USER_DATA, CMD_DEV_INFO_GET, 0);
	                        return;
	                    }
	                    else if (!Ql_strncmp(m_RxBuf_Uart, CMD_PRODUCT_SECRET_SET_HEAD, Ql_strlen(CMD_PRODUCT_SECRET_SET_HEAD)))
	                    {
	                        if (Ql_strlen(m_RxBuf_Uart) > Ql_strlen(CMD_PRODUCT_SECRET_SET_HEAD))
	                        {
	                            Ql_OS_SendMessage(main_task_id, MSG_ID_USER_DATA, CMD_PRODUCT_SECRET_SET, 0);
	                        }
	                        else
	                        {
	                            Ql_OS_SendMessage(main_task_id, MSG_ID_USER_DATA, CMD_PRODUCT_SECRET_GET, 0);
	                        }
	                        return;
	                    }
	                    else if (!Ql_strncmp(m_RxBuf_Uart, CMD_PRODUCT_SECRET_GET_HEAD, Ql_strlen(CMD_PRODUCT_SECRET_GET_HEAD)))
	                    {
	                        Ql_OS_SendMessage(main_task_id, MSG_ID_USER_DATA, CMD_PRODUCT_SECRET_GET, 0);
	                        return;
	                    }
	                    else if (!Ql_strncmp(m_RxBuf_Uart, CMD_NET_STATUS_GET_HEAD, Ql_strlen(CMD_NET_STATUS_GET_HEAD)))
	                    {
	                        Ql_OS_SendMessage(main_task_id, MSG_ID_USER_DATA, CMD_NET_STATUS_GET, 0);
	                        return;
	                    }
	                    else if (!Ql_strncmp(m_RxBuf_Uart, CMD_APP_VERSION_GET_HEAD, Ql_strlen(CMD_APP_VERSION_GET_HEAD)))
	                    {
	                        Ql_OS_SendMessage(main_task_id, MSG_ID_USER_DATA, CMD_APP_VERSION_GET, 0);
	                        return;
	                    }
	                    else if (!Ql_strncmp(m_RxBuf_Uart, CMD_RESET_HEAD, Ql_strlen(CMD_RESET_HEAD)))
	                    {
	                        Ql_OS_SendMessage(main_task_id, MSG_ID_USER_DATA, CMD_RESET, 0);
	                        return;
	                    }
						else if(!Ql_strncmp(m_RxBuf_Uart,CMD_RUN_THROUGH_MODE, Ql_strlen(CMD_RUN_THROUGH_MODE)))
						{
							write_run_through_flag_tofs("1",&writeLen);
							m2m_comm_mode = RUN_THROUGH_MODE;
							APP_DEBUG(">");
							return;
						}

					
	                    // No permission for single <cr><lf>
	                    if (Ql_strlen((char*)m_RxBuf_Uart) == 0)
	                    {
	                        return;
	                    }
	                    ret = Ql_RIL_SendATCmd((char*)m_RxBuf_Uart, totalBytes, ATResponse_Handler, NULL, 0);

					}
                }
           }
           break;
        }
    case EVENT_UART_READY_TO_WRITE:
        break;
    default:
        break;
    }
}




static void callback_mqtt_recv(u8* buffer,u32 length)
{
    u8 start_pos, end_pos;
    u32 i, j;
    u8 topic_name[128];
    u8 payload[PAYLOAD_MAX_LEN];
	u8 topic_id;
    u8 *pstart = buffer;
    u8 *pstr = NULL;
    u8 tmp_buff[256];
	u8 service_name[32], service_payload[256];
    u8 len;
	cJSON *root = NULL, *item_params = NULL, *item1_params;

    /*
     * property set example: +QMTRECV: 0,0,/sys/a1WjNmzlsCL/865933034722252/thing/service/property/set,{"method":"thing.service.property.set","id":"1757929280","params":{"PowerMode":0},"version":"1.0.0"}
     * service down example: +QMTRECV: 0,0,/sys/a1eqTmUf5Jy/865362049404443/thing/service/mcu_ota,{"method":"thing.service.mcu_ota","id":"394606312","params":{"size":256,"version":"1.0.1","url":"http://www/baidu.com"},"version":"1.0.0"}
	*/
	APP_DEBUG("<--buffer(%s),length(%d)-->\r\n",buffer,length);

    Ql_memset(topic_name, 0, sizeof(topic_name));
    Ql_memset(payload, 0, sizeof(payload));

    start_pos = seek_comma_pos(buffer, 2);
    end_pos = seek_comma_pos(buffer, 3);

    if ((start_pos == 0xff) || (end_pos == 0xff))
    {
        return;
    }

    for (i = start_pos + 1, j = 0; i < end_pos; i++)
    {
        topic_name[j++] = buffer[i];
    }

    topic_name[j] = '\0';

    APP_DEBUG("topic_name: %s\r\n", topic_name);

	start_pos = end_pos;
	for (i = start_pos + 1, j = 0; i < length; i++)
	{
		payload[j++] = buffer[i];
	}
	payload[j] = '\0';
	APP_DEBUG("payload: %s\r\n", payload);

#if 1
	topic_id = get_mqtt_topic_id((const char *)topic_name);
	switch (topic_id)
	{
	case PROPERTY_SET:
		APP_DEBUG("set property!\r\n");
		root = cJSON_Parse(payload);
		if (root == NULL || !cJSON_IsObject(root))
		{
			APP_DEBUG("JSON Parse Error\r\n");
		}
		APP_DEBUG("root: %s\r\n", cJSON_PrintUnformatted(root));

		item_params = cJSON_GetObjectItem(root, "params");
		if ((item_params != NULL) && cJSON_IsObject(item_params))
		{
			APP_DEBUG("item_params->type: %d\r\n", item_params->type);
			APP_DEBUG("item_params: %s\r\n", cJSON_PrintUnformatted(item_params));

			Ql_memset(tmp_buff, 0, PAYLOAD_MAX_LEN);
			Ql_sprintf(tmp_buff, "+ILOPDATA=JSON,%s", cJSON_PrintUnformatted(item_params));
			uart_write_data(m_myUartPort, tmp_buff);
			uart_write_data(m_myUartPort, AT_CMD_END_FLAG);
		}
		break;
	case SERVICE_DOWN:
		APP_DEBUG("service down!\r\n");
		root = cJSON_Parse(payload);
		if (root == NULL || !cJSON_IsObject(root))
		{
			APP_DEBUG("JSON Parse Error\r\n");
		}
		APP_DEBUG("root: %s\r\n", cJSON_PrintUnformatted(root));

		item_params = cJSON_GetObjectItem(root, "method");
		if (item_params != NULL && cJSON_IsString(item_params))
		{
			APP_DEBUG("method: %s\r\n", item_params->valuestring);
			Ql_memset(tmp_buff, 0, sizeof(tmp_buff));
			Ql_strcpy(tmp_buff, cJSON_PrintUnformatted(item_params));
			len = Ql_strlen(tmp_buff);
			for (i = Ql_strlen("\"thing.service."), j = 0; i < len - 1; i++)
			{
				service_name[j++] = tmp_buff[i];
			}
			service_name[j] = '\0';
			APP_DEBUG("service_name: %s\r\n", service_name);
		}

		item_params = cJSON_GetObjectItem(root, "params");
		if ((item_params != NULL) && cJSON_IsObject(item_params))
		{
			APP_DEBUG("service payload: %s\r\n", cJSON_PrintUnformatted(item_params));
		}

		break;
	case OTA_UPGRADE:
		APP_DEBUG("ota upgrade!\r\n");
		root = cJSON_Parse(payload);
		if (root == NULL || !cJSON_IsObject(root))
		{
			APP_DEBUG("JSON Parse Error\r\n");
		}
		APP_DEBUG("root: %s\r\n", cJSON_PrintUnformatted(root));

		item_params = cJSON_GetObjectItem(root, "data");
		if (item_params != NULL && cJSON_IsObject(item_params))
		{
			item1_params = cJSON_GetObjectItem(item_params, "size");
			if ((item1_params != NULL) && cJSON_IsNumber(item1_params))
			{
				APP_DEBUG("size: %d\r\n", item1_params->valueint);
			}

			item1_params = cJSON_GetObjectItem(item_params, "version");
			if ((item1_params != NULL) && cJSON_IsString(item1_params))
			{
				APP_DEBUG("version: %s\r\n", item1_params->valuestring);
			}

			item1_params = cJSON_GetObjectItem(item_params, "url");
			if ((item1_params != NULL) && cJSON_IsString(item1_params))
			{
				APP_DEBUG("url: %s\r\n", item1_params->valuestring);
			}
		}
		break;
	case RAW_DOWN_ID:
		APP_DEBUG("get_run_through_msg!\r\n");
		s32 raw_payload_len = j-1; 
		Ql_UART_Write(m_myUartPort, payload,raw_payload_len);

	case TOPIC_MAX_ID:
		break;
	default:
		break;
	}

	if (root != NULL)
	{
		cJSON_Delete(root);
	}
#endif
}

static void Callback_Timer(u32 timerId, void* param)
{
    int ret = 0;
    char sub_topic[256];
    char ota_sub_topic[128];

    if (MQTT_TIMER_ID == timerId)
    {
        switch (m_mqtt_state)
        {        
            case STATE_NW_QUERY_STATE:
            {
                s32 cgreg = 0;

                ret = RIL_NW_GetGPRSState(&cgreg);
                APP_DEBUG("<--Network State:cgreg=%d-->\r\n", cgreg);
                if((cgreg == NW_STAT_REGISTERED)||(cgreg == NW_STAT_REGISTERED_ROAMING))
                {
					//<Set PDP context 0
					RIL_NW_SetGPRSContext(0);
					APP_DEBUG("<--Set PDP context 0-->\r\n");
					//<Set APN
					ret = RIL_NW_SetAPN(1, APN, USERID, PASSWD);
					APP_DEBUG("<--Set APN-->\r\n");
					//PDP activated
					ret = RIL_NW_OpenPDPContext();
					if (ret == RIL_AT_SUCCESS)
					{
						APP_DEBUG("<--Activate PDP context,ret = %d-->\r\n", ret);
						if (dev_info_flag)
						{
							m_mqtt_state = STATE_MQTT_CFG;
						}

						if ((net_status_report & (1 << 1)) == 0 || (m_net_state == STATE_NET_DISCONNECT))
						{
							net_status_report |= (1 << 1);
							uart_write_data(m_myUartPort, NET_CONNECT_RESP);
						}
						m_net_state = STATE_NET_CONNECT;
					}
					else
					{
						APP_DEBUG("<--Activate PDP context failed,ret = %d-->\r\n", ret);
						ret = RIL_NW_ClosePDPContext();
						APP_DEBUG("<-- Close PDP context, ret=%d -->\r\n", ret);
						if (pdp_open_count++ > 20)
						{
							APP_DEBUG("<-- Open PDP Context Failed, must reboot -->\r\n");
							Ql_Reset(0);
						}
					}
                }
                else
                {
					if (((net_status_report & (1 << 0)) == 0) || (m_net_state == STATE_NET_CONNECT))
					{
						net_status_report |= (1 << 0);
						uart_write_data(m_myUartPort, NET_DISCONNECT_RESP);
					}
					m_net_state = STATE_NET_DISCONNECT;
                }
                break;
            }

            case STATE_MQTT_CFG:
            {
                ret = RIL_MQTT_QMTCFG_VERSION(connect_id, Version_3_1_1);
                if (RIL_AT_SUCCESS == ret)
                {
                    APP_DEBUG("<--mqtt version configure successfully-->\r\n");
                }else
                {
                    APP_DEBUG("<--mqtt version configure failure,error=%d.-->\r\n", ret);
					m_mqtt_state = STATE_MQTT_CFG;
                }

                ret = RIL_MQTT_QMTCFG_KEEPALIVE(connect_id, DEFAULT_KEEPALIVE_TIME);
                if (RIL_AT_SUCCESS == ret)
                {
                    APP_DEBUG("<--mqtt keep alive time configure successfully-->\r\n");
                }else
                {
                    APP_DEBUG("<--mqtt keep alive time configure failure,error=%d.-->\r\n", ret);
					m_mqtt_state = STATE_MQTT_CFG;
                }

                ret = RIL_MQTT_QMTCFG_TIMEOUT(connect_id, PKT_TIMEOUT_SECOND, RETRY_TIMES, 0);
                if (RIL_AT_SUCCESS == ret)
                {
                    APP_DEBUG("<--mqtt timeout configure successfully-->\r\n");
                    APP_DEBUG("<--mqtt pkt_timeout %d seconds, retry times: %d-->\r\n", PKT_TIMEOUT_SECOND, RETRY_TIMES);
                }else
                {
                    APP_DEBUG("<--mqtt timeout configure failure,error=%d.-->\r\n", ret);
                    m_mqtt_state = STATE_MQTT_CFG;
                }

                ret = RIL_MQTT_QMTCFG_Ali(connect_id,productKey,deviceName,deviceSecret);
                if (RIL_AT_SUCCESS == ret)
                {
                    APP_DEBUG("<- Ali Platform configure successfully-->\r\n");
					m_mqtt_state = STATE_MQTT_OPEN;
                }else
                {
                    APP_DEBUG("<--Ali Platform configure failure,error=%d.-->\r\n", ret);
					m_mqtt_state = STATE_MQTT_CFG;
                }
                break;
            }

			case STATE_MQTT_OPEN:
            {
                ret = RIL_MQTT_QMTOPEN(connect_id,sign_mqtt.hostname,sign_mqtt.port);
                if (RIL_AT_SUCCESS == ret)
                {
                    APP_DEBUG("<--Start opening an MQTT client-->\r\n");
					m_mqtt_state = STATE_TOTAL_NUM;
                }else
                {
                    APP_DEBUG("<--open a MQTT client failure,error=%d.-->\r\n", ret);
					m_mqtt_state = STATE_MQTT_OPEN;
                }
                break;
            }
            case STATE_MQTT_CONN:
            {
                ret = RIL_MQTT_QMTCONN(connect_id,productKey,NULL,NULL);
				//ret = RIL_MQTT_QMTCONN(connect_id,productKey,sign_mqtt.username,sign_mqtt.password);
	            if (RIL_AT_SUCCESS == ret)
                {
                    APP_DEBUG("<--Start connection to MQTT server-->\r\n");
					m_mqtt_state = STATE_TOTAL_NUM;
                }else
                {
                    APP_DEBUG("<--connection to MQTT server failure,error=%d.-->\r\n", ret);
					m_mqtt_state = STATE_MQTT_CONN;
                }
                break;
            }
			case STATE_MQTT_SUB:
			{
				pub_message_id = 0;

                mqtt_topic_info_t.count = 1;
				mqtt_topic_info_t.topic[0] = (u8*)Ql_MEM_Alloc(sizeof(u8)*256);

				Ql_sprintf(sub_topic, PROPERTY_SET_TOPIC, productKey, deviceName);
				APP_DEBUG("sub_topic: %s\r\n", sub_topic);
#if 0
				Ql_sprintf(ota_sub_topic, OTA_UPGRADE_SUB_TOPIC, productKey, deviceName);
				APP_DEBUG("ota_sub_topic: %s\r\n", ota_sub_topic);
#endif
				Ql_memset(mqtt_topic_info_t.topic[0],0,256);
				Ql_memcpy(mqtt_topic_info_t.topic[0],sub_topic,Ql_strlen(sub_topic));
                mqtt_topic_info_t.qos[0] = QOS1_AT_LEASET_ONCE;
				sub_message_id++;  // 1-65535.

#if 0
				Ql_memset(mqtt_topic_info_t.topic[1],0,128);
				Ql_memcpy(mqtt_topic_info_t.topic[1],ota_sub_topic,Ql_strlen(ota_sub_topic));
				mqtt_topic_info_t.qos[1] = QOS1_AT_LEASET_ONCE;
				sub_message_id++;  // 1-65535.
#endif

				ret = RIL_MQTT_QMTSUB(connect_id,sub_message_id,&mqtt_topic_info_t);
				
				Ql_MEM_Free(mqtt_topic_info_t.topic[0]);
	            mqtt_topic_info_t.topic[0] = NULL;
#if 0
				Ql_MEM_Free(mqtt_topic_info_t.topic[1]);
				mqtt_topic_info_t.topic[1] = NULL;
#endif
                if (RIL_AT_SUCCESS == ret)
                {
                    APP_DEBUG("<--Start subscribe topics-->\r\n");
					m_mqtt_state = STATE_TOTAL_NUM;
                }else
                {
                    APP_DEBUG("<--subscribe topics failure,error=%d.-->\r\n",ret);
					m_mqtt_state = STATE_MQTT_SUB;
					mqtt_sub_retry_count++;
					if (mqtt_sub_retry_count > MQTT_RETRY_MAX_COUNT)
					{
						APP_DEBUG("<--connection to MQTT server failure count > %d, must reboot-->\r\n", MQTT_RETRY_MAX_COUNT);
						Ql_Reset(0);
					}
                }
                break;
            }
            case STATE_MQTT_PUB:
            {
				pub_message_id++;  // The range is 0-65535. It will be 0 only when<qos>=0.
                if (pub_message_id == 1)
                {
                    ret = iotx_report_devinfo(connect_id, pub_message_id);
                    if (ret == SUCCESS_RETURN)
                    {
                        APP_DEBUG("iotx_report_devinfo successfully\r\n");
                    }
                    else
                    {
                        APP_DEBUG("iotx_report_devinfo failure\r\n");
						m_mqtt_state = STATE_MQTT_PUB;
						pub_message_id--;
						mqtt_pub_retry_count++;
                    }
                }

                if (pub_message_id == 2)
                {
                    ret = iotx_report_firmware_version(connect_id, pub_message_id);
                    if (ret == SUCCESS_RETURN)
                    {
                        APP_DEBUG("iotx_report_firmware_version successfully\r\n");
                    }
                    else
                    {
                        APP_DEBUG("iotx_report_firmware_version failure\r\n");
						m_mqtt_state = STATE_MQTT_PUB;
						pub_message_id--;
						mqtt_pub_retry_count++;
                    }
                }

				if (pub_message_id == 3)
                {
                    ret = iotx_report_device_imei_info();
                    if (ret == SUCCESS_RETURN)
                    {
                        APP_DEBUG("iotx_report_device_imei_info successfully\r\n");
                    }
                    else
                    {
                        APP_DEBUG("iotx_report_device_imei_info failure\r\n");
						m_mqtt_state = STATE_MQTT_PUB;
						pub_message_id--;
						mqtt_pub_retry_count++;
                    }
                }

				if (mqtt_pub_retry_count > MQTT_RETRY_MAX_COUNT * 2)
				{
					APP_DEBUG("<--pub mqtt message failure count > %d, must reboot-->\r\n", MQTT_RETRY_MAX_COUNT);
					Ql_Reset(0);
				}

                break;
            }
			case STATE_MQTT_CLOSE:
            {
				ret = RIL_MQTT_QMTCLOSE(connect_id);
                if (RIL_AT_SUCCESS == ret)
                {
                    APP_DEBUG("<--Start closed MQTT socket-->\r\n");
					m_mqtt_state = STATE_TOTAL_NUM;
                }else
                {
                    APP_DEBUG("<--closed MQTT socket failure,error=%d.-->\r\n", ret);
					m_mqtt_state = STATE_MQTT_CLOSE;
			    }
                break;
            }
			case STATE_MQTT_DISC:
            {
				ret = RIL_MQTT_QMTDISC(connect_id);
                if (RIL_AT_SUCCESS == ret)
                {
                    APP_DEBUG("<--Start disconnect MQTT socket-->\r\n");
					m_mqtt_state = STATE_TOTAL_NUM;
                }else
                {
                    APP_DEBUG("<--disconnect MQTT socket failure,error=%d.-->\r\n", ret);
					m_mqtt_state = STATE_MQTT_DISC;
                }
                break;
            }
			case STATE_TOTAL_NUM:
            {
				//APP_DEBUG("<--Process wait->\r\n");
				m_mqtt_state = STATE_TOTAL_NUM;
				break;
            }
            default:
                break;
        }
    }

	if (HEARTBEAT_TIMER_ID == timerId)
	{
		//÷Æ«∞“—æ≠¡¨Ω”…œmqtt∑˛ŒÒ∆˜
		if (mqtt_server_conn)
		{
			if (m_net_state != STATE_SERVER_CONNECT)
			{
				mqtt_disconn_time += HEARTBEAT_TIMER_PERIOD;
				if (mqtt_disconn_time > MQTT_DISCONN_MAX_TIME)
				{
					APP_DEBUG("<--MQTT Disconnect Timeout, must reboot-->\r\n");
					Ql_Reset(0);
				}
			}
		}

		if (sim_card_status == 0)
		{
			sim_card_error_time += HEARTBEAT_TIMER_PERIOD;
			if (sim_card_error_time > SIM_CARD_ERROR_MAX_TIME)
			{
				APP_DEBUG("<--SIM Card error %d ms Timeout, must reboot-->\r\n", SIM_CARD_ERROR_MAX_TIME);
				Ql_Reset(0);
			}
		}
	}
}

static void callback_WatchDogTimer(u32 timerId, void* param)
{
    /* must feed dog once within 1.6 second */
    Ql_GPIO_SetLevel(wtd_feedpin, PINLEVEL_HIGH);
    Ql_Sleep(20);
    Ql_GPIO_SetLevel(wtd_feedpin, PINLEVEL_LOW);
}

static s32 ATResponse_Handler(char* line, u32 len, void* userData)
{
    APP_DEBUG("[ATResponse_Handler] %s\r\n", (u8*)line);

    uart_write_data(m_myUartPort, (u8*)line);

    if (Ql_RIL_FindLine(line, len, "OK"))
    {
        return  RIL_ATRSP_SUCCESS;
    }
    else if (Ql_RIL_FindLine(line, len, "ERROR"))
    {
        return  RIL_ATRSP_FAILED;
    }
    else if (Ql_RIL_FindString(line, len, "+CME ERROR"))
    {
        return  RIL_ATRSP_FAILED;
    }
    else if (Ql_RIL_FindString(line, len, "+CMS ERROR:"))
    {
        return  RIL_ATRSP_FAILED;
    }

    return RIL_ATRSP_CONTINUE; //continue wait
}

/* report device information */
int iotx_report_devinfo(u8 tcp_id, u32 msg_id)
{
    int ret = 0;
    char topic_name[IOTX_URI_MAX_LEN + 1] = {0};
    char *msg = NULL;
    int  msg_len = 0;

    /* devinfo update topic name */
    Ql_snprintf(topic_name,
                IOTX_URI_MAX_LEN,
                DEV_INFO_UPDATE_TOPIC,
                productKey,
                deviceName);

    APP_DEBUG("topic_name: %s\r\n", topic_name);

    msg_len = Ql_strlen(DEVICE_INFO_UPDATE_FMT) + 10 + Ql_strlen(IOTX_SDK_VERSION) + 1;
    msg = (char *)Ql_MEM_Alloc(msg_len);
    if (msg == NULL)
    {
        APP_DEBUG("malloc memory fail!\r\n");
        return FAIL_RETURN;
    }

    /* devinfo update message */
    ret = Ql_snprintf(msg,
                      msg_len,
                      DEVICE_INFO_UPDATE_FMT,
                      iotx_report_id(),
                      IOTX_SDK_VERSION
                      );

    APP_DEBUG("devinfo report data: %s\r\n", msg);

    ret = RIL_MQTT_QMTPUB(tcp_id, msg_id, QOS1_AT_LEASET_ONCE, 0, topic_name, Ql_strlen(msg), msg);
    if (RIL_AT_SUCCESS == ret)
    {
        APP_DEBUG("<--Start publish messages to ali server-->\r\n");
        ret = SUCCESS_RETURN;

        m_mqtt_state = STATE_TOTAL_NUM;
    }
    else
    {
        APP_DEBUG("<--publish messages to ali server failure, error = %d.-->\r\n", ret);
        ret = FAIL_RETURN;
    }

    Ql_MEM_Free(msg);

    return ret;
}

/* report Firmware version */
int iotx_report_firmware_version(u8 tcp_id, u32 msg_id)
{
#define FIRMWARE_VERSION_MSG_LEN          (64)

    int ret;
    char topic_name[IOTX_URI_MAX_LEN + 1] = {0};
    char msg[FIRMWARE_VERSION_MSG_LEN] = {0};
    char version[IOTX_FIRMWARE_VERSION_LEN + 1] = {0};

    /* firmware report topic name generate */
    ret = Ql_snprintf(topic_name,
                      IOTX_URI_MAX_LEN,
                      FIRMWARE_VERSION_TOPIC,
                      productKey,
                      deviceName
                      );
    if (ret <= 0)
    {
        APP_DEBUG("firmware report topic generate err\r\n");
        return FAIL_RETURN;
    }
    APP_DEBUG("firmware report topic: %s\r\n", topic_name);

    GetFirmwareVersion(version);

    /* firmware report message json data generate */
    ret = Ql_snprintf(msg,
                      FIRMWARE_VERSION_MSG_LEN,
                      FIRMWARE_VERSION_MSG_FMT,
                      iotx_report_id(),
                      version
                      );
    if (ret <= 0)
    {
        APP_DEBUG("firmware report message json data generate err\r\n");
        return FAIL_RETURN;
    }
    APP_DEBUG("firmware report data: %s\r\n", msg);

    ret = RIL_MQTT_QMTPUB(tcp_id, msg_id, QOS1_AT_LEASET_ONCE, 0, topic_name, Ql_strlen(msg), msg);
    if (RIL_AT_SUCCESS == ret)
    {
        APP_DEBUG("<--Start publish messages to ali server-->\r\n");

        ret = SUCCESS_RETURN;
        m_mqtt_state = STATE_TOTAL_NUM;
    }
    else
    {
        APP_DEBUG("<--publish messages to ali server failure, error = %d.-->\r\n", ret);
        ret = FAIL_RETURN;
    }

    APP_DEBUG("firmware version report finished, iotx_publish() = %d\r\n", ret);

    return SUCCESS_RETURN;
}

/* report ModuleID */
int iotx_report_mid(u8 tcp_id, u32 msg_id)
{
    return SUCCESS_RETURN;
}

int iotx_report_id(void)
{
    return g_report_id++;
}

/**
 * @brief Get firmware version
 *
 * @param [ou] version: array to store firmware version, max length is IOTX_FIRMWARE_VER_LEN
 * @return the actual length of firmware version
 */
int GetFirmwareVersion(char *version)
{
    int len = Ql_strlen(APP_FIRMWARE_VERSION);
    if (len > IOTX_FIRMWARE_VERSION_LEN)
        return 0;

    Ql_memset(version, 0x0, IOTX_FIRMWARE_VERSION_LEN);
    Ql_strncpy(version, APP_FIRMWARE_VERSION, IOTX_FIRMWARE_VERSION_LEN);

    return len;
}

static void uart_write_data(char port, const char *buf)
{
    Ql_UART_Write((Enum_SerialPort)(port), (u8*)(buf), Ql_strlen((const char *)(buf)));
}

int example_publish(u8 tcp_id, u32 msg_id, char data_type, char *event_id, char *payload)
{
    int             ret = 0;
    char           *topic = NULL;
    int             topic_len = 0;

    if (data_type == PROPERTY_DATA)
    {
        topic_len = Ql_strlen(PROPERTY_POST_TOPIC) + Ql_strlen(productKey) + Ql_strlen(deviceName) + 1;
    }
    else if (data_type == EVENT_DATA)
    {
        topic_len = Ql_strlen(EVENT_POST_TOPIC) + Ql_strlen(productKey) + Ql_strlen(deviceName) + 1;
    }

    topic = (char *)Ql_MEM_Alloc(topic_len);
    if (topic == NULL)
    {
        APP_DEBUG("memory not enough\r\n");
        return -1;
    }

    Ql_memset(topic, 0, topic_len);
    if (data_type == PROPERTY_DATA)
    {
        Ql_snprintf(topic, topic_len, PROPERTY_POST_TOPIC, productKey, deviceName);
    }
    else if (data_type == EVENT_DATA)
    {
        Ql_snprintf(topic, topic_len, EVENT_POST_TOPIC, productKey, deviceName, event_id);
    }

    if (msg_id)
    {
        ret = RIL_MQTT_QMTPUB(tcp_id, msg_id, QOS1_AT_LEASET_ONCE, 0, topic, Ql_strlen(payload), payload);
    }
    else
    {
        ret = RIL_MQTT_QMTPUB(tcp_id, msg_id, QOS0_AT_MOST_ONECE, 0, topic, Ql_strlen(payload), payload);
    }

    if (RIL_AT_SUCCESS == ret)
    {
        APP_DEBUG("<--Start publish messages to ali server-->\r\n");

        ret = SUCCESS_RETURN;
        m_mqtt_state = STATE_TOTAL_NUM;
    }
    else
    {
        APP_DEBUG("<--publish messages to ali server failure, error = %d.-->\r\n", ret);
        ret = FAIL_RETURN;
    }

    Ql_MEM_Free(topic);

    return ret;
}

int example_user_post_property(u8 tcp_id, u32 msg_id, char *params_str)
{
    int             ret = 0;
    char           *payload = NULL;
    const short     payload_len = 256;

    payload = Ql_MEM_Alloc(payload_len);
    if (payload == NULL)
    {
        APP_DEBUG("memory not enough\r\n");
        return -1;
    }

    Ql_memset(payload, 0, payload_len);
    Ql_snprintf(payload, payload_len, PROPERTY_PAYLOAD_FMT, iotx_report_id(), params_str, PROPERTY_METHOD);

    APP_DEBUG("payload: %s payload_len: (%d)\r\n", payload, Ql_strlen(payload));

    ret = example_publish(tcp_id, msg_id, PROPERTY_DATA, NULL, payload);

    if (payload != NULL)
    {
        Ql_MEM_Free(payload);
    }

    return ret;
}

int example_user_post_event(u8 tcp_id, u32 msg_id, char *event_id, char *params_str)
{
    int             ret = 0;
    char           *payload = NULL;
    const short     payload_len = 256;

    payload = Ql_MEM_Alloc(payload_len);
    if (payload == NULL)
    {
        APP_DEBUG("memory not enough\r\n");
        return -1;
    }

    Ql_memset(payload, 0, payload_len);
    Ql_snprintf(payload, payload_len, PROPERTY_PAYLOAD_FMT, iotx_report_id(), params_str, EVENT_METHOD);

    APP_DEBUG("user post event, event name: %s\r\n", event_id);
    APP_DEBUG("payload: %s payload_len: (%d)\r\n", payload, Ql_strlen(payload));

    ret = example_publish(tcp_id, msg_id, EVENT_DATA, event_id, payload);

    if (payload != NULL)
    {
        Ql_MEM_Free(payload);
    }

    return ret;
}

u8 get_mqtt_topic_id(const char *topic_name)
{

	if (Ql_strstr(topic_name, "/thing/service/property/set") != NULL)
	{
		/*  Ù–‘œ¬∑¢ */
		return PROPERTY_SET;
	}
	else if((Ql_strstr(topic_name, "/thing/service/") != NULL) && (Ql_strstr(topic_name, "/thing/service/property/set") == NULL))
	{
		/* ∑˛ŒÒœ¬∑¢ */
		return SERVICE_DOWN;
	}
	else  if (Ql_strstr(topic_name, "/ota/device/upgrade") != NULL)
	{
		return OTA_UPGRADE;
	}
	else  if(Ql_strstr(topic_name, "/thing/model/down_raw") != NULL)
	{
		return RAW_DOWN_ID;
	}
	else
	{
		return TOPIC_MAX_ID;
	}
}

static s32 read_fromfs(const char *filepath, char *buf, s32 *len)
{
	static s32 handle = -1;
	s32 ret;
	s32  readLen;
	char read_buff[FILE_CONTENT_LEN];

	APP_DEBUG("\r\n<--filePath: %s-->\r\n", filepath);

	// read file
	memset(buf, 0, FILE_CONTENT_LEN);
	handle = Ql_FS_Open(filepath, QL_FS_READ_WRITE);

	if (handle < 0)
	{
		APP_DEBUG("\r\n<--The file does not exist!-->\r\n");
		ret = -1;

		return ret;
	}

	ret = Ql_FS_Seek(handle, 0, QL_FS_FILE_BEGIN);
	ret = Ql_FS_Read(handle, buf, FILE_CONTENT_LEN, &readLen);

	Ql_FS_Flush(handle); //fflush
	Ql_FS_Close(handle); //close the file

	APP_DEBUG("\r\n<--read content is (%s) len = %d-->\r\n", buf, readLen);

	*len = readLen;

	return ret;
}

static s32 write_tofs(const char *filepath, const char *buf, s32 *len)
{
	static s32 handle = -1;
	s32 ret;
	s32 writenLen, readLen;
	char read_buff[FILE_CONTENT_LEN];
	s64  space = 0;

	APP_DEBUG("\r\<--filepath: %s-->\r\n", filepath);

	Enum_FSStorage storage = Ql_FS_UFS;
	APP_DEBUG("\r\n<--OpenCPU: FILE(UFS) TEST!-->\r\n");

#if 0
	//format
    ret = Ql_FS_Format(storage);
    APP_DEBUG("\r\n<-- Ql_FS_Format ret = %d -->\r\n", ret); 
#endif

	//check freespace
    space  = Ql_FS_GetFreeSpace(storage);
    APP_DEBUG("\r\n<--Ql_FS_GetFreeSpace Bytes = %lld-->\r\n", space);

    //check total space
    space = Ql_FS_GetTotalSpace(storage);
    APP_DEBUG("\r\n<--Ql_FS_GetTotalSpace Bytes = %lld-->\r\n", space);

	//delete file
	ret = Ql_FS_Delete(filepath);

	//create file
	handle = Ql_FS_Open(filepath, QL_FS_CREATE_ALWAYS); //if file does not exist ,create it
	if (handle > 0)
	{
		APP_DEBUG("\r\n<-- Create file (%s) OK! handle = %d -->\r\n", filepath, handle);
	}
	else
	{
		APP_DEBUG("\r\n<-- failed!! Create file (%s) fail-->", filepath);
		ret = -1;

		return -1;
	}

	//write file
	APP_DEBUG("\r\n<--Please write:-->\r\n");

	writenLen = strlen(buf);

	APP_DEBUG("\r\n<--write content is (%s) len = %d-->\r\n", buf, writenLen);

	ret = Ql_FS_Seek(handle, 0, QL_FS_FILE_BEGIN); //seek end
	APP_DEBUG("\r\n<--!! Ql_FS_Seek ret = %d  writenLen = %d-->\r\n", ret, writenLen);
	ret = Ql_FS_Write(handle, buf, Ql_strlen(buf), &writenLen);
	APP_DEBUG("\r\n<--!! Ql_FS_Write ret = %d writenLen = %d-->\r\n", ret, writenLen);

	Ql_FS_Flush(handle); //fflush
	Ql_FS_Close(handle); //close the file

	// read file
	memset(read_buff, 0, FILE_CONTENT_LEN);
	handle = Ql_FS_Open(filepath, QL_FS_READ_WRITE);

	if (handle < 0)
	{
		APP_DEBUG("\r\n<--The file does not exist!-->\r\n");
		ret = -1;

		return ret;
	}

	ret = Ql_FS_Seek(handle, 0, QL_FS_FILE_BEGIN);
	ret = Ql_FS_Read(handle, read_buff, FILE_CONTENT_LEN, &readLen);

	Ql_FS_Flush(handle); //fflush
	Ql_FS_Close(handle); //close the file

	APP_DEBUG("\r\n<--read content is (%s) len = %d-->\r\n", read_buff, readLen);

	if (!Ql_strcmp(buf, read_buff))
	{
		APP_DEBUG("\r\n<--write content (%s) is OK!-->\r\n", read_buff);
		ret = 0;
		*len = writenLen;
	}
	else
	{
		APP_DEBUG("\r\n<--write content (%s) is failed!-->\r\n", read_buff);
		ret = 0;
	}

	return ret;
}

static s32 read_device_info_fromfs(char *buf, s32 *len)
{
	u8 filePath[128] = {0};
	s32 ret;

	Ql_memset(filePath, 0, 128);
    Ql_sprintf(filePath, "%s\\%s\\%s\0", PATH_ROOT, DEV_INFO_DIR, DEV_INFO_FILE);

	ret = read_fromfs(filePath, buf, len);

	return ret;
}

static s32 write_device_info_tofs(const char *buf, s32 *len)
{
	u8 filePath[128] = {0};
	s32 ret;

	Ql_memset(filePath, 0, 128);
    Ql_sprintf(filePath, "%s\\%s\\%s\0", PATH_ROOT, DEV_INFO_DIR, DEV_INFO_FILE);

	ret = write_tofs(filePath, buf, len);

	return ret;
}

static s32 read_productsecret_fromfs(char *buf, s32 *len)
{
	u8 filePath[128] = {0};
	s32 ret;

	Ql_memset(filePath, 0, 128);
    Ql_sprintf(filePath, "%s\\%s\\%s\0", PATH_ROOT, DEV_INFO_DIR, PRODUCT_SECRET_FILE);

	ret = read_fromfs(filePath, buf, len);

	return ret;

}

static s32 write_productsecret_tofs(const char *buf, s32 *len)
{
	u8 filePath[128] = {0};
	s32 ret;

	Ql_memset(filePath, 0, 128);
    Ql_sprintf(filePath, "%s\\%s\\%s\0", PATH_ROOT, DEV_INFO_DIR, PRODUCT_SECRET_FILE);

	ret = write_tofs(filePath, buf, len);

	return ret;

}
static s32 read_run_through_flag_fromfs(const char *buf, s32 *len)
{
	u8 filePath[128] = {0};
	s32 ret;

	Ql_memset(filePath, 0, 128);
    Ql_sprintf(filePath, "%s\\%s\\%s\0", PATH_ROOT, RUN_THROUGH_FLAG_DIR, RUN_THROUGH_FLAG_FILE);

	ret = read_fromfs(filePath, buf, len);

	return ret;

}

static s32 write_run_through_flag_tofs(const char *buf, s32 *len)
{
	u8 filePath[128] = {0};
	s32 ret;

	Ql_memset(filePath, 0, 128);
    Ql_sprintf(filePath, "%s\\%s\\%s\0", PATH_ROOT, RUN_THROUGH_FLAG_DIR, RUN_THROUGH_FLAG_FILE);

	ret = write_tofs(filePath, buf, len);

	return ret;

}

	

int iotx_report_device_imei_info(void)
{
	u8 payload[64];
	int ret;

	memset(payload, 0, 64);

	sprintf(payload, "{\"MACAddress\":\"%s\"}", strImei);
	APP_DEBUG("\r\n<-- payload: %s -->\r\n", payload);

	ret = example_user_post_property(connect_id, pub_message_id++, payload);
	if (ret == 0)
	{
		return 0;
	}
	else
	{
		return -1;
	}
}

s32 RIL_MQTT_QMTCFG_VERSION(u8 tcp_id, u8 version)
{
    s32 ret = RIL_AT_FAILED;
    char strAT[200];

    Ql_memset(strAT, 0, sizeof(strAT));
    Ql_sprintf(strAT, "AT+QMTCFG=\"VERSION\",%d,%d\r\n", tcp_id, version);

    ret = Ql_RIL_SendATCmd(strAT, Ql_strlen(strAT), ATResponse_Handler, NULL, 0);

    return ret;
}

s32 RIL_MQTT_QMTCFG_KEEPALIVE(u8 tcp_id, u16 keep_alive_second)
{
    s32 ret = RIL_AT_FAILED;
    char strAT[200];

    Ql_memset(strAT, 0, sizeof(strAT));
    Ql_sprintf(strAT, "AT+QMTCFG=\"KEEPALIVE\",%d,%d\r\n", tcp_id, keep_alive_second);

    ret = Ql_RIL_SendATCmd(strAT, Ql_strlen(strAT), ATResponse_Handler, NULL, 0);

    return ret;
}

/*
 * pkt_timeout: 0-60s
 * retry_times: 0-10
 * timeout_notice_flag:
 *                     0 - Not report timeout message when transmitting packet
 *                     1 - Report timeout message when transmitting packet
 */
s32 RIL_MQTT_QMTCFG_TIMEOUT(u8 tcp_id, u8 pkt_timeout, u8 retry_times, u8 timeout_notice_flag)
{
    s32 ret = RIL_AT_FAILED;
    char strAT[200];

    Ql_memset(strAT, 0, sizeof(strAT));
    Ql_sprintf(strAT, "AT+QMTCFG=\"TIMEOUT\",%d,%d,%d,%d\r\n", tcp_id, pkt_timeout, retry_times, timeout_notice_flag);

    ret = Ql_RIL_SendATCmd(strAT, Ql_strlen(strAT), ATResponse_Handler, NULL, 0);

    return ret;
}


void printf_mqtt_conn_result(u8 code)
{
	switch (code)
	{
		case 0:
			APP_DEBUG("Connection Accepted\r\n");
			break;
		case 1:
			APP_DEBUG("Connection Refused: Unacceptable Protocol Version\r\n");
			break;
		case 2:
			APP_DEBUG("Connection Refused: Identifier Rejected\r\n");
			break;
		case 3:
			APP_DEBUG("Connection Refused: Server Unavailable\r\n");
			break;
		case 4:
			APP_DEBUG("Connection Refused: Bad User Name or Password\r\n");
			break;
		case 5:
			APP_DEBUG("Connection Refused: Not Authorized\r\n");
			break;
		default:
			break;
	}
}

#endif // __ALI_MQTT_EXAMPLE__

