
#ifndef __DATA_PROCESS_H
#define __DATA_PROCESS_H

/*****************************************************************
* define net state
******************************************************************/
typedef enum{
	STATE_NET_DISCONNECT,
    STATE_NET_CONNECT,
    STATE_SERVER_DISCONNECT,
    STATE_SERVER_CONNECT,
}Enum_NETSTATE;

enum DATA_TYPE {
	PROPERTY_DATA,
	EVENT_DATA
};

char seek_comma_pos(const char *buf, char cx);
int get_product_secret(const char *buf, char *productsecret);
int get_device_info(const char *buf, char *productkey, char *devicename, char *devicesecret);
int get_event_name(const char *buf, char *event_name);
int get_paylaod(char data_type, const char *buf, char *payload);


#endif

