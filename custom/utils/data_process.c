

#include "data_process.h"

/*
 * This function look for the postion of comma on the buffer
 * @param: buff: the buffer that will look for
 * @param: cx: the index of comma what we will to find
 * @return: the postion of comma
 */
char seek_comma_pos(const char *buf, char cx)
{	 		    
    char *p = buf;

    while (cx)
    {
        if (*buf == ',')
            cx--;
		if (*buf == '\0')
			return 0xff;

        buf++;
    }

    return buf - p - 1;	 
}

int get_device_info(const char *buf, char *productkey, char *devicename, char *devicesecret)
{
	char start_pos, end_pos;
	int i, j;
	int ret;

	if (buf == 0)
	{
		return -1;
	}

	start_pos = 0;
	end_pos = seek_comma_pos(buf, 1);
	if (end_pos != 0xff)
	{
		for (i = start_pos, j = 0; i < end_pos; i++)
		{
			productkey[j++] = buf[i];
		}

		productkey[j] = '\0'; 
	}

	start_pos = end_pos;
	end_pos = seek_comma_pos(buf, 2);
	if (end_pos != 0xff)
	{
		for (i = start_pos + 1, j = 0; i < end_pos; i++)
		{
			devicename[j++] = buf[i];
		}

		devicename[j] = '\0'; 
	}

	start_pos = end_pos;
	for (i = start_pos + 1, j = 0; buf[i] != 0; i++)
	{
		devicesecret[j++] = buf[i];
	}

	devicesecret[j] = '\0';

	if (productkey != 0 && devicename != 0 && devicesecret != 0)
	{
		ret = 0;
	}
	else
	{
		ret = -1;
	}

	return ret;
}

int get_product_secret(const char *buf, char *productsecret)
{
	int i;
	int ret;

	if (buf == 0)
	{
		return -1;
	}

	for (i = 0; buf[i] != 0; i++)
	{
		productsecret[i] = buf[i];
	}

	productsecret[i] = '\0';

	if (productsecret != 0)
	{
		ret = 0;
	}
	else
	{
		ret = -1;
	}

	return ret;
}

int get_event_name(const char *buf, char *event_name)
{
	char start_pos, end_pos;
	int i, j;
	int ret;

	start_pos = seek_comma_pos(buf, 1);
	end_pos = seek_comma_pos(buf, 2);

	if (start_pos != 0xff)
	{
		for (i = start_pos + 1, j = 0; i < end_pos; i++)
		{
			event_name[j++] = buf[i];
		}

		event_name[j] = '\0'; 

		if (event_name != 0)
		{
			ret = 0;
		}
		else
		{
			ret = -1;
		}
	}
	else
	{
		ret = -1;
	}

	return ret;
}

int get_paylaod(char data_type, const char *buf, char *payload)
{
	char start_pos;
	int i, j;
	int ret;

	if (data_type == 0)
	{
		start_pos = seek_comma_pos(buf, 1);
		if (start_pos != 0xff)
		{
			for (i = start_pos + 1, j = 0; buf[i] != '\0' && buf[i] != '\r' && buf[i] != '\n'; i++)
			{
				payload[j++] = 	buf[i];
			}

			ret = 0;
		}
		else
		{
			ret = -1;
		}
	}
	else if (data_type == 1)
	{
		start_pos = seek_comma_pos(buf, 2);
		if (start_pos != 0xff)
		{
			for (i = start_pos + 1, j = 0; buf[i] != '\0' && buf[i] != '\r' && buf[i] != '\n'; i++)
			{
				payload[j++] = 	buf[i];
			}
		}
		else
		{
			ret = -1;
		}
	}

	payload[j] = '\0';

	if (payload != 0)
	{
		ret = 0;
	}

	return ret;
}



