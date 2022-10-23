#include "crc.h"



unsigned int CRC16_2_modbus(unsigned char *buf, int len)
{
    static unsigned int crc ;
	  crc = 0xFFFF;
    for (int pos = 0; pos < len; pos++)
    {
        crc ^= (unsigned int)buf[pos]; 
        for (int i = 8; i != 0; i--)   
        {
            if ((crc & 0x0001) != 0)   
            {
                crc >>= 1; 
                crc ^= 0xA001;
            }
            else // Else LSB is not set
            {
                crc >>= 1;   
            }
        }
    }
 
    //crc = ((crc & 0x00ff) << 8) | ((crc & 0xff00) >> 8);
    return crc;
}


