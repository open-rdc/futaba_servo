/*!
 *  @file	Futaba30xServo.cpp
 *  @brief	 
 *
 *  @date	2012/11/07
 *  @author	taki4416
 *
 */

#include <iostream>
#include <unistd.h>
#include "Futaba30xServo.hpp"

int Futaba30x::move ( unsigned int deg, unsigned int time_10ms, unsigned char flag )
{
	unsigned char data[12] = { 0xFA, 0xAF, id, flag, 0x1E,
			0x04, 0x01, deg&0x00FF, (deg&0xFF00)>>8,
			time_10ms&0x00FF, (time_10ms&0xFF00)>>8, id };
	if (deg < 0) {
		deg = -deg - 1;
		data[7] = ~(deg & 0x00FF);
		data[8] = ~((deg & 0xFF00) >> 8);
	}
	for (int i = 3; i < 11; ++i) {
		data[11] ^= data[i];
	}
  return write(fd, data, 12);
}

int Futaba30x::torque_on ( unsigned char flag )
{
	unsigned char data[9] = { 0xFA, 0xAF, id, flag, 0x24,
			0x01, 0x01, 0x01, id };
	for (int i = 3; i < 8; i++) {
		data[8] = data[8]^data[i];
	}
  return write(fd, data, 9);
}

int Futaba30x::torque_off ( unsigned char flag )
{
	unsigned char data[9] = { 0xFA, 0xAF, id, flag, 0x24,
			0x01, 0x01, 0x00, id };
	for (int i = 3; i < 8; i++) {
		data[8] = data[8]^data[i];
	}
  return write(fd, data, 9);
}

int Futaba30x::get_angle()
{
	unsigned char send_data[9] = { 0xFA, 0xAF, id, 0x09, 0x00, 0x00, 0x01, 0x09};
	unsigned char recv_data[26];
	int nread;
	write(fd, send_data, 9);
	nread = read(fd, recv_data, 26);
	if (nread == -1)
	{
		return 0xFF;
	}
	return recv_data[7] + (recv_data[8]>>8);
}

