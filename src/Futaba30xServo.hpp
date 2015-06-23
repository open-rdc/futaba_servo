/*!
 *  @file	Futaba30xServo.hpp
 *  @brief	 
 *
 *  @date	2012/11/07
 *  @author	taki4416
 *
 */

#ifndef FUTABA30XSERVO_HPP_
#define FUTABA30XSERVO_HPP_

class Futaba30x {
public:
	Futaba30x ( int fd, unsigned char id )
			: fd(fd), id(id)
	{
	}
	int move ( unsigned int deg, unsigned int time_10ms, unsigned char flag = 0x00 );
	int torque_on ( unsigned char flag = 0x00 );
	int torque_off ( unsigned char flag = 0x00 );
	short get_angle();
	~Futaba30x ()
	{
    close(fd);
	}
private:
	int fd;
	unsigned char id;
};

#endif /* FUTABA30XSERVO_HPP_ */
