#include <ros/ros.h>

#include <tf/transform_broadcaster.h>

#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "Futaba30xServo.hpp"

// todo rosparam
const int step = 40;
const double step_rad = static_cast<double>(step)*0.1f*M_PI/180.0f;

class FutabaDriver {
	private :
		tf::TransformBroadcaster laser_tilt_pub;
		tf::Transform laser_tilt;
		tf::TransformBroadcaster laser_offset_pub;
		tf::Transform laser_offset;
		ros::Rate loop_rate;
		Futaba30x servo;
		int  step_cnt;
		bool is_up;

	public :
		FutabaDriver(ros::NodeHandle &node, int fd) :
			servo(fd, 1), loop_rate(10), step_cnt(0), is_up(true)
	{
		servo.torque_on();
		laser_tilt.setOrigin( tf::Vector3(0.0,0.0,0.8) );
		laser_offset.setOrigin( tf::Vector3(0.0,0.0,0.085) );
		laser_offset.setRotation( tf::Quaternion(0.0, 0.0, 0.0, 0.0) );
	}
		void run() {
			while (ros::ok()) {
				// 現在のtfデータ発行
				PublishLaserTf();
				// サーボ動かす
				MoveToNextAngle();
				// ros::spinOnce();
				loop_rate.sleep();
			}
			servo.torque_off();
		}

	private :
		void PublishLaserTf() {
			laser_tilt.setRotation( tf::Quaternion(-step_cnt*step_rad,0.0,0.0, 0.0) );
			// 現在のサーボの角度を通知
			laser_tilt_pub.sendTransform(
					tf::StampedTransform(
						laser_tilt, 
						ros::Time::now(), 
						"hokuyo_link",
						"laser_link"
					)
			);
			laser_offset_pub.sendTransform(
					tf::StampedTransform(
						laser_offset,
						ros::Time::now(),
						"laser_link",
						"rear_laser"
					)
			);
		}
		// 現在のURGのスキャンデータを，laser_assemblerに対して発行するよう要求
		void MoveToNextAngle() {
			if (step*step_cnt>=400 && is_up) is_up = false;
			else if (step*step_cnt<=-400 && !is_up)  is_up = true;
			else step_cnt += is_up ? 1 : -1;
			ROS_INFO_STREAM("step_is"<<step_cnt*step);
			servo.move(step*step_cnt, 10);
		}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "servo_driver");
	ros::NodeHandle node;

	struct termios oldtio, newtio; /* シリアル通信設定 */
	// todo  変更可能に
	int servo_fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY); /* デバイスをオープンする */
	ioctl(servo_fd, TCGETS, &oldtio); /* 現在のシリアルポートの設定を待避させる */
	newtio = oldtio; /* ポートの設定をコピー */
	// 通信設定
	newtio.c_cc[VMIN] = 1; // 最低一文字送受信
	newtio.c_cc[VTIME] = 0;
	// キャラクラビット8, 受信可能に，制御信号無視
	newtio.c_cflag = CS8 | CREAD | CLOCAL;
	// ブレーク信号無視， パリティ無視
	newtio.c_iflag = IGNBRK | IGNPAR;
	int ret = cfsetspeed(&newtio, B460800);
	if (ret < 0) {
		ROS_INFO("Error in cfsetspeed");
		close(servo_fd);
		return 1;
	}
	ioctl(servo_fd, TCSETS, &newtio);

	FutabaDriver futaba_driver(node, servo_fd);
	futaba_driver.run();
	return 0;
}

