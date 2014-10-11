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
const int step = 23;
const double step_rad = static_cast<double>(step)*0.1f*M_PI/180.0f;

class FutabaDriver {
	private :
		tf::TransformBroadcaster laser_tilt_pub;
		tf::TransformBroadcaster laser_offset_pub;
		Futaba30x servo;
		ros::Rate loop_rate;
		int  step_cnt;
		bool is_up;

	public :
		FutabaDriver(ros::NodeHandle &node, int fd) :
			servo(fd, 1), loop_rate(20), step_cnt(0), is_up(true)
	{
		servo.torque_on();
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
		}

	private :
		void PublishLaserTf() {
			// 現在のサーボの角度を通知
			laser_tilt_pub.sendTransform(
					tf::StampedTransform(
						tf::Transform(tf::Quaternion(-step_cnt*step_rad, 0, 0), tf::Vector3(0.0, 0.0, 0.675)),
						ros::Time::now(),"body", "laser_link"
						)
					);
			// サーボの回転中心と，URGのオフセット FIXME 毎回更新しなくてもいいデータ
			laser_offset_pub.sendTransform(
					tf::StampedTransform(
						tf::Transform(tf::Quaternion(0, 0, 0), tf::Vector3(0.0, 0.0, 0.09)), // TODO オフセットの実際の値
						ros::Time::now(),"laser_link", "rear_laser"
						)
					);
		}
		// 現在のURGのスキャンデータを，laser_assemblerに対して発行するよう要求
		void MoveToNextAngle() {
			if (step_cnt>=20 && is_up) is_up = false;
			else if (step_cnt<=0 && !is_up)  is_up = true;
			else step_cnt += is_up ? 1 : -1;
			servo.move(step*step_cnt, 5);
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

