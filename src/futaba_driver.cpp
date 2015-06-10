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
		Futaba30x servo;
		tf::TransformBroadcaster laser_tilt_pub;
		tf::TransformBroadcaster laser_offset_pub;
		ros::Rate loop_rate;
		int  step_cnt;
		bool is_up;
		bool is_round;
		double min_deg;
		double max_deg;
		int round_time;

	public :
		FutabaDriver(ros::NodeHandle &node, int fd) :
			servo(fd,1),loop_rate(20), step_cnt(0), is_up(true),
			is_round(true),min_deg(0),max_deg(0),round_time(100)
		{
			ros::NodeHandle private_nh("~");
			private_nh.param<bool>("is_round", is_round, is_round);
			private_nh.param<double>("max_deg", max_deg, max_deg);
			private_nh.param<double>("min_deg", min_deg, min_deg);
			private_nh.param<int>("rount_time_10ms", round_time, round_time);
		}

		void run() {
			int now_deg;
			servo.torque_on();
			if(is_round){
				servo.move((int)max_deg * 10, 100);
				now_deg = max_deg;
				is_up = true;
			}
			while (ros::ok()) {
				if(is_round){
					MoveRoundTrip();
					if(is_up){
						while( now_deg >= min_deg ){
							now_deg -= ((max_deg + min_deg) / round_time * 10) * (loop_rate.expectedCycleTime().toSec() / 1000);
						}
					}
					else{
						while( now_deg <= max_deg ){
							now_deg += ((max_deg + min_deg) / round_time * 10) * (loop_rate.expectedCycleTime().toSec() / 1000);
						}
					}
					PublishLaserTf(now_deg * 2 * M_PI);
					loop_rate.sleep();
				}
			}
			servo.torque_off();
		}

	private :
		void PublishLaserTf(double rad) {
			// 現在のサーボの角度を通知
			laser_tilt_pub.sendTransform(
					tf::StampedTransform(
						tf::Transform(tf::Quaternion(-rad, 0, 0), tf::Vector3(0.0, 0.0, 0.675)),
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

		void MoveRoundTrip(){
			int iMax_deg = max_deg * 10;
			int iMin_deg = min_deg * 10;
			if(is_up){
				servo.move(iMin_deg, round_time);
				is_up = false;
			}
			else{
				servo.move(iMax_deg, round_time);
				is_up = true;
			}
		}

		void MoveToNextAngle() {
			if (step_cnt>=20 && is_up) is_up = false;
			else if (step_cnt<=-20 && !is_up)  is_up = true;
			else step_cnt += is_up ? 1 : -1;
			servo.move(step*step_cnt, 5);
		}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "servo_driver");
	ros::NodeHandle node;
	ros::NodeHandle private_nh("~");

	std::string port_name;
	port_name = "/dev/ttyUSB0";
	private_nh.getParam("port_name",port_name);

	/* シリアル通信設定 */
	struct termios oldtio, newtio;
	int servo_fd = open(port_name.c_str(), O_RDWR | O_NOCTTY); /* デバイスをオープンする */
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
		std::cout << "Error in cfsetspeed" << std::endl;
		close(servo_fd);
		return 0;
	}
	ioctl(servo_fd, TCSETS, &newtio);

	FutabaDriver futaba_driver(node, servo_fd);
	
	futaba_driver.run();
	return 0;
}

