#include "ros/ros.h"
#include <math.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Empty.h"
#include "Eigen/Dense"
#include "ardrone_autonomy/Navdata.h"
#include "ardrone_autonomy/navdata_altitude.h"
#include "ardrone_control/ROI.h"
#include "ardrone_control/ROINumber.h"
#include <iostream>

#define LOOP_RATE 20
#define WAIT_TIME 60
#define NORMAL_HEIGHT 2.0
#define FIND_HEIGHT 2.5
#define THR_HEIGHT  2.2

// #define NORMAL_HEIGHT 1.5
// #define FIND_HEIGHT 2.0
// #define THR_HEIGHT  1.7

#define SEARCH_SIZE 0.3
//#define TAKEOFF_OFFSET 0.2
using namespace std;
using namespace Eigen;

bool first_pos_received = false;
bool first_yaw_received = false;

int constrain(int a, int b, int c){return ((a)<(b)?(b):(a)>(c)?c:a);}
int dead_zone(int a, int b){return ((a)>(b)?(a):(a)<(-b)?(a):0);}
float constrain_f(float a, float b, float c){return ((a)<(b)?(b):(a)>(c)?c:a);}
float dead_zone_f(float a, float b){return ((a)>(b)?(a):(a)<(-b)?(a):0);}
int minimum(int a, int b){return (a>b?b:a);}
int maximum(int a, int b){return (a>b?a:b);}
int absolute(int a){return (a>0?a:-a);}
float absolute_f(float a){return (a>0?a:-a);}

Vector3f image_pos;
Vector3f image_pos_pre;
ros::Time image_stamp;
ros::Time number_stamp;
Vector3f landed_pos(0,0,0);
int number;
int number_last = 12;

int counter_after_takeoff = 0;
int counter_before_landing = 0;
int takeoff_counter = 0;

int move_flag1 = 0;
int move_flag2 = 0;
float move_start_x = 0;
float move_start_y = 0;

float height_sp = 2.2;    //add by CJ

//const float dists[10][2] = {{0, 0}, {82, -178}, {28, 328}, {66, -161}, {190, -156}, {284, 236}, {-282, 53}, {487, -10}, {-53, -271}, {-347, 105}};
float coord[10][10][2];


class num_flight
{
	#define STATE_IDLE 0
	#define STATE_TAKEOFF 1
	#define STATE_ACCURATE_AFTER_TAKEOFF 2
	#define STATE_INACCURATE 3
	#define STATE_ACCURATE_BEFORE_LANDING 4
	#define STATE_LANDING 5
	#define STATE_ON_GROUND 6
	#define STATE_ALT 7
	#define STATE_FIND_NUM_AFTER_TAKEOFF 8
	#define STATE_FIND_NUM_BEFORE_LANDING 9
	#define STATE_FIND_NUM_AFTER_TAKEOFF_MOVE 10
	#define STATE_FIND_NUM_BEFORE_LANDING_MOVE 11

public:
	unsigned char _state;//idle, or inaccurate pos_ctrl, or accurate image_ctrl
	Matrix<float, Dynamic, 2> relative_landpos_w;
	unsigned char _current_target;
	num_flight();
	~num_flight();
	bool idle_control		(Vector3f& vel_sp);
	bool inaccurate_control	(const Vector3f& _pos_sp, const Vector3f& _pos, Vector3f& vel_sp);
	bool accurate_control	(const Vector3f& _image_pos, float pos_z, Vector3f& vel_sp);
	bool altitude_change	(const Vector3f& _pos_sp, const Vector3f& _pos, Vector3f& vel_sp);

private:
	
};

class States
{
public:
	Matrix<float, 3, 3> R_field;
	Matrix<float, 3, 3> R_body;
	
	Vector3f acc_b;
	Vector3f vel_b;
	Vector3f pos_b;
	Vector3f pos_i;
	Vector3f pos_w;
	ros::Time navdata_stamp;
	int drone_state;
	float dt;
	float init_yaw;
	float yaw;

	ros::Publisher pose_body_pub;
	ros::Publisher pose_world_pub;

	void get_R_body(float yaw);
	States();
	~States();
	void inertial_filter_predict(float dt, float x[2], float acc);
	void inertial_filter_correct(float e, float dt, float x[2], char i, float w);
private:
	ros::NodeHandle n;
	ros::Subscriber states_sub;
	ros::Subscriber nav_sub;
	ros::Subscriber altitude_sub;
	//ros::Subscriber odometry_sub;
	void navCallback(const ardrone_autonomy::Navdata &msg);
	void altitudeCallback(const ardrone_autonomy::navdata_altitude &msg);
	//void odometryCallback(const nav_msgs::Odometry &msg);
};

num_flight::num_flight()
{
	Matrix<float, 3, 3> Rf;
	float world_field_yaw = -85/57.3;
	Rf(0,0) = cos(world_field_yaw);
	Rf(0,1) = sin(-world_field_yaw);
	Rf(0,2) = 0;
	Rf(1,0) = sin(world_field_yaw);
	Rf(1,1) = cos(world_field_yaw);
	Rf(1,2) = 0;
	Rf(2,0) = 0;
	Rf(2,1) = 0;
	Rf(2,2) = 1;
	Matrix<float, Dynamic, 3> relative_pos_field;
	relative_pos_field.resize(9,3);
	Matrix<float, 3, Dynamic> relative_pos_world;
	relative_pos_world.resize(3,9);
	relative_pos_field<<
	1.6, -1.5,0,
	0.15,3.2,0,
	0.7,-1.6,0,
	1.85,-1.55,0,
	2.95,2.2,0,
	-2.78,0.8,0,
	4.83,0.0,0,
	-0.6,-2.65,0,
	-3.35,1.05,0;

	_state = STATE_IDLE;
	_current_target=0;
	relative_pos_world = Rf*relative_pos_field.transpose();
	relative_landpos_w.resize(9,2);
	for(int i = 0; i < 9; i++){
		relative_landpos_w(i,0) = relative_pos_world(0,i);
		relative_landpos_w(i,1) = relative_pos_world(1,i);
	}

}

num_flight::~num_flight()
{
}

bool num_flight::idle_control(Vector3f& vel_sp)
{
	for(int i = 0; i < 3; i++)
		vel_sp(i) = 0;
	return false;
}

bool num_flight::inaccurate_control(const Vector3f& _pos_sp, const Vector3f& _pos, Vector3f& vel_sp)
{
	float speed = 0.08;
	bool is_arrived;
	Vector3f err = _pos_sp - _pos;
	err(2) = 0;
	float dist = err.norm();
	if(dist > 0.8){
		Vector3f direction = err / dist;
		vel_sp = direction * speed;
		is_arrived = false;
	//	ROS_INFO("\npos(%f, %f)\nsetpt(%f, %f)\nvelsp(%f, %f)\n",_pos(0),_pos(1),_pos_sp(0),_pos_sp(1),vel_sp(0),vel_sp(1));
	}else if(dist > 0.3)
	{
		ros::Duration dur;
		dur = ros::Time::now() - image_stamp;
		if(dur.toSec()< 0.5){
			is_arrived = true;
		}else{
			Vector3f direction = err / dist;
			vel_sp = direction * speed;
			is_arrived = false;
		}
	}
	else{
		is_arrived = true;
	}

	return is_arrived;
}

bool num_flight::accurate_control(const Vector3f& image_pos, float pos_z, Vector3f& vel_sp)
{
	static Vector2f err_last;
	static Vector2f err_int;
	static bool new_start = true;
	float P_pos = 0.00015 ,D_pos = 0.00007, I_pos = 0.0;
	Vector2f vel_sp_2d;
	Vector2f image_center(320.0,180.0);
	Vector2f image_pos_2d;
	image_pos_2d(0) = image_pos(0);
	image_pos_2d(1) = image_pos(1);
	Vector2f err = image_pos_2d - image_center;
	if(new_start){
		err_last = err;
		err_int(0) = 0;
		err_int(1) = 0;
		new_start = false;
	}
	Vector2f err_d = (err - err_last) * LOOP_RATE;
	vel_sp_2d = err * P_pos + err_d * D_pos + err_int * I_pos;
	
	vel_sp(0) = -vel_sp_2d(1);
	vel_sp(1) = -vel_sp_2d(0);
	vel_sp(2) = 0;
	if(vel_sp(0)>0.08)vel_sp(0)=0.08;
	if(vel_sp(0)<-0.08)vel_sp(0)=-0.08;
	if(vel_sp(1) >0.08)vel_sp(1) =0.08;
	if(vel_sp(1) <-0.08)vel_sp(1) =-0.08;
	
	bool is_arrived;
	float dist = err.norm();
	if(dist > 30.0){
		is_arrived = false;
	}

	else{
		is_arrived = true;
		err_int(0)=0;
		err_int(1)=0;
	}
	err_last = err;
	err_int += err / LOOP_RATE;
	return is_arrived;
}

bool num_flight::altitude_change(const Vector3f& _pos_sp, const Vector3f& _pos, Vector3f& vel_sp)
{
	float speed = 0.4;
	bool is_arrived;
	float err = _pos_sp(2) - _pos(2);
	float dist = absolute_f(err);
	// if(dist > 0.1){
	// 	float direction = err / dist;
	// 	vel_sp(2) = direction * speed;
	// 	is_arrived = false;
	// }
	// else{
	// 	is_arrived = true;
	// }
	if(_pos(2) < 0.0001){
 		vel_sp(2) = -0.2;
 	}else{
 		if(dist > 0.2){
 			float direction = err / dist;
 			vel_sp(2) = direction * speed;
 			is_arrived = false;
 		}
 		else{
 			is_arrived = true;
 		}
  	}
	return is_arrived;
}


States::States()
{
	nav_sub = n.subscribe("/ardrone/navdata", 1, &States::navCallback,this);
	altitude_sub = n.subscribe("/ardrone/navdata_altitude", 1, &States::altitudeCallback,this);
	//odometry_sub = n.subscribe("/ardrone/odometry", 1, &States::odometryCallback,this);
	pose_body_pub = n.advertise<geometry_msgs::PoseStamped>("/ardrone/position_body", 1);
	pose_world_pub = n.advertise<geometry_msgs::PoseStamped>("/ardrone/position_world", 1);
	for(int i=0;i<3;i++){
		pos_i(i) = 0;
		pos_b(i) = 0;
		pos_w(i) = 0;
	}
}
States::~States()
{
}

void States::get_R_body(float yaw)
{
	R_body(0,0) = cos(yaw);
	R_body(0,1) = sin(-yaw);
	R_body(0,2) = 0;
	R_body(1,0) = sin(yaw);
	R_body(1,1) = cos(yaw);
	R_body(1,2) = 0;
	R_body(2,0) = 0;
	R_body(2,1) = 0;
	R_body(2,2) = 1;
}

void States::inertial_filter_predict(float dt, float x[2], float acc)
{
	x[0] += x[1] * dt + 0.5 * acc * dt * dt;
	x[1] = acc * dt;
}
void States::inertial_filter_correct(float e, float dt, float x[2], char i, float w)
{
	float ewdt = e * w * dt;
	x[i] += ewdt;
	if (i == 0){
		x[1] += w * ewdt;
	}
}

void States::navCallback(const ardrone_autonomy::Navdata &msg)
{
	geometry_msgs::PoseStamped body_pose;
	geometry_msgs::PoseStamped world_pose;
	Vector3f raw_v ;
	Vector3f raw_a ;
	static float last_time = 0;
	raw_v(0) = msg.vx;
	raw_v(1) = msg.vy;
	raw_v(2) = msg.vz;
	raw_a(0) = msg.ax;
	raw_a(1) = msg.ay;
	raw_a(2) = msg.az;
	dt = (msg.tm - last_time)/1000000.0;
	last_time = msg.tm;
	acc_b = raw_a;
	pos_i += raw_v * dt / 1000.0;
	
	navdata_stamp = msg.header.stamp;

	yaw = (msg.rotZ) / 57.3;
	get_R_body(yaw);
	first_yaw_received = true;

	vel_b = raw_v;
	pos_b = pos_i;
//	ROS_INFO("yaw_deg: %f",yaw*57.3);
	// float x[2], y[2], z[2];
	// x[0] = pos_b(0);
	// y[0] = pos_b(1);
	// z[0] = pos_b(2);
	// x[1] = vel_b(0);
	// y[1] = vel_b(1);
	// z[1] = vel_b(2);

	// inertial_filter_predict(dt, x, acc_b(0));
	// inertial_filter_predict(dt, y, acc_b(1));
	// inertial_filter_predict(dt, z, acc_b(2));
	// float ex = pos_i(0) - x[0];
	// float ey = pos_i(1) - y[0];
	// float ez = pos_i(2) - z[0];
	// #define WEIGHT_P 0.8
	// #define WEIGHT_V 0.8
	// inertial_filter_correct(ex, dt, x, 0, WEIGHT_P);
	// inertial_filter_correct(ey, dt, y, 0, WEIGHT_P);
	// inertial_filter_correct(ez, dt, z, 0, WEIGHT_P);
	// float evx = raw_v(0) - x[1];
	// float evy = raw_v(1) - y[1];
	// float evz = raw_v(2) - z[1];
	// inertial_filter_correct(evx, dt, x, 1, WEIGHT_V);
	// inertial_filter_correct(evy, dt, y, 1, WEIGHT_V);
	// inertial_filter_correct(evz, dt, z, 1, WEIGHT_V);
	// pos_b(0) = x[0];
	// vel_b(0) = x[1];
	// pos_b(1) = y[0];
	// vel_b(1) = y[1];
	// pos_b(2) = z[0];
	// vel_b(2) = z[1];
	pos_w = R_body * pos_b;

	body_pose.pose.position.x = pos_b(0);
	body_pose.pose.position.y = pos_b(1);
	body_pose.pose.position.z = pos_b(2);
	world_pose.pose.position.x = pos_w(0);
	world_pose.pose.position.y = pos_w(1);
	world_pose.pose.position.z = pos_w(2);

	pose_body_pub.publish(body_pose);
	pose_world_pub.publish(world_pose);

	// 0: Unknown
	// 1: Inited
	// 2: Landed
	// 3,7: Flying
	// 4: Hovering
	// 5: Test (?)
	// 6: Taking off
	// 8: Landing
	// 9: Looping (?)
	drone_state = msg.state;
}

// void States::odometryCallback(const nav_msgs::Odometry &msg)
// {
// 	pos_w(2) = msg.pose.pose.position.z;
// }

void States::altitudeCallback(const ardrone_autonomy::navdata_altitude &msg)
{
	pos_w(2) = msg.altitude_vision/1000.0;
}

void imagepositionCallback(const ardrone_control::ROI &msg)
{
	image_pos_pre(0) = image_pos(0);
	image_pos_pre(1) = image_pos(1);
	image_pos(0) = msg.pose1.x;
	image_pos(1) = msg.pose1.y;
	image_stamp = ros::Time::now();
	first_pos_received = true;

}

void numberCallback(const ardrone_control::ROINumber &msg)
{
	if(msg.image1 == number_last) number = msg.image1;
	number_last =  msg.image1;
	number_stamp = ros::Time::now();
}

void calcCoords(float coord[10][10][2])
{
	Matrix<float, 10, 3> dists;
	Matrix<float, 3, 10> dists_world_temp;
	Matrix<float, 10, 3> dists_world;
	Matrix<float, 3, 3> Rf;
	dists<<
	0.0, 0.0, 0.0,
	1.85,-1.78,0.0,
	0.28,2.78,0.0,
	0.66,-1.61,0.0,
	1.9,-1.56,0.0,
	2.84,2.36,0.0,
	-2.8,0.53,0.0,
	4.87,-0.1,0.0,
	-0.53,-2.71,0.0,
	-3.47,1.05,0.0;
	float world_field_yaw = -85/57.3;
	Rf(0,0) = cos(world_field_yaw);
	Rf(0,1) = sin(-world_field_yaw);
	Rf(0,2) = 0;
	Rf(1,0) = sin(world_field_yaw);
	Rf(1,1) = cos(world_field_yaw);
	Rf(1,2) = 0;
	Rf(2,0) = 0;
	Rf(2,1) = 0;
	Rf(2,2) = 1;
	dists_world_temp = Rf*dists.transpose();
	dists_world = dists_world_temp.transpose();

	for (int i = 0; i < 9; ++i)
	{
		coord[i][i+1][0] = dists_world(i+1, 0);
		coord[i][i+1][1] = dists_world(i+1, 1);
		coord[i][i][0] = 0;
		coord[i][i][1] = 0;
	}

	for (int i = 0; i < 10; ++i)
	{
		for (int j = 0; j < 10; ++j)
		{
			if (j < i)
			{
				coord[i][j][0] = -coord[j][i][0];
				coord[i][j][1] = -coord[j][i][1];
			}
			if (j - i > 1)
			{
				for (int k = i; k < j; ++k)
				{
					coord[i][j][0] += coord[k][k + 1][0];
					coord[i][j][1] += coord[k][k + 1][1];
				}
			}
		}
	}

	for(int i=0;i<10;i++)
	{
		for(int j=0;j<10;j++)
		{
			cout<<coord[i][j][0]<<" ";
			cout<<coord[i][j][1]<<"\t";

		}
		cout<<endl;
	}
	cout<<endl;
}

int main(int argc, char **argv)
{
	int keypress = 0;
	
	ros::init(argc, argv, "position_control2");
	num_flight flight;
	States state;
	ros::NodeHandle n;
	ros::Subscriber image_pos_sub = n.subscribe("/ROI", 1, imagepositionCallback);
	ros::Subscriber number_sub = n.subscribe("/number_result", 1, numberCallback);
	ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	ros::Publisher takeoff_pub = n.advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
	ros::Publisher land_pub = n.advertise<std_msgs::Empty>("/ardrone/land", 1);
	ros::Publisher stop_pub = n.advertise<std_msgs::Empty>("/ardrone/reset", 1);
	
	ros::Rate loop_rate(LOOP_RATE);

	std_msgs::Empty order;
	bool isArrived = false;
	Vector3f vel_sp(0.0, 0.0, 0.0);
	Vector3f next_pos_sp(0.0, 0.0, 0.0);
	geometry_msgs::Twist cmd;

	calcCoords(coord);

	flight._state = STATE_TAKEOFF;
//	next_pos_sp(2) = state.pos_w(2);
	ROS_INFO("started");

	while(ros::ok() && !first_yaw_received){
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	while(ros::ok())
	{
		
		ros::Duration dur;
        	isArrived = false;
		switch(flight._state){
			case STATE_TAKEOFF:
				if(state.drone_state == 2){//landed
					takeoff_pub.publish(order);
					landed_pos=state.pos_w;
				}
				else{//flying, takeoff completed
					if(takeoff_counter > 40)
					{
						height_sp = NORMAL_HEIGHT;
						next_pos_sp(2) = height_sp;
						isArrived = flight.altitude_change(next_pos_sp, state.pos_w, vel_sp);
						vel_sp(0) = 0;
						vel_sp(1) = 0;
						takeoff_counter = 0;
					}else{
						takeoff_counter++;
					}
					
				}
				break;
			case STATE_ACCURATE_AFTER_TAKEOFF:
				isArrived = flight.accurate_control(image_pos, state.pos_w(2), vel_sp);
				dur = ros::Time::now() - image_stamp;
				if(dur.toSec() > 0.5 && counter_after_takeoff<=WAIT_TIME){
					vel_sp(0) = 0;
					vel_sp(1) = 0;
					vel_sp(2) = 0;
					counter_after_takeoff ++;
				}else if(counter_after_takeoff > WAIT_TIME){
					if(state.pos_w(2) < THR_HEIGHT){
						flight._state = STATE_FIND_NUM_AFTER_TAKEOFF;

					}else{
						flight._state = STATE_FIND_NUM_AFTER_TAKEOFF_MOVE;
						move_start_x = state.pos_w(0); //chg
						move_start_y = state.pos_w(1);//chg

					}
					counter_after_takeoff = 0;
				}
				break;
			case STATE_IDLE:
				isArrived = flight.idle_control(vel_sp);
				break;
			case STATE_INACCURATE:
				isArrived = flight.inaccurate_control(next_pos_sp, state.pos_w, vel_sp);
				dur = ros::Time::now() - state.navdata_stamp;
				if(dur.toSec() > 0.5){
					vel_sp(0) = 0;
					vel_sp(1) = 0;
					vel_sp(2) = 0;
				}
				break;
			case STATE_ACCURATE_BEFORE_LANDING:
				isArrived = flight.accurate_control(image_pos, state.pos_w(2), vel_sp);
				dur = ros::Time::now() - image_stamp;
				if(dur.toSec() > 0.5 && counter_before_landing<=WAIT_TIME){
					vel_sp(0) = 0;
					vel_sp(1) = 0;
					vel_sp(2) = 0;
					counter_before_landing ++;
				}else if(counter_before_landing>WAIT_TIME){
					if(state.pos_w(2) < THR_HEIGHT){
						flight._state = STATE_FIND_NUM_BEFORE_LANDING;
					}else{
						flight._state = STATE_FIND_NUM_BEFORE_LANDING_MOVE;
						move_start_x = state.pos_w(0); //chg
						move_start_y = state.pos_w(1);//chg
					}
					counter_before_landing =0;
				}
				break;
			case STATE_LANDING:
				land_pub.publish(order);
				if(state.drone_state == 2){//landed
					isArrived = true;
				}
				break;
			case STATE_ALT:
				isArrived = flight.altitude_change(next_pos_sp, state.pos_w, vel_sp);
				break;
			case STATE_FIND_NUM_AFTER_TAKEOFF:
				height_sp = FIND_HEIGHT;
				next_pos_sp(2) = height_sp;
				isArrived = flight.altitude_change(next_pos_sp, state.pos_w, vel_sp);
				vel_sp(0) = 0;
				vel_sp(1) = 0;
				break;
			case STATE_FIND_NUM_BEFORE_LANDING:
				height_sp = FIND_HEIGHT;
				next_pos_sp(2) = height_sp;
				isArrived = flight.altitude_change(next_pos_sp, state.pos_w, vel_sp);
				vel_sp(0) = 0;
				vel_sp(1) = 0;
				break;
			case STATE_FIND_NUM_AFTER_TAKEOFF_MOVE:
				switch(move_flag1)
				{
					case 0:
					next_pos_sp(0) = move_start_x + SEARCH_SIZE;
					break;
					case 1:
					next_pos_sp(1) = move_start_y + SEARCH_SIZE;
					break;
					case 2:
					next_pos_sp(0) = move_start_x - 2*SEARCH_SIZE;
					break;
					case 3:
					next_pos_sp(1) = move_start_y - 2*SEARCH_SIZE;
					break;
					case 4:
					next_pos_sp(0) = move_start_x + 2*SEARCH_SIZE;
					break;
					case 5:
					next_pos_sp(0) = move_start_x - SEARCH_SIZE;
					next_pos_sp(1) = move_start_y + SEARCH_SIZE;
					break;
				}
				
				isArrived = flight.inaccurate_control(next_pos_sp, state.pos_w, vel_sp);
				if(isArrived) {move_flag1++; cout<<"1 case "<<move_flag1<<" finished!"<<endl;}
				if(move_flag1 > 5) move_flag1 = 0;
				break;

			case STATE_FIND_NUM_BEFORE_LANDING_MOVE:
				switch(move_flag2)
				{
					case 0:
					next_pos_sp(0) = move_start_x + SEARCH_SIZE;
					break;
					case 1:
					next_pos_sp(1) = move_start_y + SEARCH_SIZE;
					break;
					case 2:
					next_pos_sp(0) = move_start_x - 2*SEARCH_SIZE;
					break;
					case 3:
					next_pos_sp(1) = move_start_y - 2*SEARCH_SIZE;
					break;
					case 4:
					next_pos_sp(0) = move_start_x + 2*SEARCH_SIZE;
					break;
					case 5:
					next_pos_sp(0) = move_start_x - SEARCH_SIZE;
					next_pos_sp(1) = move_start_y + SEARCH_SIZE;
					break;
				}
				
				isArrived = flight.inaccurate_control(next_pos_sp, state.pos_w, vel_sp);
				if(isArrived) {move_flag2++;cout<<"2  case "<<move_flag2<<" finished!"<<endl;}
				if(move_flag2 > 5) move_flag2 = 0;
				break;

		}
		if(isArrived){
			switch(flight._state){
				case STATE_TAKEOFF:
					ROS_INFO("takeoff finished\n");
					flight._state = STATE_ACCURATE_AFTER_TAKEOFF;
					break;
				case STATE_ACCURATE_AFTER_TAKEOFF:
					ROS_INFO("accurate arrived after takeoff\n");
					vel_sp(0) = 0;
					vel_sp(1) = 0;
					vel_sp(2) = 0;
					
					if(flight._current_target == 0){
						next_pos_sp(0) = coord[number][flight._current_target+1][0]+state.pos_w(0);//Modify by CJ
						next_pos_sp(1) = coord[number][flight._current_target+1][1]+state.pos_w(1);//Modify by CJ

			    		move_flag1=3;
			    		move_flag2=0;
			    		flight._state = STATE_INACCURATE;

					}
					else{
						if(number < 10)
						{	
							next_pos_sp(0) = coord[number][flight._current_target+1][0]+state.pos_w(0);
							next_pos_sp(1) = coord[number][flight._current_target+1][1]+state.pos_w(1);
							if(flight._current_target+1 == 2||flight._current_target+1 == 6||flight._current_target+1 == 7) 
					    	{
					    		move_flag1=4;
					    		move_flag2=5;
					    	}
						    else 
						    {
						    	move_flag1 = 2;
						    	move_flag2 = 0;
						    }
						    flight._state = STATE_INACCURATE;
						}


					}
					break;
				case STATE_INACCURATE:
					ROS_INFO("Target %d arrived inaccurately\n", flight._current_target+1);
					vel_sp(0)=0;
					vel_sp(1)=0;
					vel_sp(2)=0;
					if(flight._current_target<9){
						flight._state = STATE_ACCURATE_BEFORE_LANDING;
					}
					break;
				case STATE_ACCURATE_BEFORE_LANDING:
					dur = ros::Time::now() - number_stamp;
					if(dur.toSec() > 1.0){
						vel_sp(0) = 0;
						vel_sp(1) = 0;
						vel_sp(2) = 0;
					}else{
						if(number == flight._current_target + 1){
							ROS_INFO("Ready to land\n");
							vel_sp(0) = 0;
							vel_sp(1) = 0;
							vel_sp(2) = 0;
							flight._state = STATE_LANDING;
						}else{
							flight._state=STATE_ACCURATE_AFTER_TAKEOFF; //modified by Chen
							ROS_INFO("Wrong number!\n");
						}
					}
					break;
				case STATE_LANDING:
					ROS_INFO("landed\n");
					flight._current_target++;
					if(flight._current_target > 8){
						return 0;
					}
					else{
						flight._state = STATE_TAKEOFF;

					}
	                vel_sp(0) = 0;
	                vel_sp(1) = 0;
					break;
				case STATE_FIND_NUM_AFTER_TAKEOFF:
					flight._state = STATE_ACCURATE_AFTER_TAKEOFF;
					break;
				case STATE_FIND_NUM_BEFORE_LANDING:
					flight._state = STATE_ACCURATE_BEFORE_LANDING;
					break;
				case STATE_FIND_NUM_AFTER_TAKEOFF_MOVE:
					flight._state = STATE_ACCURATE_AFTER_TAKEOFF;
					break;
				case STATE_FIND_NUM_BEFORE_LANDING_MOVE:
					flight._state = STATE_ACCURATE_BEFORE_LANDING;
					break;	
			}
		}
		Vector3f vel_sp_b;
		if(flight._state == STATE_ACCURATE_BEFORE_LANDING || flight._state == STATE_ACCURATE_AFTER_TAKEOFF){
			vel_sp_b = vel_sp;
		}
		else if(flight._state == STATE_INACCURATE){
			vel_sp_b = state.R_body.transpose() * vel_sp;
		}
		else{
			vel_sp_b = state.R_body.transpose() * vel_sp;
		}

//		ROS_INFO("\nvel_body(%f,%f)",vel_sp_b(0),vel_sp_b(1));
		cmd.linear.x = vel_sp_b(0);
		cmd.linear.y = vel_sp_b(1);
		cmd.linear.z = vel_sp_b(2);
		cmd_pub.publish(cmd);
		ros::spinOnce();
		loop_rate.sleep();
	}	
	return 0;
}
