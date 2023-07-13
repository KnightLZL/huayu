#pragma once
#include <iostream>
#include <vector>
using namespace std;

struct Localization_info  //车辆定位信息
{
	double host_vx;
	double host_vy;
	double host_ax;
	double host_ay;
	double host_heading;
	double host_x;
	double host_y;
	
};



struct ReferenceInfo  //平滑后的参考线信息
{
	double referenceline_X;
	double referenceline_y;
	double referenceline_heading;
	double referencelinr_kappa;
	//int origin_match_point_index;
	//double orgiin_x;
	//double origin_y;
};