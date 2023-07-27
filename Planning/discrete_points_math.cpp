#include "discrete_points_math.h"

#include <cmath>


namespace planning {
	bool DiscretePointsMath::ComputePathProfile(
		const std::vector<std::pair<double, double>>& xy_points,
		std::vector<double>* headings, std::vector<double>* accumulated_s,
		std::vector<double>* kappas, std::vector<double>* dkappas) {
		headings->clear(); //清除这些指针指向的空间
		kappas->clear();
		dkappas->clear();
		
		if (xy_points.size() < 2)
		{
			return false; //只有一个点，不能计算信息
		}
		std::vector<double> dxs;
		std::vector<double> dys;
		std::vector<double> y_over_s_first_derivatives;
		std::vector<double> x_over_s_first_derivatives;
		std::vector<double> y_over_s_second_derivatives;
		std::vector<double> x_over_s_second_derivatives;

		//为了方便计算heading和kappa，通过有限微信来计算dx和dy
		//首先计算dx和dy
		int points_size = xy_points.size();  //size_t会提高代码的平台适用性,一般可以用int
		for (int i = 0; i < points_size; ++i)
		{
			double x_delta = 0.0;
			double y_delta = 0.0;
			if (i == 0)  //第一个点，向后欧拉法
			{
				x_delta = (xy_points[i + 1].first - xy_points[i].first);
				y_delta = (xy_points[i + 1].second - xy_points[i].second);
			}
			else if (i == points_size - 1) //最后一个点。向后欧拉法
			{
				x_delta = (xy_points[i].first - xy_points[i - 1].first);
				y_delta = (xy_points[i].second - xy_points[i - 1].second);
			}
			else //中间的点，中点欧拉法
			{
				x_delta = 0.5*(xy_points[i + 1].first - xy_points[i - 1].first);
				y_delta = 0.5*(xy_points[i + 1].second - xy_points[i - 1].second);
			}
			dxs.push_back(x_delta);
			dys.push_back(y_delta);
		}

		//heading计算
		for (int i = 0; i < points_size; i++)
		{
			headings->push_back(std::atan2(dys[i], dxs[i]));  //atan2对象限敏感，可以根据输入得到象限，输出范围为[-pi,pi],而atan不敏感，输出的是[-pi/2,pi/2]
		}

		//为了计算kappa，对s进行线性插值,得到accumulated_s
		double distance = 0.0;
		accumulated_s->push_back(distance);
		double fx = xy_points[0].first;
		double fy = xy_points[0].second;
		double nx = 0.0;
		double ny = 0.0;
		for (int i = 1; i < points_size; ++i)
		{
			nx = xy_points[i].first;
			ny = xy_points[i].second;
			double end_segment_s =
				std::sqrt((fx - nx)*(fx - nx) + (fy - ny)*(fy - ny));
			accumulated_s->push_back(end_segment_s + distance);
			distance += end_segment_s;
			fx = nx;
			fy = ny;
		}

		//为了计算kappa，进行x和y的一阶差分,获得一阶差分
		for (int i = 0; i < points_size; ++i)
		{
			double xds = 0.0;
			double yds = 0.0;
			if (i == 0)
			{
				xds = (xy_points[i + 1].first - xy_points[i].first) /
					(accumulated_s->at(i+1) - accumulated_s->at(i));
				yds = (xy_points[i + 1].second - xy_points[i].second)/
					(accumulated_s->at(i+1) - accumulated_s->at(i));
			}
			else if (i == points_size - 1)
			{
				xds = (xy_points[i].first - xy_points[i - 1].first) /
					(accumulated_s->at(i) - accumulated_s->at(i-1));  //xy_points通过引用传入，直接使用[]索引进行获取；accumulated_s通过指针传入，需要通过at(i)函数获取索引为i的值；
				yds = (xy_points[i].second - xy_points[i - 1].second) /
					(accumulated_s->at(i) - accumulated_s->at(i-1));
			}
			else
			{
				xds = (xy_points[i + 1].first - xy_points[i - 1].first) /
					(accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
				yds = (xy_points[i + 1].second - xy_points[i - 1].second) /
					(accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
			}
			x_over_s_first_derivatives.push_back(xds);
			y_over_s_first_derivatives.push_back(yds);
		}

		//为了计算kappa，需要获得x和y的二阶微分
		for (int i = 0; i < points_size; ++i)
		{
			double xdds = 0.0;
			double ydds = 0.0;
			if (i == 0)
			{
				xdds = (x_over_s_first_derivatives[i + 1] - x_over_s_first_derivatives[i]) /
					(accumulated_s->at(i + 1) - accumulated_s->at(i));
				ydds = (y_over_s_first_derivatives[i + 1] - y_over_s_first_derivatives[i]) /
					(accumulated_s->at(i + 1) - accumulated_s->at(i));
			}
			else if (i == points_size - 1)
			{
				xdds = (x_over_s_first_derivatives[i] - x_over_s_first_derivatives[i - 1]) /
					(accumulated_s->at(i) - accumulated_s->at(i - 1));
				ydds = (y_over_s_first_derivatives[i] - y_over_s_first_derivatives[i - 1]) /
					(accumulated_s->at(i) - accumulated_s->at(i - 1));
			}
			else
			{
				xdds = (x_over_s_first_derivatives[i + 1] - x_over_s_first_derivatives[i - 1]) /
					(accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
				ydds = (y_over_s_first_derivatives[i + 1] - y_over_s_first_derivatives[i - 1]) /
					(accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
			}
			x_over_s_second_derivatives.push_back(xdds);
			y_over_s_second_derivatives.push_back(ydds);
		}


		//计算kappa
		for (int i = 0; i < points_size; ++i)
		{
			double xds = x_over_s_first_derivatives[i];
			double yds = y_over_s_first_derivatives[i];
			double xdds = x_over_s_second_derivatives[i];
			double ydds = y_over_s_second_derivatives[i];
			double kappa =
				(xds * ydds - yds * xdds) /
				(std::sqrt(xds * xds + yds * yds) * (xds * xds + yds * yds) + 1e-6);  
			kappas->push_back(kappa);
		}
		
		//计算Dkappa
		for (int i = 0; i < points_size; ++i)
		{
			double dkappa = 0.0;
			if (i == 0) {
				dkappa = (kappas->at(i + 1) - kappas->at(i)) /
					(accumulated_s->at(i + 1) - accumulated_s->at(i));
			}
			else if (i == points_size - 1) {
				dkappa = (kappas->at(i) - kappas->at(i - 1)) /
					(accumulated_s->at(i) - accumulated_s->at(i - 1));
			}
			else {
				dkappa = (kappas->at(i + 1) - kappas->at(i - 1)) /
					(accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
			}
			dkappas>push_back(dkappa);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                
		}
		return 1 ;
	}

} //namespace planning
