#include "discrete_points_math.h"

#include <cmath>


namespace planning {
	bool DiscretePointsMath::ComputePathProfile(
		const std::vector<std::pair<double, double>>& xy_points,
		std::vector<double>* headings, std::vector<double>* accumulated_s,
		std::vector<double>* kappas, std::vector<double>* dkappas) {
		headings->clear(); //�����Щָ��ָ��Ŀռ�
		kappas->clear();
		dkappas->clear();
		
		if (xy_points.size() < 2)
		{
			return false; //ֻ��һ���㣬���ܼ�����Ϣ
		}
		std::vector<double> dxs;
		std::vector<double> dys;
		std::vector<double> y_over_s_first_derivatives;
		std::vector<double> x_over_s_first_derivatives;
		std::vector<double> y_over_s_second_derivatives;
		std::vector<double> x_over_s_second_derivatives;

		//Ϊ�˷������heading��kappa��ͨ������΢��������dx��dy
		//���ȼ���dx��dy
		int points_size = xy_points.size();  //size_t����ߴ����ƽ̨������,һ�������int
		for (int i = 0; i < points_size; ++i)
		{
			double x_delta = 0.0;
			double y_delta = 0.0;
			if (i == 0)  //��һ���㣬���ŷ����
			{
				x_delta = (xy_points[i + 1].first - xy_points[i].first);
				y_delta = (xy_points[i + 1].second - xy_points[i].second);
			}
			else if (i == points_size - 1) //���һ���㡣���ŷ����
			{
				x_delta = (xy_points[i].first - xy_points[i - 1].first);
				y_delta = (xy_points[i].second - xy_points[i - 1].second);
			}
			else //�м�ĵ㣬�е�ŷ����
			{
				x_delta = 0.5*(xy_points[i + 1].first - xy_points[i - 1].first);
				y_delta = 0.5*(xy_points[i + 1].second - xy_points[i - 1].second);
			}
			dxs.push_back(x_delta);
			dys.push_back(y_delta);
		}

		//heading����
		for (int i = 0; i < points_size; i++)
		{
			headings->push_back(std::atan2(dys[i], dxs[i]));  //atan2���������У����Ը�������õ����ޣ������ΧΪ[-pi,pi],��atan�����У��������[-pi/2,pi/2]
		}

		//Ϊ�˼���kappa����s�������Բ�ֵ,�õ�accumulated_s
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

		//Ϊ�˼���kappa������x��y��һ�ײ��,���һ�ײ��
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
					(accumulated_s->at(i) - accumulated_s->at(i-1));  //xy_pointsͨ�����ô��룬ֱ��ʹ��[]�������л�ȡ��accumulated_sͨ��ָ�봫�룬��Ҫͨ��at(i)������ȡ����Ϊi��ֵ��
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

		//Ϊ�˼���kappa����Ҫ���x��y�Ķ���΢��
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


		//����kappa
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
		
		//����Dkappa
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
