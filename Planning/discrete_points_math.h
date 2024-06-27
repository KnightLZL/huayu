/**
*用来计算导航点中的航向角和曲率信息
*可以用以后续相同模块的通用计算
**/
#pragma once

#include <utility>
#include <vector>


namespace planning {

	class DiscretePointsMath {
	public:
		DiscretePointsMath() = delete;

		static bool ComputePathProfile(
			const std::vector<std::pair<double, double>>& xy_points,
			std::vector<double>* headings, std::vector<double>* accumulated_s,
			std::vector<double>* kappas, std::vector<double>* dkappas);

};
} //namespace planning
