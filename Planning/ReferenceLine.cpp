#include <iostream>
#include <vector>
#include <algorithm>
#include <utility>

#include "Input.h"
#include "discrete_points_math.h"
using namespace std;

void ReferenceLine(const std::vector<std::pair<double, double>>&, const ReferenceInfo &);
ReferenceInfo referenceLine;

//测试ReferenceLine函数输入
void test01()  
{
	std::vector<std::pair<double, double>> v1;  
	v1.push_back(std::make_pair(1, 2));
	ReferenceLine(v1, referenceLine);
}

//测试计算离散点heading和kappa的函数DiscretePointsmath::ComputePathProfile
void test02()  
{
	std::vector<std::pair<double, double>> v2;  
	v2.push_back(std::make_pair(1, 1));
	v2.push_back(std::make_pair(2, 0));
	v2.push_back(std::make_pair(3, 1));
	std::vector<double> heading;
	std::vector<double> accumulated;
	std::vector<double> kappa;
	std::vector<double> dkappa;

	bool a;
	a = planning::DiscretePointsMath::ComputePathProfile(v2, &heading, &accumulated, &kappa, &dkappa);
	cout << a << endl;
	cout << heading.at(1) << endl;
	cout << accumulated.at(1) << endl;
	cout << kappa.at(1) << endl;
	cout << dkappa.at(1) << endl;

}

int main()
{
	//test01(); 
	test02();

} 


//参考线平滑函数
void ReferenceLine( const std::vector<std::pair<double, double>>& routine_line,  const ReferenceInfo & referenceInfo)
{
	cout << routine_line.front().first << endl; 
	cout << routine_line.front().second << endl; 
}
