// Fight.cpp : 定义控制台应用程序的入口点。
//



#include "FileDisposal.h"
#include "SpaceImgCorrLSs.h"
#include "MinSampsonError.h"

#include <opencv2\opencv.hpp>

int main()
{
	FileDisposal fd;

	cv::Mat K;
	std::string KMatPath = "..\\Data\\K.txt";
	fd.readKMat(KMatPath,
		K);

	std::string spacePtsPath = "..\\Data\\spacePts.txt";
	std::vector<SpaceImgCorrLSs> spaceImgCorrLSs;

	cv::Mat trueR;
	cv::Mat truet;
	fd.constrSpaceImgCorrPts(spacePtsPath, K, 
		spaceImgCorrLSs, trueR, truet);

	std::cout << "True R" << std::endl;
	std::cout << trueR << std::endl;

	std::cout << "True t" << std::endl;
	std::cout << truet << std::endl;


	MinSampsonError mse;

	cv::Mat theta = cv::Mat::zeros(dimTheta, 1, CV_64FC1); // initialize as zero-vec - reserve as initial interface

	if (mse.minSampsonErrorbyFMS(spaceImgCorrLSs, K,
		theta) == true)
	{
		std::cout << "=== Convergence ===" << std::endl;
		cv::Mat optR;
		cv::Mat optt;
		mse.getOptimalR(theta,
			optR, optt);

		std::cout << "Optimal R" << std::endl;
		std::cout << optR << std::endl;

		std::cout << "Optimal t" << std::endl;
		std::cout << optt << std::endl;
	}
	else
	{
		std::cout << "=== Not Convergence ===" << std::endl;
	}

	return 0;
}

