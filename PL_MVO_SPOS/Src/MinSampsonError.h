#pragma once

#include "SpaceImgCorrLSs.h"

#include <vector>

#include <opencv2\opencv.hpp>
#include <Eigen\Dense>

const int dimTheta = 9; // lack of r1 from first row; r5 from second row; r9 from third row, t is enough

class MinSampsonError
{
public:
	MinSampsonError();
	~MinSampsonError();

	// Need to adjust for specific problem

	// Minimize Sampson Error by FNS(main thread function)
	bool minSampsonErrorbyFMS(const std::vector<SpaceImgCorrLSs>& spaceImgCorrLSs, const cv::Mat& K,
		cv::Mat& theta);

	// 1. Construct ksis
	void constrKsi(const cv::Mat& dv_c_W, const cv::Mat& n_c_W, const cv::Point2d& s_c_n, const cv::Point2d& e_c_n, const cv::Mat& K,
		cv::Mat& ksi);

	// 2. Construct Covariance Matrix
	void constrCovMat(const cv::Mat& dv_c_W, const cv::Mat& n_c_W, const cv::Point2d& s_c_n, const cv::Point2d& e_c_n, const cv::Mat& K,
		cv::Mat& Cov);

	// 3. Convert theta to optimalR
	void getOptimalR(const cv::Mat& theta,
		cv::Mat& optR, cv::Mat& optt);


	// General Function(Applied for every problem)

	// Iteration(FNS), return num of iteration
	int iterationbyFNS(const std::vector<cv::Mat>& ksis, const std::vector<cv::Mat>& Covs,
		cv::Mat& theta);

	// Construct M Mat & L Mat, the last param is unknown vector to solve by iteration
	void constrMandL(const std::vector<double>& weights, const std::vector<cv::Mat>& ksis, const std::vector<cv::Mat>& Covs, const cv::Mat& curTheta,
		cv::Mat& M, cv::Mat& L);

	// Solve the eigen problem(use Eigen Library)
	void solveEig(const cv::Mat& M, const cv::Mat& L,
		cv::Mat& newTheta);
};

