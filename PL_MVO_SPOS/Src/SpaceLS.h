#pragma once

#include <opencv2\opencv.hpp>

class SpaceLS
{
public:
	SpaceLS();
	~SpaceLS();

	SpaceLS(const cv::Point3d& S_c_W, const cv::Point3d& E_c_W);

	void getDirVecinW(const cv::Point3d& S_c_W, const cv::Point3d& E_c_W,
		cv::Mat& dv_c_W);

	void getNormVecinW(const cv::Point3d& S_c_W, const cv::Point3d& E_c_W,
		cv::Mat& n_c_W);

	cv::Point3d S_c_W;
	cv::Point3d E_c_W;

	// v_W
	cv::Mat dv_c_W;

	// n_W
	cv::Mat n_c_W;
};

