#pragma once

#include <opencv2\opencv.hpp>

class ImgLS
{
public:
	ImgLS();
	~ImgLS();

	ImgLS(const cv::Point2d& s_c_o, const cv::Point2d& e_c_o, const cv::Mat& K);

	// get normalized image plane coordiante
	cv::Point2d getNormCoord(const cv::Point2d& c_o, const cv::Mat& K);

	// ordinary image plane
	cv::Point2d s_c_o; 
	cv::Point2d e_c_o;

	// normalized image plane
	cv::Point2d s_c_n; 
	cv::Point2d e_c_n;
};

