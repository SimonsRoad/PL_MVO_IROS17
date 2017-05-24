
#include "ImgLS.h"


ImgLS::ImgLS()
{
}


ImgLS::~ImgLS()
{
}


ImgLS::ImgLS(const cv::Point2d& s_c_o, const cv::Point2d& e_c_o, const cv::Mat& K)
{
	this->s_c_o = s_c_o;
	this->e_c_o = e_c_o;

	// Automatically get normalized coordinate
	this->s_c_n = this->getNormCoord(s_c_o, K);
	this->e_c_n = this->getNormCoord(e_c_o, K);
}

// get normalized image plane coordiante
cv::Point2d ImgLS::getNormCoord(const cv::Point2d& c_o, const cv::Mat& K)
{
	/*
	xn = (u - cu)/f
	yn = (v - cv)/f
	*/
	double f = (K.at<double>(0, 0) + K.at<double>(1, 1)) / 2;
	double cx = K.at<double>(0, 2);
	double cy = K.at<double>(1, 2);

	double cu_n = (c_o.x - cx) / f;
	double cv_n = (c_o.y - cy) / f;

	return cv::Point2d(cu_n, cv_n);
}
