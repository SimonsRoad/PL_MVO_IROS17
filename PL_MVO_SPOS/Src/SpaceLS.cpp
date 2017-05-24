
#include "SpaceLS.h"


SpaceLS::SpaceLS()
{
}


SpaceLS::~SpaceLS()
{
}


SpaceLS::SpaceLS(const cv::Point3d& S_c_W, const cv::Point3d& E_c_W)
{
	this->S_c_W = S_c_W;
	this->E_c_W = E_c_W;

	// Automatically get direction vector
	cv::Mat dv_c_W;
	this->getDirVecinW(S_c_W, E_c_W,
		dv_c_W);

	this->dv_c_W = dv_c_W.clone();

	// Automatically get norm vector
	cv::Mat n_c_W;
	this->getNormVecinW(S_c_W, E_c_W,
		n_c_W);

	this->n_c_W = n_c_W.clone();
}


void SpaceLS::getDirVecinW(const cv::Point3d& S_c_W, const cv::Point3d& E_c_W,
	cv::Mat& dv_c_W)
{
	dv_c_W = cv::Mat::zeros(3, 1, CV_64FC1);
	dv_c_W.at<double>(0) = E_c_W.x - S_c_W.x;
	dv_c_W.at<double>(1) = E_c_W.y - S_c_W.y;
	dv_c_W.at<double>(2) = E_c_W.z - S_c_W.z;

	//dv_c_W = dv_c_W / cv::norm(dv_c_W);
}


void SpaceLS::getNormVecinW(const cv::Point3d& S_c_W, const cv::Point3d& E_c_W,
	cv::Mat& n_c_W)
{
	cv::Mat S_c_WMat = cv::Mat::zeros(3, 1, CV_64FC1);
	S_c_WMat.at<double>(0) = S_c_W.x;
	S_c_WMat.at<double>(1) = S_c_W.y;
	S_c_WMat.at<double>(2) = S_c_W.z;

	cv::Mat E_c_WMat = cv::Mat::zeros(3, 1, CV_64FC1);
	E_c_WMat.at<double>(0) = E_c_W.x;
	E_c_WMat.at<double>(1) = E_c_W.y;
	E_c_WMat.at<double>(2) = E_c_W.z;

	n_c_W = S_c_WMat.cross(E_c_WMat);

	//n_c_W = n_c_W / cv::norm(n_c_W);
}
