#pragma once

#include <fstream>
#include <sstream>
#include <vector>

#include <opencv2\opencv.hpp>

#include "SpaceImgCorrLSs.h"

class FileDisposal
{
public:
	FileDisposal();
	~FileDisposal();

	void readKMat(std::string KMatPath, 
		cv::Mat& K);

	void constrSpaceImgCorrPts(std::string spaceLSsPath, const cv::Mat& K,
		std::vector<SpaceImgCorrLSs>& spaceImgCorrLSs, cv::Mat& trueR, cv::Mat& truet);

	cv::Point2d projSpacePt(const cv::Point3d& c_W, const cv::Mat& K, const cv::Mat& R, const cv::Mat& t);
};

