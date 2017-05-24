
#include "FileDisposal.h"
#include "SpaceImgCorrLSs.h"


FileDisposal::FileDisposal()
{
}


FileDisposal::~FileDisposal()
{
}


void FileDisposal::readKMat(std::string KPath, cv::Mat& K)
{
	std::ifstream ifs(KPath, std::ios::in);
	double paramArray[9];

	int num = 0;
	double param;
	while (ifs >> param)
	{
		paramArray[num] = param;
		num++;
	}
	ifs.close();

	K = cv::Mat(3, 3, CV_64FC1, paramArray).clone();  
}


// We provide some data to test, readers can also use their own data
void FileDisposal::constrSpaceImgCorrPts(std::string spaceLSsPath, const cv::Mat& K,
	std::vector<SpaceImgCorrLSs>& spaceImgCorrLSs, cv::Mat& trueR, cv::Mat& truet)
{
	// Test data1
	// 0.90 // [0.0090, -0.2259, 0.0262]
	//double RArr[9] = { 0.901, -0.054, -0.429,
	//	0.046, 0.998, -0.028,
	//	0.430, 0.005, 0.902 };
	//cv::Mat R(3, 3, CV_64FC1, RArr);

	//double tArr[3] = { 2.867,
	//	0.102,
	//	0.171 };
	//cv::Mat t(3, 1, CV_64FC1, tArr);

	// Test data2
	//0.36 // [0.0431, -0.6810, 0.0454]
	double RArr[9] = { 0.365, -0.102, -0.925,
		0.0218, 0.994, -0.101,
		0.930, 0.016, 0.365 };
	cv::Mat R(3, 3, CV_64FC1, RArr);

	double tArr[3] = { 6.142,
		0.211,
		3.231 };
	cv::Mat t(3, 1, CV_64FC1, tArr);


	trueR = R.clone();
	truet = t.clone();

	std::ifstream ifs(spaceLSsPath, std::ios::in);
	std::string oneRowStr;  

	while (std::getline(ifs, oneRowStr))  
	{
		std::stringstream ss(oneRowStr);  
		std::string oneChildStr; 

		std::vector<std::string> childStrsinOneRow;  
		while (std::getline(ss, oneChildStr, ','))
		{
			childStrsinOneRow.push_back(oneChildStr);
		}

		cv::Point3d S_c_W(atof(childStrsinOneRow[0].c_str()), atof(childStrsinOneRow[1].c_str()), atof(childStrsinOneRow[2].c_str()));
		cv::Point3d E_c_W(atof(childStrsinOneRow[3].c_str()), atof(childStrsinOneRow[4].c_str()), atof(childStrsinOneRow[5].c_str()));
		SpaceLS spaceLS(S_c_W, E_c_W);

		cv::Point2d s_c_o = this->projSpacePt(S_c_W, K, trueR, truet); // atof return double
		cv::Point2d e_c_o = this->projSpacePt(E_c_W, K, trueR, truet);
		ImgLS imgLS(s_c_o, e_c_o, K);
		spaceImgCorrLSs.push_back(SpaceImgCorrLSs(spaceLS, imgLS));
	}
}


cv::Point2d FileDisposal::projSpacePt(const cv::Point3d& c_W, const cv::Mat& K, const cv::Mat& R, const cv::Mat& t)
{


	cv::Mat P = cv::Mat::ones(3, 4, CV_64FC1);
	R.copyTo(P(cv::Rect(0, 0, 3, 3)));
	t.copyTo(P(cv::Rect(3, 0, 1, 3)));
	P = K*P;

	cv::Mat cH_W = cv::Mat::ones(4, 1, CV_64FC1);
	cH_W.at<double>(0) = c_W.x;
	cH_W.at<double>(1) = c_W.y;
	cH_W.at<double>(2) = c_W.z;

	cv::Mat ch_o = P * cH_W;

	float f = ch_o.at<double>(2);


	cv::Point2d c_o(ch_o.at<double>(0) / ch_o.at<double>(2), ch_o.at<double>(1) / ch_o.at<double>(2));

	return c_o;
}