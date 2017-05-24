
#include "MinSampsonError.h"


MinSampsonError::MinSampsonError()
{
}


MinSampsonError::~MinSampsonError()
{
}

// Need to adjust for specific problem

// Minimize Sampson Error by FNS(main thread function)
bool MinSampsonError::minSampsonErrorbyFMS(const std::vector<SpaceImgCorrLSs>& spaceImgCorrLSs, const cv::Mat& K,
	cv::Mat& theta)
{
	std::vector<cv::Mat> Covs;
	std::vector<cv::Mat> ksis;

	for (int i = 0; i < spaceImgCorrLSs.size(); i++)
	{
		// constraint's constant variable
		cv::Mat v_c_W = spaceImgCorrLSs[i].spaceLS.dv_c_W;
		cv::Mat n_c_W = spaceImgCorrLSs[i].spaceLS.n_c_W;

		cv::Point2d s_c_o = spaceImgCorrLSs[i].imgLS.s_c_o;
		cv::Point2d e_c_o = spaceImgCorrLSs[i].imgLS.e_c_o;
		
		// 1. Construct ksis
		cv::Mat ksi;
		this->constrKsi(v_c_W, n_c_W, s_c_o, e_c_o, K,
			ksi);

		ksis.push_back(ksi);

		// 2. Construct Cov Mat
		cv::Mat Cov;
		this->constrCovMat(v_c_W, n_c_W, s_c_o, e_c_o, K,
			Cov);

		Covs.push_back(Cov);
	}

	// iteration by FNS
	int iterNum = this->iterationbyFNS(ksis, Covs,
		theta);


	if (iterNum == 30)
	{
		return false;
	}
	else if (iterNum < 30)
	{
		return true;
	}
	return true;
}


// 1. Construct ksis
void MinSampsonError::constrKsi(const cv::Mat& v_c_W, const cv::Mat& n_c_W, const cv::Point2d& s_c_o, const cv::Point2d& e_c_o, const cv::Mat& K,
	cv::Mat& ksi)
{
	// space direction vector
	double v_cX_W = v_c_W.at<double>(0);
	double v_cY_W = v_c_W.at<double>(1);
	double v_cZ_W = v_c_W.at<double>(2);

	// n_c_W
	double n_cX_W = n_c_W.at<double>(0);
	double n_cY_W = n_c_W.at<double>(1);
	double n_cZ_W = n_c_W.at<double>(2);

	// img extreme pt
	double s_cu_o = s_c_o.x;
	double s_cv_o = s_c_o.y;

	double e_cu_o = e_c_o.x;
	double e_cv_o = e_c_o.y;

	// K
	double fx = K.at<double>(0, 0);
	double fy = K.at<double>(1, 1);
	double cx = K.at<double>(0, 2);
	double cy = K.at<double>(1, 2);

	ksi = cv::Mat::zeros(dimTheta, 1, CV_64FC1);

	ksi.at<double>(0) = e_cv_o*fx*n_cZ_W - fx*n_cZ_W*s_cv_o;
	ksi.at<double>(1) = fx*n_cY_W*s_cv_o - e_cv_o*fx*n_cY_W;
	ksi.at<double>(2) = fy*n_cZ_W*s_cu_o - e_cu_o*fy*n_cZ_W;
	ksi.at<double>(3) = e_cu_o*fy*n_cY_W - fy*n_cY_W*s_cu_o;
	ksi.at<double>(4) = cx*e_cv_o*n_cZ_W - cy*e_cu_o*n_cZ_W - cx*n_cZ_W*s_cv_o + cy*n_cZ_W*s_cu_o + e_cu_o*n_cZ_W*s_cv_o - e_cv_o*n_cZ_W*s_cu_o;
	ksi.at<double>(5) = cy*e_cu_o*n_cY_W - cx*e_cv_o*n_cY_W + cx*n_cY_W*s_cv_o - cy*n_cY_W*s_cu_o - e_cu_o*n_cY_W*s_cv_o + e_cv_o*n_cY_W*s_cu_o;
	ksi.at<double>(6) = fx*s_cv_o*v_cX_W - e_cv_o*fx*v_cX_W;
	ksi.at<double>(7) = e_cu_o*fy*v_cX_W - fy*s_cu_o*v_cX_W;
	ksi.at<double>(8) = cy*e_cu_o*v_cX_W - cx*e_cv_o*v_cX_W + cx*s_cv_o*v_cX_W - cy*s_cu_o*v_cX_W - e_cu_o*s_cv_o*v_cX_W + e_cv_o*s_cu_o*v_cX_W;

	//double RtArr[12] = { 0.901, -0.054, -0.429, 0.046, 0.998, -0.028, 0.430, 0.005, 0.902, 2.867, 0.102, 0.171 };
	//cv::Mat Rt(12, 1, CV_64FC1, RtArr);
	//Rt = Rt / cv::norm(Rt);
	//cv::Mat theta = Rt.clone();

	//std::cout << ksi << std::endl;
	//std::cout << theta.dot(ksi) << std::endl;
}


// 2. Construct Covariance Matrix
void MinSampsonError::constrCovMat(const cv::Mat& dv_c_W, const cv::Mat& n_c_W, const cv::Point2d& s_c_o, const cv::Point2d& e_c_o, const cv::Mat& K,
	cv::Mat& Cov)
{
	// K
	double fx = K.at<double>(0, 0);
	double fy = K.at<double>(1, 1);
	double cx = K.at<double>(0, 2);
	double cy = K.at<double>(1, 2);

	// space direction vector
	double v_cX_W = dv_c_W.at<double>(0);
	double v_cY_W = dv_c_W.at<double>(1);
	double v_cZ_W = dv_c_W.at<double>(2);

	// n_c_W
	double n_cX_W = n_c_W.at<double>(0);
	double n_cY_W = n_c_W.at<double>(1);
	double n_cZ_W = n_c_W.at<double>(2);

	// img extreme pt
	double s_cu_o = s_c_o.x;
	double s_cv_o = s_c_o.y;

	double e_cu_o = e_c_o.x;
	double e_cv_o = e_c_o.y;

	Cov = cv::Mat::eye(dimTheta, dimTheta, CV_64FC1);

	
	Cov.at<double>(0, 0) = 2 * pow(fx, 2) * pow(n_cZ_W, 2);
	Cov.at<double>(0, 1) = Cov.at<double>(1, 0) = -2 * pow(fx, 2) * n_cY_W*n_cZ_W;
	Cov.at<double>(0, 2) = Cov.at<double>(2, 0) = 0;
	Cov.at<double>(0, 3) = Cov.at<double>(3, 0) = 0;
	Cov.at<double>(0, 4) = Cov.at<double>(4, 0) = fx*n_cZ_W*(cx*n_cZ_W - n_cZ_W*s_cu_o) + fx*n_cZ_W*(cx*n_cZ_W - e_cu_o*n_cZ_W);
	Cov.at<double>(0, 5) = Cov.at<double>(5, 0) = -fx*n_cZ_W*(cx*n_cY_W - n_cY_W*s_cu_o) - fx*n_cZ_W*(cx*n_cY_W - e_cu_o*n_cY_W);
	Cov.at<double>(0, 6) = Cov.at<double>(6, 0) = -2 * pow(fx, 2) * n_cZ_W*v_cX_W;
	Cov.at<double>(0, 7) = Cov.at<double>(7, 0) = 0;
	Cov.at<double>(0, 8) = Cov.at<double>(8, 0) = -fx*n_cZ_W*(cx*v_cX_W - e_cu_o*v_cX_W) - fx*n_cZ_W*(cx*v_cX_W - s_cu_o*v_cX_W);

	Cov.at<double>(1, 1) = 2 * pow(fx, 2) * pow(n_cY_W, 2);
	Cov.at<double>(1, 2) = Cov.at<double>(2, 1) = 0;
	Cov.at<double>(1, 3) = Cov.at<double>(3, 1) = 0;
	Cov.at<double>(1, 4) = Cov.at<double>(4, 1) = -fx*n_cY_W*(cx*n_cZ_W - n_cZ_W*s_cu_o) - fx*n_cY_W*(cx*n_cZ_W - e_cu_o*n_cZ_W);
	Cov.at<double>(1, 5) = Cov.at<double>(5, 1) = fx*n_cY_W*(cx*n_cY_W - n_cY_W*s_cu_o) + fx*n_cY_W*(cx*n_cY_W - e_cu_o*n_cY_W);
	Cov.at<double>(1, 6) = Cov.at<double>(6, 1) = 2 * pow(fx, 2) * n_cY_W*v_cX_W;
	Cov.at<double>(1, 7) = Cov.at<double>(7, 1) = 0;
	Cov.at<double>(1, 8) = Cov.at<double>(8, 1) = fx*n_cY_W*(cx*v_cX_W - e_cu_o*v_cX_W) + fx*n_cY_W*(cx*v_cX_W - s_cu_o*v_cX_W);

	Cov.at<double>(2, 2) = 2 * pow(fy, 2) * pow(n_cZ_W, 2);
	Cov.at<double>(2, 3) = Cov.at<double>(3, 2) = -2 * pow(fy, 2) * n_cY_W*n_cZ_W;
	Cov.at<double>(2, 4) = Cov.at<double>(4, 2) = fy*n_cZ_W*(cy*n_cZ_W - n_cZ_W*s_cv_o) + fy*n_cZ_W*(cy*n_cZ_W - e_cv_o*n_cZ_W);
	Cov.at<double>(2, 5) = Cov.at<double>(5, 2) = -fy*n_cZ_W*(cy*n_cY_W - n_cY_W*s_cv_o) - fy*n_cZ_W*(cy*n_cY_W - e_cv_o*n_cY_W);
	Cov.at<double>(2, 6) = Cov.at<double>(6, 2) = 0;
	Cov.at<double>(2, 7) = Cov.at<double>(7, 2) = -2 * pow(fy, 2) * n_cZ_W*v_cX_W;
	Cov.at<double>(2, 8) = Cov.at<double>(8, 2) = -fy*n_cZ_W*(cy*v_cX_W - e_cv_o*v_cX_W) - fy*n_cZ_W*(cy*v_cX_W - s_cv_o*v_cX_W);

	Cov.at<double>(3, 3) = 2 * pow(fy, 2) * pow(n_cY_W, 2);
	Cov.at<double>(3, 4) = Cov.at<double>(4, 3) = -fy*n_cY_W*(cy*n_cZ_W - n_cZ_W*s_cv_o) - fy*n_cY_W*(cy*n_cZ_W - e_cv_o*n_cZ_W);
	Cov.at<double>(3, 5) = Cov.at<double>(5, 3) = fy*n_cY_W*(cy*n_cY_W - n_cY_W*s_cv_o) + fy*n_cY_W*(cy*n_cY_W - e_cv_o*n_cY_W);
	Cov.at<double>(3, 6) = Cov.at<double>(6, 3) = 0;
	Cov.at<double>(3, 7) = Cov.at<double>(7, 3) = 2 * pow(fy, 2) * n_cY_W*v_cX_W;
	Cov.at<double>(3, 8) = Cov.at<double>(8, 3) = fy*n_cY_W*(cy*v_cX_W - e_cv_o*v_cX_W) + fy*n_cY_W*(cy*v_cX_W - s_cv_o*v_cX_W);

	Cov.at<double>(4, 4) = pow((cx*n_cZ_W - e_cu_o*n_cZ_W), 2) + pow((cy*n_cZ_W - e_cv_o*n_cZ_W), 2) + pow((cx*n_cZ_W - n_cZ_W*s_cu_o), 2) + pow((cy*n_cZ_W - n_cZ_W*s_cv_o), 2);
	Cov.at<double>(4, 5) = Cov.at<double>(5, 4) = -(cx*n_cY_W - n_cY_W*s_cu_o)*(cx*n_cZ_W - n_cZ_W*s_cu_o) - (cy*n_cY_W - n_cY_W*s_cv_o)*(cy*n_cZ_W - n_cZ_W*s_cv_o) - (cx*n_cY_W - e_cu_o*n_cY_W)*(cx*n_cZ_W - e_cu_o*n_cZ_W) - (cy*n_cY_W - e_cv_o*n_cY_W)*(cy*n_cZ_W - e_cv_o*n_cZ_W);
	Cov.at<double>(4, 6) = Cov.at<double>(6, 4) = -fx*v_cX_W*(cx*n_cZ_W - e_cu_o*n_cZ_W) - fx*v_cX_W*(cx*n_cZ_W - n_cZ_W*s_cu_o);
	Cov.at<double>(4, 7) = Cov.at<double>(7, 4) = -fy*v_cX_W*(cy*n_cZ_W - e_cv_o*n_cZ_W) - fy*v_cX_W*(cy*n_cZ_W - n_cZ_W*s_cv_o);
	Cov.at<double>(4, 8) = Cov.at<double>(8, 4) = -(cx*n_cZ_W - n_cZ_W*s_cu_o)*(cx*v_cX_W - s_cu_o*v_cX_W) - (cy*n_cZ_W - n_cZ_W*s_cv_o)*(cy*v_cX_W - s_cv_o*v_cX_W) - (cx*n_cZ_W - e_cu_o*n_cZ_W)*(cx*v_cX_W - e_cu_o*v_cX_W) - (cy*n_cZ_W - e_cv_o*n_cZ_W)*(cy*v_cX_W - e_cv_o*v_cX_W);
	
	Cov.at<double>(5, 5) = pow((cx*n_cY_W - e_cu_o*n_cY_W), 2) + pow((cy*n_cY_W - e_cv_o*n_cY_W), 2) + pow((cx*n_cY_W - n_cY_W*s_cu_o), 2) + pow((cy*n_cY_W - n_cY_W*s_cv_o), 2);
	Cov.at<double>(5, 6) = Cov.at<double>(6, 5) = Cov.at<double>(7, 6) = Cov.at<double>(6, 5) = fx*v_cX_W*(cx*n_cY_W - e_cu_o*n_cY_W) + fx*v_cX_W*(cx*n_cY_W - n_cY_W*s_cu_o);
	Cov.at<double>(5, 7) = Cov.at<double>(7, 5) = Cov.at<double>(7, 5) = fy*v_cX_W*(cy*n_cY_W - e_cv_o*n_cY_W) + fy*v_cX_W*(cy*n_cY_W - n_cY_W*s_cv_o);
	Cov.at<double>(5, 8) = Cov.at<double>(8, 5) = Cov.at<double>(8, 5) = (cx*n_cY_W - n_cY_W*s_cu_o)*(cx*v_cX_W - s_cu_o*v_cX_W) + (cy*n_cY_W - n_cY_W*s_cv_o)*(cy*v_cX_W - s_cv_o*v_cX_W) + (cx*n_cY_W - e_cu_o*n_cY_W)*(cx*v_cX_W - e_cu_o*v_cX_W) + (cy*n_cY_W - e_cv_o*n_cY_W)*(cy*v_cX_W - e_cv_o*v_cX_W);

	Cov.at<double>(6, 6) = 2 * pow(fx, 2) * pow(v_cX_W, 2);
	Cov.at<double>(6, 7) = Cov.at<double>(7, 6) = 0;
	Cov.at<double>(6, 8) = Cov.at<double>(8, 6) = fx*v_cX_W*(cx*v_cX_W - e_cu_o*v_cX_W) + fx*v_cX_W*(cx*v_cX_W - s_cu_o*v_cX_W);

	Cov.at<double>(7, 7) = 2 * pow(fy, 2) * pow(v_cX_W, 2);
	Cov.at<double>(7, 8) = Cov.at<double>(8, 7) = fy*v_cX_W*(cy*v_cX_W - e_cv_o*v_cX_W) + fy*v_cX_W*(cy*v_cX_W - s_cv_o*v_cX_W);

	Cov.at<double>(8, 8) = pow((cx*v_cX_W - e_cu_o*v_cX_W), 2) + pow((cy*v_cX_W - e_cv_o*v_cX_W), 2) + pow((cx*v_cX_W - s_cu_o*v_cX_W), 2) + pow((cy*v_cX_W - s_cv_o*v_cX_W), 2);

	
}


// 3. Convert theta to optimalR
void MinSampsonError::getOptimalR(const cv::Mat& theta,
	cv::Mat& optR, cv::Mat& optt)
{
	// 2-norm of every row of R =1

	// first row of R
	double r2 = theta.at<double>(0);
	double r3 = theta.at<double>(1);

	// second row of R
	double r5 = theta.at<double>(2);
	double r6 = theta.at<double>(3);

	// third row of R
	double r8 = theta.at<double>(4);
	double r9 = theta.at<double>(5);

	double t1 = theta.at<double>(6);
	double t2 = theta.at<double>(7);
	double t3 = theta.at<double>(8);


	double col2Arr[3] = { r2, r5, r8 };
	cv::Mat col2 = cv::Mat(3, 1, CV_64FC1, col2Arr).clone();

	double scale = cv::norm(col2);

	col2 = col2 / scale;

	double col3Arr[3] = { r3, r6, r9 };
	cv::Mat col3 = cv::Mat(3, 1, CV_64FC1, col3Arr).clone();

	col3 = col3 / scale;

	if (col3.at<double>(2) < 0)
	{
		col2 = -col2;
		col3 = -col3;
		t1 = -t1;
		t2 = -t2;
		t3 = -t3;
	}

	cv::Mat col1 = col2.cross(col3);

	optR = cv::Mat(3, 3, CV_64FC1);

	col1.copyTo(optR(cv::Rect(0, 0, 1, 3)));
	col2.copyTo(optR(cv::Rect(1, 0, 1, 3)));
	col3.copyTo(optR(cv::Rect(2, 0, 1, 3)));

	optt = cv::Mat::zeros(3, 1, CV_64FC1);
	optt.at<double>(0) = t1 / scale;
	optt.at<double>(1) = t2 / scale;
	optt.at<double>(2) = t3 / scale;

}


// General Function(Applied for every problem)

int MinSampsonError::iterationbyFNS(const std::vector<cv::Mat>& ksis, const std::vector<cv::Mat>& Covs,
	cv::Mat& theta)
{
	// iteratiion
	int numFeature = ksis.size();

	// initialization
	// Weights to update
	std::vector<double> weights(numFeature, 1); // initialize as 1

	// theta to update
	cv::Mat oldTheta = theta.clone();
	cv::Mat newTheta = theta.clone(); // initialize as zero-vec

	int iterNum = 0;
	while (1)
	{

		// Construct M & L
		cv::Mat M;
		cv::Mat L;
		this->constrMandL(weights, ksis, Covs, oldTheta,
			M, L);

		this->solveEig(M, L,
			newTheta);

		std::cout << "--- Iteratioin " << (iterNum + 1) << " ---" << std::endl;
		std::cout << "New Theta" << std::endl;
		//std::cout << newTheta << std::endl;

		iterNum++;

		double diff1 = cv::norm(oldTheta + newTheta); 
		double diff2 = cv::norm(oldTheta - newTheta);

		// break normally
		if (diff1 > 1e-2 && diff2 > 1e-2)
		{
			// update weights
			for (int i = 0; i < numFeature; i++)
			{
				weights[i] = 1.0 / newTheta.dot(Covs[i] * newTheta);  
			}

			// update theta
			oldTheta = newTheta.clone();
		}
		else
		{
			theta = newTheta.clone();
			break;
		}

		// break abnormally
		if (iterNum == 30)
		{
			break;
		}

	}

	return iterNum;
}


// Construct M Mat & L Mat, the last param is unknown vector to solve by iteration
void MinSampsonError::constrMandL(const std::vector<double>& weights, const std::vector<cv::Mat>& ksis, const std::vector<cv::Mat>& Covs, const cv::Mat& curTheta,
	cv::Mat& M, cv::Mat& L)
{
	M = cv::Mat::zeros(dimTheta, dimTheta, CV_64FC1);
	L = cv::Mat::zeros(dimTheta, dimTheta, CV_64FC1);

	int numFeature = weights.size();

	for (int i = 0; i < numFeature; i++)
	{
		// Construct Mi(Kantani12 Eq.(27), M)

		cv::Mat Mi = weights[i] * ksis[i] * (ksis[i].t());

		// Construct Li
		cv::Mat Lj = pow(weights[i], 2) * pow(curTheta.dot(ksis[i]), 2) * Covs[i];

		M = M + Mi;
		L = L + Lj;
	}

	M = M / numFeature;
	L = L / numFeature;
}


// Solve the eigen problem(use Eigen Library)
void MinSampsonError::solveEig(const cv::Mat& M, const cv::Mat& L,
	cv::Mat& newTheta_cv)
{
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, dimTheta, dimTheta>> eigenSolver; // solver for sym matrix

	cv::Mat S_cv = M - L;

	// Transfer cv format to Eigen format by mapping(Note that the necessary of T)
	S_cv = S_cv.t();

	double* pS = (double*)S_cv.data;
	Eigen::Map<Eigen::Matrix<double, dimTheta, dimTheta>> SMap(pS);
	Eigen::Matrix<double, dimTheta, dimTheta> S_Eig = SMap;

	eigenSolver.compute(S_Eig);

	// eigen vector related to smallset eigen value
	Eigen::Matrix<double, dimTheta, 1> newTheta_Eig = eigenSolver.eigenvectors().col(0); // similar to cv: col(i) return Mat, cols return num

	// Eig to cv
	newTheta_cv = cv::Mat::zeros(dimTheta, 1, CV_64FC1);

	for (int i = 0; i < dimTheta; i++)
	{
		newTheta_cv.at<double>(i) = newTheta_Eig(i);
	}

}
