#include "lidarIrisDescriptor.h"

lidar_iris_descriptor::lidar_iris_descriptor(
	int rows = 80,
	int columns = 360,
	int n_scan = 64,
	double distance_threshold = 0.45,
	int exclude_recent_frame_num = 30,
	int match_mode = 2,
	int candidates_num = 10,
	int nscale = 4,
	int min_wave_length = 18,
	float mult = 1.6,
	float sigma_on_f = 0.75,
	int robot_num = 1,
	int id = 0) : rows_(rows),										   // 80 in the original paper
				  columns_(columns),								   // 360 in the original paper
				  n_scan_(n_scan),									   // lidar sensor lines
				  distance_threshold_(distance_threshold),			   // 0.10-0.45 is ok.
																	   // 0.10-0.15 is very fine (rare false-alarms);
																	   // 0.15-0.35 is good but ICP fitness score check should be required for robustness;
																	   // 0.35-0.45 is choice for using with robust outlier rejection.
				  exclude_recent_frame_num_(exclude_recent_frame_num), // simply just keyframe gap
				  match_mode_(match_mode),							   // 0 for detecting same direction
																	   // 1 for detecting opposite direction
																	   // 2 for detecting both same and opposite direction
				  candidates_num_(candidates_num),					   // 6-10 is enough
				  nscale_(nscale),									   // 4 in the original paper
				  min_wave_length_(min_wave_length),				   // 18 in the original paper
				  mult_(mult),										   // 1.6 in the original paper
				  sigma_on_f_(sigma_on_f),							   // 0.75 in the original paper
				  robot_num_(robot_num),							   // number of robot in robotic swarm
				  id_(id)											   // this robot id
{
	// allocate memory
	for (int i = 0; i < robot_num_; i++)
	{
		unordered_map<int, lidar_iris_descriptor::featureDesc> base_feature_desc;
		unordered_map<int, Eigen::MatrixXf> base_matrix;
		unordered_map<int, int> base_int;
		iris_features.push_back(base_feature_desc);
		iris_rowkeys.push_back(base_matrix);
		indexes_maps.push_back(base_int);
	}
}

lidar_iris_descriptor::~lidar_iris_descriptor()
{
}

std::pair<Eigen::VectorXf, cv::Mat1b> lidar_iris_descriptor::getIris(
	const pcl::PointCloud<pcl::PointXYZI> &cloud)
{
	cv::Mat1b iris_image = cv::Mat1b::zeros(rows_, columns_);
	Eigen::MatrixXf iris_row_key_matrix = Eigen::MatrixXf::Zero(rows_, columns_);

	// extract lidar iris image提取激光雷达虹膜图像
	if (n_scan_ == 6) // livox
	{
		for (auto p : cloud.points)
		{
			float dis = sqrt(p.data[0] * p.data[0] + p.data[1] * p.data[1]); // xy-plane distance
			float arc = (atan2(p.data[2], dis) * 180.0f / M_PI) + 25;		 // [0, 26.9] deg.
			float yaw = (atan2(p.data[1], p.data[0]) * 180.0f / M_PI) + 180; // atan2: (-pi, pi] rad.
			int Q_dis = std::min(std::max((int)floor(dis), 0), (rows_ - 1));
			int Q_arc = std::min(std::max((int)floor(arc / 3.3), 0), 7);
			int Q_yaw = std::min(std::max((int)floor(yaw + 0.5), 0), (columns_ - 1));
			iris_image.at<uint8_t>(Q_dis, Q_yaw) |= (1 << Q_arc);
			if (iris_row_key_matrix(Q_dis, Q_yaw) < p.data[2])
			{
				iris_row_key_matrix(Q_dis, Q_yaw) = p.data[2]; // update for taking maximum value at that bin
			}
		}
	}
	else if (n_scan_ == 16) // VLP16
	{
		for (auto p : cloud.points)
		{
			float dis = sqrt(p.data[0] * p.data[0] + p.data[1] * p.data[1]);
			float arc = (atan2(p.data[2], dis) * 180.0f / M_PI) + 15; // [0, 30] deg.
			float yaw = (atan2(p.data[1], p.data[0]) * 180.0f / M_PI) + 180;
			int Q_dis = std::min(std::max((int)floor(dis), 0), (rows_ - 1));
			int Q_arc = std::min(std::max((int)floor(arc / 4.0f), 0), 7);
			// int Q_arc = std::min(std::max((int)ceil(p.data[2] + 5), 0), 7);
			int Q_yaw = std::min(std::max((int)floor(yaw + 0.5), 0), (columns_ - 1));
			iris_image.at<uint8_t>(Q_dis, Q_yaw) |= (1 << Q_arc);
			if (iris_row_key_matrix(Q_dis, Q_yaw) < p.data[2])
			{
				iris_row_key_matrix(Q_dis, Q_yaw) = p.data[2];
			}
		}
	}
	else if (n_scan_ == 64) // HDL64
	{
		for (auto p : cloud.points)
		{
			float dis = sqrt(p.data[0] * p.data[0] + p.data[1] * p.data[1]);
			// float arc = (atan2(p.data[2], dis) * 180.0f / M_PI) + 25; // [0, 26.9] deg.
			float yaw = (atan2(p.data[1], p.data[0]) * 180.0f / M_PI) + 180;
			int Q_dis = std::min(std::max((int)floor(dis), 0), (rows_ - 1));
			// int Q_arc = std::min(std::max((int)floor(arc / 3.3), 0), 7);
			int Q_arc = std::min(std::max((int)ceil(p.data[2] + 5), 0), 7);
			int Q_yaw = std::min(std::max((int)floor(yaw + 0.5), 0), (columns_ - 1));
			iris_image.at<uint8_t>(Q_dis, Q_yaw) |= (1 << Q_arc);
			if (iris_row_key_matrix(Q_dis, Q_yaw) < p.data[2])
			{
				iris_row_key_matrix(Q_dis, Q_yaw) = p.data[2];
			}
		}
	}
	else if (n_scan_ == 40) // K1
	{
		// ROS_INFO("K1:使用40线激光雷达");
		for (auto p : cloud.points)
		{
			float dis = sqrt(p.data[0] * p.data[0] + p.data[1] * p.data[1]);
			float arc = (atan2(p.data[2], dis) * 180.0f / M_PI) + 7; // K1是-7～52度.
			float yaw = (atan2(p.data[1], p.data[0]) * 180.0f / M_PI) + 180;
			int Q_dis = std::min(std::max((int)floor(dis), 0), (rows_ - 1));
			int Q_arc = std::min(std::max((int)floor(arc / 4.0f), 0), 7);
			// int Q_arc = std::min(std::max((int)ceil(p.data[2] + 5), 0), 7);
			int Q_yaw = std::min(std::max((int)floor(yaw + 0.5), 0), (columns_ - 1));
			iris_image.at<uint8_t>(Q_dis, Q_yaw) |= (1 << Q_arc);
			if (iris_row_key_matrix(Q_dis, Q_yaw) < p.data[2])
			{
				iris_row_key_matrix(Q_dis, Q_yaw) = p.data[2];
			}
		}
	}
	// extract rowkey
	Eigen::VectorXf rowkey = Eigen::VectorXf::Zero(rows_);
	for (int i = 0; i < iris_row_key_matrix.rows(); i++)
	{
		Eigen::VectorXf curr_row = iris_row_key_matrix.row(i);
		rowkey(i) = curr_row.mean();
	}

	return std::make_pair(rowkey, iris_image);
}

inline cv::Mat lidar_iris_descriptor::circRowShift(
	const cv::Mat &src,
	int shift_m_rows)
{
	if (shift_m_rows == 0)
	{
		return src.clone();
	}
	shift_m_rows %= src.rows;
	int m = shift_m_rows > 0 ? shift_m_rows : src.rows + shift_m_rows;
	cv::Mat dst(src.size(), src.type());
	src(cv::Range(src.rows - m, src.rows), cv::Range::all()).copyTo(dst(cv::Range(0, m), cv::Range::all()));
	src(cv::Range(0, src.rows - m), cv::Range::all()).copyTo(dst(cv::Range(m, src.rows), cv::Range::all()));
	return dst;
}

inline cv::Mat lidar_iris_descriptor::circColShift(
	const cv::Mat &src,
	int shift_n_cols)
{
	if (shift_n_cols == 0)
	{
		return src.clone();
	}
	shift_n_cols %= src.cols;
	int n = shift_n_cols > 0 ? shift_n_cols : src.cols + shift_n_cols;
	cv::Mat dst(src.size(), src.type());
	src(cv::Range::all(), cv::Range(src.cols - n, src.cols)).copyTo(dst(cv::Range::all(), cv::Range(0, n)));
	src(cv::Range::all(), cv::Range(0, src.cols - n)).copyTo(dst(cv::Range::all(), cv::Range(n, src.cols)));
	return dst;
}

cv::Mat lidar_iris_descriptor::circShift(
	const cv::Mat &src,
	int shift_m_rows,
	int shift_n_cols)
{
	return circColShift(circRowShift(src, shift_m_rows), shift_n_cols);
}

std::vector<cv::Mat2f> lidar_iris_descriptor::logGaborFilter(
	const cv::Mat1f &src,
	unsigned int nscale,
	int min_wave_length,
	double mult,
	double sigma_on_f)
{
	int rows = src.rows;
	int cols = src.cols;
	cv::Mat2f filtersum = cv::Mat2f::zeros(1, cols);
	std::vector<cv::Mat2f> EO(nscale);
	int ndata = cols;
	if (ndata % 2 == 1)
	{
		ndata--;
	}
	cv::Mat1f logGabor = cv::Mat1f::zeros(1, ndata);
	cv::Mat2f result = cv::Mat2f::zeros(rows, ndata);
	cv::Mat1f radius = cv::Mat1f::zeros(1, ndata / 2 + 1);
	radius.at<float>(0, 0) = 1;
	for (int i = 1; i < ndata / 2 + 1; i++)
	{
		radius.at<float>(0, i) = i / (float)ndata;
	}
	double wavelength = min_wave_length;
	for (int s = 0; s < nscale; s++)
	{
		double fo = 1.0 / wavelength;
		double rfo = fo / 0.5;

		cv::Mat1f temp; //(radius.size());
		cv::Mat1f radius_tmp = radius / fo;
		cv::log(radius_tmp, temp);
		cv::pow(temp, 2, temp);
		cv::Mat1f mid_result = (-temp) / (2 * log(sigma_on_f) * log(sigma_on_f));
		cv::exp(mid_result, temp);
		temp.copyTo(logGabor.colRange(0, ndata / 2 + 1));

		logGabor.at<float>(0, 0) = 0;
		cv::Mat2f filter;
		cv::Mat1f filterArr[2] = {logGabor, cv::Mat1f::zeros(logGabor.size())};
		cv::merge(filterArr, 2, filter);
		filtersum = filtersum + filter;
		for (int r = 0; r < rows; r++)
		{
			cv::Mat2f src2f;
			cv::Mat1f srcArr[2] = {src.row(r).clone(), cv::Mat1f::zeros(1, src.cols)};
			cv::merge(srcArr, 2, src2f);
			cv::dft(src2f, src2f);
			cv::mulSpectrums(src2f, filter, src2f, 0);
			cv::idft(src2f, src2f);
			src2f.copyTo(result.row(r));
		}
		EO[s] = result.clone();
		wavelength *= mult;
	}
	filtersum = circShift(filtersum, 0, cols / 2);
	return EO;
}

void lidar_iris_descriptor::logFeatureEncode(
	const cv::Mat1b &src,
	unsigned int nscale,
	int min_wave_length,
	double mult,
	double sigma_on_f,
	cv::Mat1b &T,
	cv::Mat1b &M)
{
	cv::Mat1f src_float;
	src.convertTo(src_float, CV_32FC1);
	auto list = logGaborFilter(src_float, nscale, min_wave_length, mult, sigma_on_f);
	std::vector<cv::Mat1b> Tlist(nscale * 2), Mlist(nscale * 2);
	for (int i = 0; i < list.size(); i++)
	{
		cv::Mat1f arr[2];
		cv::split(list[i], arr);
		Tlist[i] = arr[0] > 0;
		Tlist[i + nscale] = arr[1] > 0;

		cv::Mat1f m;
		cv::magnitude(arr[0], arr[1], m);
		Mlist[i] = m < 0.0001;
		Mlist[i + nscale] = m < 0.0001;
	}
	cv::vconcat(Tlist, T);
	cv::vconcat(Mlist, M);
}

lidar_iris_descriptor::featureDesc lidar_iris_descriptor::getFeature(
	const cv::Mat1b &src)
{
	lidar_iris_descriptor::featureDesc desc;
	desc.img = src;
	logFeatureEncode(src, nscale_, min_wave_length_, mult_, sigma_on_f_, desc.T, desc.M);
	return desc;
}

lidar_iris_descriptor::featureDesc lidar_iris_descriptor::getFeature(
	const cv::Mat1b &src,
	std::vector<float> &vec)
{
	cv::Mat1f temp;
	src.convertTo(temp, CV_32FC1);
	cv::reduce((temp != 0) / 255, temp, 1, cv::REDUCE_AVG);
	vec = temp.isContinuous() ? temp : temp.clone();
	return getFeature(src);
}

void lidar_iris_descriptor::recomb(
	cv::Mat &src,
	cv::Mat &dst) // Recombinate image quaters
{
	int cx = src.cols >> 1;
	int cy = src.rows >> 1;
	cv::Mat tmp;
	tmp.create(src.size(), src.type());
	src(cv::Rect(0, 0, cx, cy)).copyTo(tmp(cv::Rect(cx, cy, cx, cy)));
	src(cv::Rect(cx, cy, cx, cy)).copyTo(tmp(cv::Rect(0, 0, cx, cy)));
	src(cv::Rect(cx, 0, cx, cy)).copyTo(tmp(cv::Rect(0, cy, cx, cy)));
	src(cv::Rect(0, cy, cx, cy)).copyTo(tmp(cv::Rect(cx, 0, cx, cy)));
	dst = tmp;
}

void lidar_iris_descriptor::forwardFFT(
	cv::Mat &src,
	cv::Mat *f_img,
	bool do_recomb = true) // 2D Forward FFT
{
	int M = cv::getOptimalDFTSize(src.rows);
	int N = cv::getOptimalDFTSize(src.cols);
	cv::Mat padded;
	copyMakeBorder(src, padded, 0, M - src.rows, 0, N - src.cols, cv::BORDER_CONSTANT, cv::Scalar::all(0));
	cv::Mat planes[] = {cv::Mat_<float>(padded), cv::Mat::zeros(padded.size(), CV_32F)};
	cv::Mat complex_img;
	merge(planes, 2, complex_img);
	dft(complex_img, complex_img);
	split(complex_img, planes);
	planes[0] = planes[0](cv::Rect(0, 0, planes[0].cols & -2, planes[0].rows & -2));
	planes[1] = planes[1](cv::Rect(0, 0, planes[1].cols & -2, planes[1].rows & -2));
	if (do_recomb)
	{
		recomb(planes[0], planes[0]);
		recomb(planes[1], planes[1]);
	}
	planes[0] /= float(M * N);
	planes[1] /= float(M * N);
	f_img[0] = planes[0].clone();
	f_img[1] = planes[1].clone();
}

void lidar_iris_descriptor::highpass(cv::Size sz, cv::Mat &dst)
{
	cv::Mat a = cv::Mat(sz.height, 1, CV_32FC1);
	cv::Mat b = cv::Mat(1, sz.width, CV_32FC1);

	float step_y = CV_PI / sz.height;
	float val = -CV_PI * 0.5;

	for (int i = 0; i < sz.height; ++i)
	{
		a.at<float>(i) = cos(val);
		val += step_y;
	}

	val = -CV_PI * 0.5;
	float step_x = CV_PI / sz.width;
	for (int i = 0; i < sz.width; ++i)
	{
		b.at<float>(i) = cos(val);
		val += step_x;
	}

	cv::Mat tmp = a * b;
	dst = (1.0 - tmp).mul(2.0 - tmp);
}

float lidar_iris_descriptor::logpolar(
	cv::Mat &src,
	cv::Mat &dst)
{
	float radii = src.cols;
	float angles = src.rows;
	cv::Point2f center(src.cols / 2, src.rows / 2);
	float d = cv::norm(cv::Vec2f(src.cols - center.x, src.rows - center.y));
	float log_base = std::pow(10.0, log10(d) / radii);
	float d_theta = CV_PI / (float)angles;
	float theta = CV_PI / 2.0;
	float radius = 0;
	cv::Mat map_x(src.size(), CV_32FC1);
	cv::Mat map_y(src.size(), CV_32FC1);
	for (int i = 0; i < angles; ++i)
	{
		for (int j = 0; j < radii; ++j)
		{
			radius = std::pow(log_base, float(j));
			float x = radius * sin(theta) + center.x;
			float y = radius * cos(theta) + center.y;
			map_x.at<float>(i, j) = x;
			map_y.at<float>(i, j) = y;
		}
		theta += d_theta;
	}
	cv::remap(src, dst, map_x, map_y, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
	return log_base;
}

cv::RotatedRect lidar_iris_descriptor::logPolarFFTTemplateMatch(
	cv::Mat &im0,
	cv::Mat &im1 /*, double canny_threshold1, double canny_threshold2*/)
{
	// Accept 1 or 3 channel CV_8U, CV_32F or CV_64F images.
	CV_Assert((im0.type() == CV_8UC1) || (im0.type() == CV_8UC3) ||
			  (im0.type() == CV_32FC1) || (im0.type() == CV_32FC3) ||
			  (im0.type() == CV_64FC1) || (im0.type() == CV_64FC3));

	CV_Assert(im0.rows == im1.rows && im0.cols == im1.cols);

	CV_Assert(im0.channels() == 1 || im0.channels() == 3 || im0.channels() == 4);

	CV_Assert(im1.channels() == 1 || im1.channels() == 3 || im1.channels() == 4);

	// cv::Mat im0_tmp = im0.clone();
	// cv::Mat im1_tmp = im1.clone();
	if (im0.channels() == 3)
	{
		cv::cvtColor(im0, im0, cv::ColorConversionCodes::COLOR_BGR2GRAY);
	}

	if (im0.channels() == 4)
	{
		cv::cvtColor(im0, im0, cv::ColorConversionCodes::COLOR_BGRA2GRAY);
	}

	if (im1.channels() == 3)
	{
		cv::cvtColor(im1, im1, cv::ColorConversionCodes::COLOR_BGR2GRAY);
	}

	if (im1.channels() == 4)
	{
		cv::cvtColor(im1, im1, cv::ColorConversionCodes::COLOR_BGRA2GRAY);
	}

	if (im0.type() == CV_32FC1)
	{
		im0.convertTo(im0, CV_8UC1, 255.0);
	}

	if (im1.type() == CV_32FC1)
	{
		im1.convertTo(im1, CV_8UC1, 255.0);
	}

	if (im0.type() == CV_64FC1)
	{
		im0.convertTo(im0, CV_8UC1, 255.0);
	}

	if (im1.type() == CV_64FC1)
	{
		im1.convertTo(im1, CV_8UC1, 255.0);
	}

	// Canny(im0, im0, canny_threshold1, canny_threshold2); // you can change this
	// Canny(im1, im1, canny_threshold1, canny_threshold2);

	// Ensure both images are of CV_32FC1 type
	im0.convertTo(im0, CV_32FC1, 1.0 / 255.0);
	im1.convertTo(im1, CV_32FC1, 1.0 / 255.0);

	cv::Mat F0[2], F1[2];
	cv::Mat f0, f1;
	forwardFFT(im0, F0);
	forwardFFT(im1, F1);
	cv::magnitude(F0[0], F0[1], f0);
	cv::magnitude(F1[0], F1[1], f1);

	// Create filter
	cv::Mat h;
	highpass(f0.size(), h);

	// Apply it in freq domain
	f0 = f0.mul(h);
	f1 = f1.mul(h);

	float log_base;
	cv::Mat f0lp, f1lp;

	log_base = logpolar(f0, f0lp);
	log_base = logpolar(f1, f1lp);

	// Find rotation and scale
	cv::Point2d rotation_and_scale = cv::phaseCorrelate(f1lp, f0lp);

	float angle = 180.0 * rotation_and_scale.y / f0lp.rows;
	float scale = pow(log_base, rotation_and_scale.x);
	// --------------
	if (scale > 1.8)
	{
		rotation_and_scale = cv::phaseCorrelate(f1lp, f0lp);
		angle = -180.0 * rotation_and_scale.y / f0lp.rows;
		scale = 1.0 / pow(log_base, rotation_and_scale.x);
		if (scale > 1.8)
		{
			std::cout << "Images are not compatible. Scale change > 1.8" << std::endl;
			return cv::RotatedRect();
		}
	}
	// --------------
	if (angle < -90.0)
	{
		angle += 180.0;
	}
	else if (angle > 90.0)
	{
		angle -= 180.0;
	}

	// Now rotate and scale fragment back, then find translation
	cv::Mat rot_mat = cv::getRotationMatrix2D(cv::Point(im1.cols / 2, im1.rows / 2), angle, 1.0 / scale);

	// rotate and scale
	cv::Mat im1_rs;
	cv::warpAffine(im1, im1_rs, rot_mat, im1.size());

	// find translation
	cv::Point2d tr = cv::phaseCorrelate(im1_rs, im0);

	// compute rotated rectangle parameters
	cv::RotatedRect rr;
	rr.center = tr + cv::Point2d(im0.cols / 2, im0.rows / 2);
	rr.angle = -angle;
	rr.size.width = im1.cols / scale;
	rr.size.height = im1.rows / scale;

	// im0 = im0_tmp.clone();
	// im1 = im1_tmp.clone();

	return rr;
}

cv::RotatedRect lidar_iris_descriptor::fftMatch(
	const cv::Mat &im0,
	const cv::Mat &im1)
{
	cv::Mat im0_tmp = im0.clone();
	cv::Mat im1_tmp = im1.clone();
	return logPolarFFTTemplateMatch(im0_tmp, im1_tmp);
}

void lidar_iris_descriptor::getHammingDistance(
	const cv::Mat1b &T1,
	const cv::Mat1b &M1,
	const cv::Mat1b &T2,
	const cv::Mat1b &M2,
	int scale,
	float &dis,
	int &bias)
{
	dis = NAN;
	bias = -1;
	// #pragma omp parallel for num_threads(8)
	for (int shift = scale - 2; shift <= scale + 2; shift++)
	{
		cv::Mat1b T1s = circShift(T1, 0, shift);
		cv::Mat1b M1s = circShift(M1, 0, shift);
		cv::Mat1b mask = M1s | M2;
		cv::Mat1b mask_tmp = mask / 255;
		int MaskBitsNum = cv::sum(mask_tmp)[0];
		int totalBits = T1s.rows * T1s.cols - MaskBitsNum;
		cv::Mat1b C = T1s ^ T2;
		C = C & ~mask;
		cv::Mat1b c_tmp = C / 255;
		int bitsDiff = cv::sum(c_tmp)[0];
		if (totalBits == 0)
		{
			dis = NAN;
		}
		else
		{
			float currentDis = bitsDiff / (float)totalBits;
			if (currentDis < dis || isnan(dis))
			{
				dis = currentDis;
				bias = shift;
			}
		}
	}
	return;
}

float lidar_iris_descriptor::compare(
	const lidar_iris_descriptor::featureDesc &img1,
	const lidar_iris_descriptor::featureDesc &img2,
	int *bias)
{
	if (match_mode_ == 2)
	{
		float dis1;
		int bias1;
		float dis2 = 0;
		int bias2 = 0;
		// #pragma omp parallel for num_threads(8)
		for (int i = 0; i < 2; i++)
		{
			if (i == 0)
			{
				auto firstRect = fftMatch(img2.img, img1.img);
				int firstShift = firstRect.center.x - img1.img.cols / 2;
				getHammingDistance(img1.T, img1.M, img2.T, img2.M, firstShift, dis1, bias1);
			}
			else
			{
				auto T2x = circShift(img2.T, 0, 180);
				auto M2x = circShift(img2.M, 0, 180);
				auto img2x = circShift(img2.img, 0, 180);

				auto secondRect = fftMatch(img2x, img1.img);
				int secondShift = secondRect.center.x - img1.img.cols / 2;
				getHammingDistance(img1.T, img1.M, T2x, M2x, secondShift, dis2, bias2);
			}
		}

		if (dis1 < dis2)
		{
			if (bias)
				*bias = bias1;
			return dis1;
		}
		else
		{
			if (bias)
				*bias = (bias2 + 180) % 360;
			return dis2;
		}
	}
	if (match_mode_ == 1)
	{
		auto T2x = circShift(img2.T, 0, 180);
		auto M2x = circShift(img2.M, 0, 180);
		auto img2x = circShift(img2.img, 0, 180);

		auto secondRect = fftMatch(img2x, img1.img);
		int secondShift = secondRect.center.x - img1.img.cols / 2;
		float dis2 = 0;
		int bias2 = 0;
		getHammingDistance(img1.T, img1.M, T2x, M2x, secondShift, dis2, bias2);
		if (bias)
			*bias = (bias2 + 180) % 360;
		return dis2;
	}
	if (match_mode_ == 0)
	{
		auto firstRect = fftMatch(img2.img, img1.img);
		int firstShift = firstRect.center.x - img1.img.cols / 2;
		float dis1;
		int bias1;
		getHammingDistance(img1.T, img1.M, img2.T, img2.M, firstShift, dis1, bias1);
		if (bias)
			*bias = bias1;
		return dis1;
	}
}

// user-side API
void lidar_iris_descriptor::saveDescriptorAndKey(
	const float *iris,
	const int8_t robot,
	const int index)
{
	cv::Mat1b iris_map = cv::Mat1b::zeros(rows_, columns_);
	Eigen::VectorXf rowkey = Eigen::VectorXf::Zero(rows_);
	for (int row_idx = 0; row_idx < iris_map.rows; row_idx++)
	{
		for (int col_idx = 0; col_idx < iris_map.cols; col_idx++)
		{
			iris_map(row_idx, col_idx) = iris[row_idx * (iris_map.cols) + col_idx];
		}
	}

	for (int row_idx = 0; row_idx < iris_map.rows; row_idx++)
	{
		rowkey(row_idx) = iris[row_idx + iris_map.rows * iris_map.cols];
	}

	save(iris_map, rowkey, robot, index);
}

void lidar_iris_descriptor::save(
	const cv::Mat1b iris,
	Eigen::MatrixXf rowkey,
	const int8_t robot,
	const int index)
{
	std::vector<float> rowKeyVec;
	auto irisFeature = getFeature(iris);

	// iris freature (descriptor) for single robot
	iris_features[robot].emplace(make_pair(index, irisFeature));
	// rowkey for knn search
	// iris_rowkeys[robot].conservativeResize(rows_, iris_features[robot].size());
	// iris_rowkeys[robot].block(0, iris_features[robot].size()-1, rows_, 1) = rowkey.block(0, 0, rows_, 1);
	iris_rowkeys[robot].emplace(make_pair(index, rowkey));
	// trasform local index to global
	indexes_maps[robot].emplace(make_pair(index, iris_feature_index_pairs.size()));
	// descriptor global index
	iris_feature_index_pairs.push_back(std::make_pair(robot, index)); // index
}

std::vector<float> lidar_iris_descriptor::makeAndSaveDescriptorAndKey(
	const pcl::PointCloud<pcl::PointXYZI> &scan,
	const int8_t robot,
	const int index)
{
	auto scan_iris_image = getIris(scan);
	save(scan_iris_image.second, scan_iris_image.first, robot, index);

	std::vector<float> descriptor_msg_data;
	for (int row_idx = 0; row_idx < scan_iris_image.second.rows; row_idx++)
	{
		for (int col_idx = 0; col_idx < scan_iris_image.second.cols; col_idx++)
		{
			descriptor_msg_data.push_back(scan_iris_image.second(row_idx, col_idx));
		}
	}

	for (int row_idx = 0; row_idx < scan_iris_image.second.rows; row_idx++)
	{
		descriptor_msg_data.push_back(scan_iris_image.first(row_idx));
	}

	return descriptor_msg_data;
}

std::pair<int, float> lidar_iris_descriptor::detectIntraLoopClosureID(
	const int cur_ptr)
{
	std::pair<int, float> result{-1, 0.0};
	Eigen::VectorXf iris_rowkey = iris_rowkeys[id_][cur_ptr];					   // current query rowkey
	lidar_iris_descriptor::featureDesc iris_feature = iris_features[id_][cur_ptr]; // current feature

	if (cur_ptr < exclude_recent_frame_num_ + candidates_num_ + 1)
	{
		return result; // early return
	}

	// step 1: candidates from rowkey tree
	// kd tree construction
	Eigen::MatrixXf new_iris_rowkeys;
	// int history_indexes = cur_ptr - exclude_recent_frame_num_;
	int cur_row = 0;
	for (auto iris_rowkey : iris_rowkeys[id_])
	{
		if (iris_rowkey.first > cur_ptr - exclude_recent_frame_num_)
		{
			continue;
		}

		new_iris_rowkeys.conservativeResize(rows_, cur_row + 1);
		new_iris_rowkeys.block(0, cur_row, rows_, 1) = iris_rowkey.second.block(0, 0, rows_, 1);
		cur_row++;
	}
	// new_iris_rowkeys.conservativeResize(rows_, history_indexes);
	// new_iris_rowkeys.block(0, 0, rows_, history_indexes) = iris_rowkeys[id_].block(0, 0, rows_, history_indexes);
	kdTree = Nabo::NNSearchF::createKDTreeTreeHeap(new_iris_rowkeys, rows_);

	// search n nearest neighbors
	Eigen::VectorXi indice(candidates_num_);
	Eigen::VectorXf distance(candidates_num_);
	float min_distance = 10000000.0;
	int min_index = -1;
	int min_bias = 0;

	// knn search
	kdTree->knn(iris_rowkey, indice, distance, candidates_num_);

	// step 2: pairwise distance
	for (int i = 0; i < std::min(candidates_num_, int(indice.size())); i++)
	{
		if (indice[i] >= indexes_maps[id_].size())
		{
			continue;
		}

		int bias;
		lidar_iris_descriptor::featureDesc candidate_iris_feature = iris_features[id_][indice[i]];
		float candidate_distance = compare(iris_feature, candidate_iris_feature, &bias);

		if (candidate_distance < min_distance)
		{
			min_distance = candidate_distance;
			min_index = indice[i];
			min_bias = bias;
		}
	}

	// threshold check
	if (min_distance < distance_threshold_)
	{
		result.first = min_index;
		result.second = min_bias;
		ROS_DEBUG("\033[1;33m[Iris Intra Loop<%d>] btn %d and %d. Dis: %.2f.\033[0m",
				  id_, cur_ptr, min_index, min_distance);
	}
	else
	{
		ROS_DEBUG("\033[1;33m[Iris Intra Not loop<%d>] btn %d and %d. Dis: %.2f.\033[0m",
				  id_, cur_ptr, min_index, min_distance);
	}
	return result;
}

std::pair<int, float> lidar_iris_descriptor::detectInterLoopClosureID(
	const int cur_ptr)
{
	std::pair<int, float> result{-1, 0.0};
	int robot_id = iris_feature_index_pairs[cur_ptr].first;
	int frame_id = iris_feature_index_pairs[cur_ptr].second;
	Eigen::VectorXf iris_rowkey = iris_rowkeys[robot_id][frame_id];						 // current query rowkey
	lidar_iris_descriptor::featureDesc iris_feature = iris_features[robot_id][frame_id]; // current feature

	// step 1: candidates from rowkey tree
	// kd tree construction
	float min_distance = 10000000.0;
	int min_index = -1;
	int min_bias = 0;
	for (int i = 0; i < robot_num_; i++)
	{
		if (robot_id == id_ && i == id_ || robot_id != id_ && i != id_)
		{
			continue;
		}

		Eigen::MatrixXf new_iris_rowkeys;
		std::vector<int> new_indexes_maps;
		std::vector<lidar_iris_descriptor::featureDesc> new_iris_features;

		if (indexes_maps[i].size() > 0)
		{
			for (auto iris_rowkey : iris_rowkeys[i])
			{
				int cur_row = new_iris_rowkeys.cols();
				new_iris_rowkeys.conservativeResize(rows_, cur_row + 1);
				new_iris_rowkeys.block(0, cur_row, rows_, 1) = iris_rowkey.second.block(0, 0, rows_, 1);
			}

			for (auto indexes_map : indexes_maps[i])
			{
				new_indexes_maps.emplace_back(indexes_map.second);
			}

			for (auto iris_feature : iris_features[i])
			{
				new_iris_features.emplace_back(iris_feature.second);
			}
		}

		if (new_indexes_maps.size() > candidates_num_ + 2)
		{
			kdTree = Nabo::NNSearchF::createKDTreeLinearHeap(new_iris_rowkeys, rows_);

			// search n nearest neighbors
			Eigen::VectorXi indice(candidates_num_);
			Eigen::VectorXf distance(candidates_num_);

			// knn search
			kdTree->knn(iris_rowkey, indice, distance, candidates_num_);

			// step 2: pairwise distance
			// #pragma omp parallel for num_threads(4)
			for (int i = 0; i < std::min(candidates_num_, int(indice.size())); i++)
			{
				if (indice[i] >= new_indexes_maps.size())
				{
					continue;
				}

				int bias;
				lidar_iris_descriptor::featureDesc candidate_iris_feature = new_iris_features[indice[i]];
				float candidate_distance = compare(iris_feature, candidate_iris_feature, &bias);

				if (candidate_distance < min_distance)
				{
					min_distance = candidate_distance;
					min_index = new_indexes_maps[indice[i]];
					min_bias = bias;
				}
			}
		}
	}

	// threshold check
	if (min_distance < distance_threshold_)
	{
		result.first = min_index;
		result.second = min_bias;
		ROS_DEBUG("\033[1;33m[Iris Inter Loop<%d>] btn %d-%d and %d-%d. Dis: %.2f. Bias:%d\033[0m", id_,
				  iris_feature_index_pairs[cur_ptr].first, iris_feature_index_pairs[cur_ptr].second,
				  iris_feature_index_pairs[min_index].first, iris_feature_index_pairs[min_index].second, min_distance, min_bias);
	}
	else
	{
		ROS_DEBUG("\033[1;33m[Iris Inter Not loop<%d>] btn %d-%d and %d-%d. Dis: %.2f. Bias:%d\033[0m", id_,
				  iris_feature_index_pairs[cur_ptr].first, iris_feature_index_pairs[cur_ptr].second,
				  iris_feature_index_pairs[min_index].first, iris_feature_index_pairs[min_index].second, min_distance, min_bias);
	}
	return result;
}

std::pair<int8_t, int> lidar_iris_descriptor::getIndex(
	const int key)
{
	return iris_feature_index_pairs[key];
}

int lidar_iris_descriptor::getSize(
	const int id = -1)
{
	if (id == -1)
	{
		return iris_feature_index_pairs.size();
	}
	else
	{
		return indexes_maps[id].size();
	}
}
