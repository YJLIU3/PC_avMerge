#include <fstream>
#include "LSD.hpp"
#include "panorama.h"
#include "parameter.h"
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include<time.h> 
#include "fftm.hpp"

static Mat front_before;
static Mat front_now;
static Mat rear_before;
static Mat rear_now;

static Mat test_before;
static Mat test_now;

static Mat matrix_bypast[25];
int bypast_cont = 0;

#define grid_size 20
#define min_match 5
#define pi 3.1415926
static Mat imMaskS;
static Mat imTime;
static Mat ims;
static Mat imTime2;
static Mat imMask2;
//static Mat im2;
static Mat matrix;
static Mat expand_roi;
static Mat merge_roi;
extern Mat frontimage, rearimage;
static Mat output(Size_BG, CV_8UC4);
static Mat Highlander;


static Mat front(image_size, CV_8UC4, Scalar::all(0));
static Mat rear(image_size, CV_8UC4, Scalar::all(0));


static Mat im1;
static Mat im2;

Panorama::Panorama()
{

}

Panorama::~Panorama()
{

}


void Panorama::compute_merge_matrix(Mat frontChessboard, Mat rearChessboard,
	Size board_size, int offset_x, int offset_y)
{
	Mat frontChessboardGray, rearChessboardGray;
	vector<Point2f> rearCornersArray1, frontCornersArray1;
	vector<Point2f> frontCorners, rearCorners;
	if (0)
	{
	}
	else
	{
		Point2f frontCornersArray[40];
		Point2f rearCornersArray[40];
		Point2f F2R[40];

		for (int i = 0; i < 10; i++)
		{
			if (i == 0)
			{
				F2R[i].x = 0;
				F2R[i + 10].x = 0;
				F2R[i + 20].x = 0;
				F2R[i + 30].x = 0;
			}
			else
			{
				F2R[i].x = i * grid_size - 1;
				F2R[i + 10].x = i * grid_size - 1;
				F2R[i + 20].x = i * grid_size - 1;
				F2R[i + 30].x = i * grid_size - 1;
			}

			F2R[i].y = 0;
			F2R[i + 10].y = grid_size - 1;
			F2R[i + 20].y = grid_size * 2 - 1;
			F2R[i + 30].y = grid_size * 3 - 1;
		}
		for (int i = 0; i < 40; i++)
		{
			frontCornersArray[i] = F2R[i];
		}
		for (int i = 0; i < 40; i++)
		{
			rearCornersArray[i] = F2R[39 - i];
		}

		rearCornersArray1.clear(); frontCornersArray1.clear();
		for (int i = 0; i < 40; i++)
		{
			frontCornersArray[i].y += offset_y + 61;
			frontCornersArray[i].x += offset_x;
			rearCornersArray1.push_back(rearCornersArray[i]);
			frontCornersArray1.push_back(frontCornersArray[i]);
		}
		merge_matrix = estimateRigidTransform(rearCornersArray1, frontCornersArray1, false);
	}
}
void Panorama::mergeFrontRearMat(Mat frontMat, Mat rearMat, Mat& out)
{
	Mat ROI_rear = Mat(out, Rect((output.cols - rearMat.cols) * 0.5, out.rows - rearMat.rows, rearMat.cols, rearMat.rows));
	rearMat.copyTo(ROI_rear);
	merge_roi = Mat(out, Rect((out.cols - frontMat.cols) * 0.5, 0, frontMat.cols, frontMat.rows));
	frontMat.copyTo(merge_roi);
}


void Panorama::mergeFrontMat(Mat frontMat, Mat& out)
{
	merge_roi = Mat(out, Rect((out.cols - frontMat.cols) * 0.5, 0, frontMat.cols, frontMat.rows));
	frontMat.copyTo(merge_roi);
}
void Panorama::mergeRearMat(Mat rearMat, Mat& out)
{
	Mat ROI_rear = Mat(out, Rect((output.cols - rearMat.cols) * 0.5, out.rows - rearMat.rows, rearMat.cols, rearMat.rows));
	rearMat.copyTo(ROI_rear);
}


void create_timeImg_from_mask(Mat mask, Mat& imTime, int curIdx)
{
	imTime.create(mask.size(), CV_8UC1);
	threshold(mask, imTime, 0, 1, CV_THRESH_BINARY);
	imTime.convertTo(imTime, CV_32FC1);
	imTime = imTime * curIdx;
}


Mat compute_alpha(Mat mask1, Mat mask2, Mat time1, Mat time2, float timeRatioThrd)
{
	Mat alpha(mask1.size(), CV_32FC1, Scalar::all(0));
	for (int i = 0; i < alpha.rows; i++)
	{
		for (int j = 0; j < alpha.cols; j++)
		{
			double ratioTime = 1.0 - (IMAGE_ELEM(time2, float, i, j) - IMAGE_ELEM(time1, float, i, j)) / timeRatioThrd;
			if (ratioTime > 1.0)
				ratioTime = 1.0;
			if (IMAGE_ELEM(mask1, float, i, j) + IMAGE_ELEM(mask2, float, i, j) > 0)
			{
				IMAGE_ELEM(alpha, float, i, j) = IMAGE_ELEM(mask1, float, i, j) * ratioTime /
					(IMAGE_ELEM(mask1, float, i, j) + IMAGE_ELEM(mask2, float, i, j));
			}
			else
			{
				IMAGE_ELEM(alpha, float, i, j) = 1.0;
			}
		}
	}
	return alpha;

}

void mix_image_front(Mat image1, Mat image2, Mat alpha, Mat alpha_1, Mat& output)
{
#if 1	
	Mat Image1_ROI = image1(Rect(0, front.rows, image1.cols, image1.rows - front.rows));
	Mat Image2_ROI = image2(Rect(0, front.rows, image1.cols, image1.rows - front.rows));
	Image1_ROI.copyTo(Image2_ROI);
	//imwrite("Image1_ROI.png", Image1_ROI);
	output = image2;
#endif
}

void mix_image_rear(Mat &image1, Mat &image2, double width)
{
#if 0

	Mat Image1_ROI = image1(Rect(0, 0, 630, image1.rows - 300));
	Mat Image2_ROI = image2(Rect(0, 0, 630, image1.rows - 300));
	Image1_ROI.copyTo(Image2_ROI);
	//imwrite("Image1_ROI.png", Image1_ROI);
	output = image2;
#else
	Mat Image1_ROI = image1(Rect(0, 0, 640, image1.rows - 300));
	Mat Image2_ROI = image2(Rect(0, 0, 640, image1.rows - 300));
	Image1_ROI.copyTo(Image2_ROI);
	for (int i = image1.rows - 300; i < image1.rows - abs(width); i++)
	{
		float alpha = (i - image1.rows + 300) / (300 - abs(width));
		for (int j = 0; j < image1.cols; j++)
		{
			Vec4b t = image2.at<Vec4b>(i, j);
			image2.at<Vec4b>(i, j)[0] = alpha * image2.at<Vec4b>(i, j)[0] + (1 - alpha) * image1.at<Vec4b>(i, j)[0];
			image2.at<Vec4b>(i, j)[1] = alpha * image2.at<Vec4b>(i, j)[1] + (1 - alpha) * image1.at<Vec4b>(i, j)[1];
			image2.at<Vec4b>(i, j)[3] = alpha * image2.at<Vec4b>(i, j)[3] + (1 - alpha) * image1.at<Vec4b>(i, j)[3];
			image2.at<Vec4b>(i, j)[2] = alpha * image2.at<Vec4b>(i, j)[2] + (1 - alpha) * image1.at<Vec4b>(i, j)[2];
		}
	}
	
#endif
}




void Panorama::expand(Mat input, Mat& output)
{
	output = Mat(Size_BG, input.type(), Scalar::all(0));
}
int isColor1 = 1;
int fps1 = 25;
int frameWidth1 = 320;
int frameHeight1 = 640;
VideoWriter writer1("tx_ty.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), fps1, Size(frameWidth1, frameHeight1), isColor1);

Mat Panorama::front_process(Mat front, Mat rear)
{
	if (bypast_cont > 23)
		bypast_cont = 0;

	if (!im1.data)
	{
		idx = 1;


		expand(front, im1);
		mergeFrontMat(front, im1);

		imMask1 = weight.clone();
		create_timeImg_from_mask(imMask1, imTime1, 1);

		cvtColor(front, front, CV_RGB2GRAY);

		preImg = im1;

		front_before = front;
		output = im1;
	}
	else
	{
		idx++;
		clock_t a = clock();
		if (idx == 2)
		{
			imMask2 = weight.clone();

			create_timeImg_from_mask(imMask2, imTime2, idx);

			expand(front, im2);
		}


		//        flip(rear, rear, -1);

		mergeFrontMat(front, im2);

		cvtColor(front, front, CV_RGB2GRAY);
		//		cvtColor(rear, rear, CV_RGB2GRAY);

		front_now = front;
		//        rear_now = rear;
		clock_t b = clock();
		if (DEBUG_MSG)
			cout << "Before matrix Running time  is: " << static_cast<double>(b - a) / CLOCKS_PER_SEC * 1000 << "ms" << endl;

		clock_t warp_st1 = clock();
		float angle;
		Mat matrix = LogPolarFFTTemplateMatch(front_before, front_now, angle, 200, 100, idx);
		clock_t warp_st2 = clock();
		if (DEBUG_MSG)
			cout << "Compute_matrix Running time  is: " << static_cast<double>(warp_st2 - warp_st1) / CLOCKS_PER_SEC * 1000 << "ms" << endl;

		clock_t warp_st3 = clock();


		bool deltaX = abs((int)matrix.at<double>(0, 2)) > 5;
		bool deltaY = abs((int)matrix.at<double>(1, 2)) > 20;



		if (deltaX || deltaY)
		{
			matrix = matrix_back;
		}


		matrix_bypast[bypast_cont++] = matrix;
		matrix = Mat(2, 3, CV_64FC1, Scalar(0.0));

		if (idx < 25)
		{
			for (size_t i = 0; i < bypast_cont; i++)
			{

				matrix += (matrix_bypast[i] / (bypast_cont));

			}
		}

		else
		{
			for (size_t i = 0; i < 24; i++)
			{
				matrix += matrix_bypast[i] / 24.0;
			}
		}

		matrix_back = matrix;

		if (1)
		{
			matrix_zero.at<double>(0, 2) = matrix.at<double>(0, 2);
			matrix_zero.at<double>(1, 2) = matrix.at<double>(1, 2);
			if (DEBUG_MSG)
				cout << "*****************" << matrix.at<double>(0, 2) / matrix.at<double>(1, 2) << endl;
			matrix = matrix_zero;
		}

		clock_t warp_st4 = clock();
		if (DEBUG_MSG)
			cout << "Process_matrix Running time  is: " << static_cast<double>(warp_st4 - warp_st3) / CLOCKS_PER_SEC * 1000 << "ms" << endl;

		if (DEBUG_MSG)
			cout << "+++++++++++++Current speed is++++++++++" << abs(matrix.at<double>(1, 2)*0.25)*3.6 << "Km/h" << endl;

		clock_t warp_st = clock();
        warpAffine(im1, im1t, matrix, WEIGHT_BIGSIZE, INTER_NEAREST);   

		clock_t warp_en = clock();
		if (DEBUG_MSG)
			cout << "warpAffine Running time  is: " << static_cast<double>(warp_en - warp_st) / CLOCKS_PER_SEC * 1000 << "ms" << endl;


		clock_t warp_st5 = clock();

		if (idx == 2)
		{
			warpAffine(imMask1, imMask1t, matrix, WEIGHT_BIGSIZE, INTER_NEAREST);
			warpAffine(imTime1, imTime1t, matrix, WEIGHT_BIGSIZE, INTER_NEAREST);

			clock_t st_mer1 = clock();
			alpha = compute_alpha(imMask1t, imMask2, imTime1t, imTime2, timeRatioThrdd);

			clock_t en_mer1 = clock();
			cout << "compute_alpha Running time  is: " << static_cast<double>(en_mer1 - st_mer1) / CLOCKS_PER_SEC * 1000 << "ms" << endl;

			alpha_1 = Mat::ones(alpha.size(), CV_32FC1);
			alpha_1 = alpha_1 - alpha;
			imMaskS = imMask2.mul(alpha) + imMask1t.mul(alpha_1);
			imTime = max(imTime1t, imTime2);

		}

		mix_image_front(im1t, im2, alpha, alpha_1, ims);


		front_before = front_now;

		im1 = ims;
		imMask1 = imMaskS;
		imTime1 = imTime;

		output = ims.clone();
		if (DEBUG_MSG_IMG)
			imwrite("debug/output.png", output);
#if 0

		double alphaValue = 0.6;
		double betaValue = 1 - alphaValue;
		cout << Highlander.cols << Highlander.rows << endl;

		Mat Car_RoI = output(Rect((output.cols - Highlander.cols) / 2 + matrix.at<double>(1, 2), 140, Highlander.cols, Highlander.rows));

		addWeighted(Car_RoI, alphaValue, Highlander, betaValue, 0., Car_RoI);
		clock_t warp_st6 = clock();
		if (DEBUG_MSG)
			cout << "End Process Running time  is: " << static_cast<double>(warp_st6 - warp_st5) / CLOCKS_PER_SEC * 1000 << "ms" << endl;
#endif
	}

	return output;
}

Mat Panorama::rear_process(Mat front, Mat rear, Mat test_mat)
{
	string imageFileName;
	std::stringstream StrStm;
	StrStm << idx;
	StrStm >> imageFileName;
	imageFileName += ".bmp";

	imwrite("img/" + imageFileName, test_mat);

	if (bypast_cont > 23)
		bypast_cont = 0;
	Mat test_matrix = Mat::zeros(Size(3, 2),CV_64FC1);
	if (!im1.data)
	{
		idx = 1;

		
		expand(front, im1);

//		mergeRearMat(rear, im1);
		mergeRearMat(test_mat, im1);

		imMask1 = weight.clone();
		create_timeImg_from_mask(imMask1, imTime1, 1);

//		cvtColor(rear, rear, CV_BGR2GRAY);
		cvtColor(test_mat, test_mat, CV_BGR2GRAY);
		preImg = im1;

		rear_before = rear;
		test_before = test_mat;
		output = im1;
		return im1;
		//        init_vx();
	}
	else
	{
		idx++;
		clock_t a = clock();
		if (idx == 2)
		{
			imMask2 = weight.clone();

			create_timeImg_from_mask(imMask2, imTime2, idx);

			expand(front, im2);
		}

//		mergeRearMat(rear, im2);
		mergeRearMat(test_mat, im2);
//		cvtColor(rear, rear, CV_RGB2GRAY);
		cvtColor(test_mat, test_mat, CV_BGR2GRAY);

		rear_now = rear;
		test_now = test_mat;

		Mat c_rear_be = Mat::zeros(im1.size(), CV_8UC1);
		Mat c_rear_no = Mat::zeros(im1.size(), CV_8UC1);
//		test_before.copyTo(c_rear_be(Rect((im1.cols - test_before.cols) / 2, (im1.rows - test_before.rows), test_before.cols, test_before.rows)));
//		test_now.copyTo(c_rear_no(Rect((im1.cols - test_before.cols) / 2, (im1.rows - test_before.rows), test_before.cols, test_before.rows)));
		clock_t b = clock();
		if (DEBUG_MSG)
			cout << "Before matrix Running time  is: " << static_cast<double>(b - a) / CLOCKS_PER_SEC * 1000 << "ms" << endl;
		float angle;
		clock_t warp_st1 = clock();
		Mat matrix = LogPolarFFTTemplateMatch(test_now, test_before, angle, 200, 100, idx);
		Mat gad;
//		warpAffine(rear_before, gad, matrix, Size(rear_before.cols + 20, rear_before.rows + 20));
		warpAffine(test_before, gad, matrix, Size(test_before.cols + 20, test_before.rows + 20));
	
		double cos_theta = matrix.at<double>(0, 0);
		double sin_theta = matrix.at<double>(1, 0);

		matrix.at<double>(0, 2) += 35 - 35 * cos_theta + 980 * sin_theta;
		matrix.at<double>(1, 2) += 980 - 35 * sin_theta - 980 * cos_theta;
		//		Mat matrix = match_by_line(test_before, test_now);
		//Mat rot_mat = getRotationMatrix2D(Point(160, 524), angle, 1.0);
		//matrix = matrix + rot_mat;
	
 		clock_t warp_st2 = clock();
		if (DEBUG_MSG)
			cout << "Compute_matrix Running time  is: " << static_cast<double>(warp_st2 - warp_st1) / CLOCKS_PER_SEC * 1000 << "ms" << endl;

		clock_t warp_st3 = clock();


		bool deltaX = abs((int)(matrix.at<double>(0, 2) - matrix_back.at<double>(0, 2))) > 10;
		bool deltaY = abs((int)(matrix.at<double>(1, 2) - matrix_back.at<double>(1, 2))) > 25;
		bool deltaT = abs(matrix.at<double>(0, 1)) > 0.5;


//		if (deltaX || deltaY || deltaT)
		if(0)
		{
			matrix = matrix_back;
		}

		Mat test = matrix;
		matrix_bypast[bypast_cont++] = matrix;
		matrix = Mat(2, 3, CV_64FC1, Scalar(0.0));

		if (idx < 25)
		{
			for (size_t i = 0; i < bypast_cont; i++)
			{

				matrix += (matrix_bypast[i] / (bypast_cont));

			}
		}

		else
		{
			for (size_t i = 0; i < 24; i++)
			{
				matrix += matrix_bypast[i] / 24.0;
			}
		}

		matrix_back = matrix;

		if (1)
		{

			matrix_zero.at<double>(0, 2) = matrix.at<double>(0, 2);
			matrix_zero.at<double>(1, 2) = matrix.at<double>(1, 2);
			if (DEBUG_MSG)
				cout << "*****************" << matrix.at<double>(0, 2) / matrix.at<double>(1, 2) << endl;
//			matrix = matrix_zero;
		}

		clock_t warp_st4 = clock();
		if (DEBUG_MSG)
			cout << "Process_matrix Running time  is: " << static_cast<double>(warp_st4 - warp_st3) / CLOCKS_PER_SEC * 1000 << "ms" << endl;

		if (DEBUG_MSG)
			cout << "+++++++++++++Current speed is++++++++++" << abs(matrix.at<double>(1, 2)*0.25)*3.6 << "Km/h" << endl;
		//test.at<double>(0, 2) -= 6;
        warpAffine(im1, im1t, test, WEIGHT_BIGSIZE, 1);   

		clock_t warp_st5 = clock();

		if (idx == 2)
		{
			warpAffine(imMask1, imMask1t, matrix, WEIGHT_BIGSIZE, INTER_NEAREST);
			warpAffine(imTime1, imTime1t, matrix, WEIGHT_BIGSIZE, INTER_NEAREST);

			clock_t st_mer1 = clock();
			alpha = compute_alpha(imMask1t, imMask2, imTime1t, imTime2, timeRatioThrdd);

			clock_t en_mer1 = clock();
			cout << "compute_alpha Running time  is: " << static_cast<double>(en_mer1 - st_mer1) / CLOCKS_PER_SEC * 1000 << "ms" << endl;

			alpha_1 = Mat::ones(alpha.size(), CV_32FC1);
			alpha_1 = alpha_1 - alpha;
			imMaskS = imMask2.mul(alpha) + imMask1t.mul(alpha_1);
			imTime = max(imTime1t, imTime2);

		}

		mix_image_rear(im1t, im2, matrix.at<double>(1, 2));

		rear_before = rear_now.clone();
		test_before = test_now;
		im1 = im2.clone();
		imMask1 = imMaskS;
		imTime1 = imTime;

		output = im2.clone();
		if (DEBUG_MSG_IMG)
			imwrite("debug/output.png", output);

		clock_t warp_st6 = clock();
		if (DEBUG_MSG)
			cout << "End Process Running time  is: " << static_cast<double>(warp_st6 - warp_st5) / CLOCKS_PER_SEC * 1000 << "ms" << endl;
#if 0

		double alphaValue = 0.6;
		double betaValue = 1 - alphaValue;
		cout << Highlander.cols << Highlander.rows << endl;

		Mat Car_RoI = output(Rect((output.cols - Highlander.cols) / 2 + matrix.at<double>(1, 2), 140, Highlander.cols, Highlander.rows));

		addWeighted(Car_RoI, alphaValue, Highlander, betaValue, 0., Car_RoI);

#endif
	}
	char dx[50];
	sprintf(dx, "dx = %.3f dy= %.3f", test_matrix.at<double>(0, 2), test_matrix.at<double>(1, 2));
	Mat out_test = output.clone();
	cvtColor(out_test, out_test, CV_BGRA2BGR);
	putText(out_test, dx, Point(50, 50), FONT_HERSHEY_COMPLEX, 0.5, Scalar(0, 255, 255), 1, 8, 0);
//	writer1.write(out_test);
	return im1;
}


void Panorama::preProcess(Mat front_mask, Mat rear_mask)
{
	for (int i = 0; i < front_mask.rows; i++)
	{
		for (int j = 0; j < front_mask.cols; j++)
		{
			int val = (255.0 / front_mask.rows) * (i + 1);


			if (front_mask.ptr<uchar>(i)[j])
			{
				front_mask.ptr<uchar>(i)[j] = val;
			}
			if (rear_mask.ptr<uchar>(i)[j])
			{
				rear_mask.ptr<uchar>(i)[j] = val;
			}
		}
	}
	expand(front_mask, weight);

	mergeFrontRearMat(front_mask, rear_mask, weight);

	weight.convertTo(weight, CV_32FC1);
	weight /= 255.0;

}
