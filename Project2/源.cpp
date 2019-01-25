#include "panorama.h"
#include <opencv2/opencv.hpp>
#include "parameter.h"
#include <time.h> 

static Mat Map_Fx = Mat(image_size, CV_32FC1);
static Mat Map_Fy = Mat(image_size, CV_32FC1);
static Mat Map_Rx = Mat(image_size, CV_32FC1);
static Mat Map_Ry = Mat(image_size, CV_32FC1);

static Mat Map_x = Mat(Size(1280, 720), CV_32FC1);
static Mat Map_y = Mat(Size(1280, 720), CV_32FC1);
static Mat warp_mat = Mat(Size(3, 3), CV_64FC1);

static Mat front_image(720, 1280, CV_8UC4);
static Mat rear_image(720, 1280, CV_8UC4);

#define Size_Out_AGRB 640*320*4


static char *ptr = NULL;
static char *pData = NULL;
static char * out_buf = (char *)malloc(sizeof(char) * Size_Out_AGRB);


Mat frontimage, rearimage;
Mat front_mask, rear_mask;
Mat front_mask1, rear_mask1;
Mat frontMat, rearMat;
using namespace std;
using namespace cv;
bool init_ = true;
static Panorama pa;

static Mat front_trs(image_size, CV_8UC4, Scalar::all(0));
static Mat rear_trs(image_size, CV_8UC4, Scalar::all(0));
static Mat out = Mat(Size(space_x, space_y), CV_8UC4, Scalar::all(0));


void GetMapForRemap(Mat matrix[(grid_rows - 1)*(grid_cols - 1)], Mat Map_Fx, Mat Map_Fy)
{

	Mat output;

	for (size_t i = 0; i < (grid_rows - 1)*(grid_cols - 1); i++)
	{
		matrix[i] = matrix[i].inv();
	}

	Point2f po;

	for (size_t p = 0; p < grid_cols - 1; p++)
	{
		for (size_t q = 0; q < grid_rows - 1; q++)
		{
			vector<Point2f> Map_FP;
			vector<Point2f> SRC;
			for (float i = (grid_size)* p; i < (grid_size)*(p + 1); i++)
			{
				for (float j = (grid_size)* q; j < (grid_size)*(q + 1); j++)
				{
					po.x = j;
					po.y = i;
					SRC.push_back(po);
				}
			}
			perspectiveTransform(SRC, Map_FP, matrix[p*(grid_rows - 1) + q]);
			int idpix = 0;
			for (float i = grid_size * p; i < grid_size*(p + 1); i++)
			{
				for (float j = grid_size * q; j < grid_size*(q + 1); j++)
				{
					idpix = (i - grid_size * p) * grid_size + (j - grid_size * q);
					Map_Fx.at<float>(i, j) = Map_FP[idpix].x;
					Map_Fy.at<float>(i, j) = Map_FP[idpix].y;
				}
			}
			Map_FP.clear();
		}
	}
}


Mat Mask[grid_rows * grid_cols];
Mat matrix_affine[grid_rows*grid_cols];
Mat matrix_affine_r[grid_rows*grid_cols];
Mat result[grid_rows*grid_cols];
Mat result_r[grid_rows*grid_cols];

void get_Univariate_matrix(void)
{
#if 1
	Mat img = imread("F.bmp", 0);
	Mat img_r = imread("B.bmp", 0);
	vector<Point2f> corners;
	vector<Point2f> corners_r;
	vector<Point2f> corner_tmp;
	cout << findChessboardCorners(img, CALIBRATOR_BOARD_SIZE, corners) << endl;

	cout << findChessboardCorners(img_r, CALIBRATOR_BOARD_SIZE, corners_r) << endl;
	TermCriteria criteria = TermCriteria(TermCriteria::MAX_ITER + TermCriteria::EPS, 40, 0.1);
	cornerSubPix(img, corners, Size(5, 5), Size(-1, -1), criteria);
	cornerSubPix(img_r, corners_r, Size(5, 5), Size(-1, -1), criteria);
	for (int i = corners_r.size() - 1; i > -1; i--)
	{
		corner_tmp.push_back(corners_r[i]);
	}
	corners_r = corner_tmp;
#else

	ifstream myfile("F1F.txt");
	ifstream myfile_R("B1B.txt");
	string temp;
	string temp_R;
	if (!myfile.is_open())
	{
		cout << "未成功打开文件" << endl;
	}
	if (!myfile_R.is_open())
	{
		cout << "未成功打开文件" << endl;
	}
	vector<Point2f> corners;
	vector<Point2f> corners_r;
	int index = 0;
	while (getline(myfile, temp))
	{
		Point2f p1;
		int a[2] = { 0 };
		istringstream iss;//istringstream提供读 string 的功能
		iss.str(temp);//将 string 类型的 test 复制给 iss，返回 void
		string s;
		int i = 0;
		while (iss >> s)
		{
			int x = stoi(s);
			a[i] = x;
			i++;
		}
		p1.x = a[0];
		p1.y = a[1];
		corners.push_back(p1);
		index++;
	}

	int index_r = 0;
	while (getline(myfile_R, temp_R))
	{
		Point2f p1;
		int a[2] = { 0 };
		istringstream iss;//istringstream提供读 string 的功能
		iss.str(temp_R);//将 string 类型的 test 复制给 iss，返回 void
		string s;
		int i = 0;
		while (iss >> s)
		{
			int x = stoi(s);
			a[i] = x;
			i++;
		}
		p1.x = a[0];
		p1.y = a[1];
		corners_r.push_back(p1);
		index_r++;
	}
	cout << corners_r << endl;
#endif		
	for (int i = 0; i < 9; i++)
	{
		Mat mode = Mat::zeros(dstImg_cols, dstImg_rows, CV_8UC1);
		Mat mode1 = Mat::zeros(dstImg_cols, dstImg_rows, CV_8UC1);
		Mat mode2 = Mat::zeros(dstImg_cols, dstImg_rows, CV_8UC1);
		Mat mask_mini = Mat::ones(grid_size, grid_size, CV_8UC1);
		Mat ROI;
		ROI = mode(Rect(i*grid_size, 0, grid_size, grid_size));
		mask_mini.copyTo(ROI);
		Mask[i] = mode;
		Mat ROI1;
		ROI1 = mode1(Rect(i * grid_size, grid_size, grid_size, grid_size));
		mask_mini.copyTo(ROI1);
		Mask[i + grid_rows] = mode1;
		Mat ROI2;
		ROI2 = mode2(Rect(i * grid_size, grid_size * 2, grid_size, grid_size));
		mask_mini.copyTo(ROI2);
		Mask[i + grid_rows * 2] = mode2;
	}

	Point2f Src[grid_cols*grid_rows], Dst[grid_cols*grid_rows], Src_r[grid_cols*grid_rows];
	for (int i = 0; i < grid_cols*grid_rows; i++)
	{
		Src[i] = corners[i];
		Src_r[i] = corners_r[i];
	}
	for (int i = 0; i < grid_rows; i++)
	{
		if (i == 0)
		{
			for (int j = 0; j < grid_cols; j++)
			{
				Dst[i + grid_rows * j].x = 0;
			}
		}
		else
		{
			for (int j = 0; j < grid_cols; j++)
			{
				Dst[i + grid_rows * j].x = i * grid_size - 1;
			}
		}
		Dst[i].y = 0;
		for (int j = 1; j < grid_cols; j++)
		{
			Dst[i + grid_rows * j].y = grid_size * j - 1;;
		}
	}
	vector<Point2f> Dsst, Test;
	for (int i = 0; i < grid_cols*grid_rows; i++)
	{
		Dsst.push_back(Dst[i]);
	}
	for (int i = 0; i < grid_rows - 1; i++)
	{
		Point2f m[4], n[4], m_r[4];
		for (int j = 0; j < grid_cols - 1; j++)
		{
			m[0] = corners[grid_rows * j + i];
			m[1] = corners[grid_rows * j + i + 1];
			m[2] = corners[grid_rows * (j + 1) + i];
			m[3] = corners[grid_rows * (j + 1) + 1 + i];
			m_r[0] = corners_r[grid_rows * j + i];
			m_r[1] = corners_r[grid_rows * j + i + 1];
			m_r[2] = corners_r[grid_rows * (j + 1) + i];
			m_r[3] = corners_r[grid_rows * (j + 1) + 1 + i];
			n[0] = Dst[grid_rows * j + i];
			n[1] = Dst[grid_rows * j + i + 1];
			n[2] = Dst[grid_rows * (j + 1) + i];
			n[3] = Dst[grid_rows * (j + 1) + 1 + i];
			matrix_affine_r[i + (grid_rows - 1) * j] = getPerspectiveTransform(m_r, n);
			matrix_affine[i + (grid_rows - 1) * j] = getPerspectiveTransform(m, n);
		}

	}

}




int main(int argc, char **argv)
{
	Mat dst = Mat::zeros(480, 960, CV_8UC4);
	Mat output(480, 905, CV_8UC4);
	Mat show_img;
	show_img.create(480, 960, CV_8UC4);
	unsigned int *pbuff = NULL;
	unsigned int *ptr = NULL;
	Mat img;
	unsigned int i, x, d;
	unsigned int * p;
	img.create(720, 1280, CV_8UC4);


	ifstream input("Mapx.txt", ios::in | ios::binary);
	if (!input)
	{
		cerr << "Open input file error!" << endl;
		exit(-1);
	}
	input.read((char *)Map_x.data, sizeof(float) * 1280 * 720);


	ifstream input1("Mapy.txt", ios::in | ios::binary);
	if (!input1)
	{
		cerr << "Open input file error!" << endl;
		exit(-1);
	}
	input1.read((char *)Map_y.data, sizeof(float) * 1280 * 720);

	ifstream input2("warp_mat.txt", ios::in | ios::binary);
	if (!input2)
	{
		cerr << "Open input file error!" << endl;
		exit(-1);
	}
	input2.read((char *)warp_mat.data, sizeof(double) * 3 * 3);


	if (init_ = true)
	{
		cout << "*********start**********" << endl;
		get_Univariate_matrix();
		cout << "*********start**********" << endl;
		GetMapForRemap(matrix_affine, Map_Fx, Map_Fy);
		GetMapForRemap(matrix_affine_r, Map_Rx, Map_Ry);

		Mat front_chess = imread("F.bmp");
		Mat rear_chess = imread("uB.jpg");

		remap(front_chess, front_chess, Map_Fx, Map_Fy, INTER_LINEAR, BORDER_CONSTANT);
		remap(rear_chess, rear_chess, Map_Rx, Map_Ry, INTER_LINEAR, BORDER_CONSTANT);
		if (1)
		{
			cout << "*#*#*#*#*#*#*#*#*#*#**##*" << endl;
			imwrite("debug/F_chess.jpg", front_chess);
			imwrite("debug/B_chess.jpg", rear_chess);
		}

		pa.compute_merge_matrix(frontMat, rearMat, CALIBRATOR_BOARD_SIZE, offsize_xx, offsize_yy);

		front_mask1 = Mat::ones(image_size, CV_8UC1);
		imwrite("result/融合参数/front_mask1.bmp", front_mask1);

		rear_mask1 = Mat::ones(image_size, CV_8UC1);
		imwrite("result/融合参数/rear_mask1.bmp", rear_mask1);

		pa.preProcess(front_mask1, rear_mask1);

		init_ = false;
	}


	cout << "步骤4, 开始处理图像序列..." << endl;
	int idx0 = 0; //FR

	VideoCapture front_cap("3_B2.mkv");   //1_F.mkv    5F.avi
	VideoCapture rear_cap("3_B2.mkv");   //1_B.mkv     5B.avi


	/*-----------------------------------------------------------------------------------------------------------------------------------------------------*/
	/*------------------------------------------------------------------------------------------------------------------------------------------------------*/
	clock_t total_start = clock();
	if (front_cap.isOpened() && rear_cap.isOpened())
	{

		front_cap >> frontimage;
		rear_cap >> rearimage;

		Mat front_trs(front_image.size(), CV_8UC4, Scalar::all(0));
		Mat rear_trs(rear_image.size(), CV_8UC4, Scalar::all(0));

		while (frontimage.data && rearimage.data)
		{
			clock_t tc3;
			cout << "image " << idx0++ << endl;


			if (idx0 > 3900 && (idx0%1 ==0))
			{

				{
					cvtColor(frontimage, front_image, COLOR_BGR2BGRA);
					cvtColor(rearimage, rear_image, COLOR_BGR2BGRA);

					Mat test_mat;
					remap(rear_image, test_mat, Map_x, Map_y, 1);
					Mat input_cali;
					warpPerspective(test_mat, input_cali, warp_mat, Size(19 * 30, 20 * 15), 1);

					Mat bf;
//					bilateralFilter(rearimage, bf, 25, 30, 15);
					rearimage = bf;
					string imageFileName = to_string(idx0);
					imageFileName += ".bmp";
					cv::Mat image = imread("bmp/" + imageFileName);
//					cvtColor(image, image, CV_BGR2BGRA);

					clock_t FT_st = clock();

					output = av_merge_image(front_image, rear_image, 1, input_cali);
					imshow("out", output);
					waitKey(10);
				

				}
			}
			front_cap >> frontimage;
			rear_cap >> rearimage;

		}
		clock_t total_end = clock();
		front_cap.release();
		rear_cap.release();

	}
	return 0;
}

int isColor = 1;
int fps = 30;
int frameWidth = 320;
int frameHeight = 640;
VideoWriter writer("avmerge.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), fps, Size(frameWidth, frameHeight), isColor);
Mat back_trs = Mat::zeros(Size(260, 180), CV_8UC4);
Mat av_merge(Mat front_image, Mat rear_image, bool Reversing, Mat test_mat)
{
	Mat out;
	if (!Reversing)
	{
		clock_t end_remap = clock();
		remap(front_image, front_trs, Map_Fx, Map_Fy, INTER_CUBIC, BORDER_CONSTANT);
		if (front_trs.size() != image_size)
		{
			if (DEBUG_MSG)
				cout << "#################resize####################" << endl;
			resize(front_trs, front_trs, image_size);
			resize(rear_trs, rear_trs, image_size);
		}

		clock_t end_process = clock();

		if (DEBUG_MSG)
			cout << "###############################front process Running time  is: " << static_cast<double>(end_process - end_remap) / CLOCKS_PER_SEC * 1000 << "ms#####################" << endl;
		out = pa.front_process(front_trs, rear_trs);
	}
	else
	{
		clock_t end_remap = clock();
		remap(rear_image, rear_trs, Map_Rx, Map_Ry, INTER_CUBIC, BORDER_CONSTANT);


		if (front_trs.size() != image_size)
		{
			if (DEBUG_MSG)
				cout << "#################resize####################" << endl;
			resize(front_trs, front_trs, image_size);
			resize(rear_trs, rear_trs, image_size);
		}


		clock_t end_process = clock();
		if (DEBUG_MSG)
			cout << "###############################rear process Running time  is: " << static_cast<double>(end_process - end_remap) / CLOCKS_PER_SEC * 1000 << "ms#####################" << endl;
		out = pa.rear_process(front_trs, rear_trs, test_mat);
		cvtColor(out, out, COLOR_BGRA2BGR);
//		writer.write(out);
	}
	return out;
}


Mat av_merge_image(Mat front_buf, Mat rear_buf, bool Reversing, Mat test_mat)
{
	if (init_ == true)
	{
		cout << "*********start**********" << endl;
		get_Univariate_matrix();
		cout << "*********start**********" << endl;
		GetMapForRemap(matrix_affine, Map_Fx, Map_Fy);
		GetMapForRemap(matrix_affine_r, Map_Rx, Map_Ry);

		pa.compute_merge_matrix(frontMat, rearMat, CALIBRATOR_BOARD_SIZE, offsize_xx, offsize_yy);

		Mat front_chess = imread("F.bmp");
		Mat rear_chess = imread("B.bmp");

		remap(front_chess, front_chess, Map_Fx, Map_Fy, INTER_LINEAR, BORDER_CONSTANT);
		remap(rear_chess, rear_chess, Map_Rx, Map_Ry, INTER_LINEAR, BORDER_CONSTANT);
		if (1)
		{
			cout << "*#*#*#*#*#*#*#*#*#*#**##*" << endl;
			imwrite("debug/F_chess.jpg", front_chess);
			imwrite("debug/B_chess.jpg", rear_chess);
		}
		front_mask1 = Mat::ones(image_size, CV_8UC1);

		rear_mask1 = Mat::ones(image_size, CV_8UC1);

		pa.preProcess(front_mask1, rear_mask1);
		if (DEBUG_MSG)
			cout << "##################end Init_parameter###########" << endl;
		init_ = false;
	}

	clock_t st_b = clock();
	if (!Reversing)
	{
		//        front_image.data = (unsigned char *)front_buf;
		if (DEBUG_MSG_IMG)
			imwrite("front_input.png", front_image);
		clock_t en_b = clock();

		cout << "###############################bef Running time  is: " << static_cast<double>(en_b - st_b) / CLOCKS_PER_SEC * 1000 << "ms#####################" << endl;

		out = av_merge(front_image, rear_image, Reversing, test_mat);
	}
	else
	{
		//        rear_image.data = (unsigned char *)rear_buf;

		if (DEBUG_MSG_IMG)
			imwrite("rear_input.png", rear_image);

		clock_t en_c = clock();

		cout << "###############################bef Running time  is: " << static_cast<double>(en_c - st_b) / CLOCKS_PER_SEC * 1000 << "ms#####################" << endl;
		out = av_merge(front_image, rear_image, Reversing, test_mat);
	}

	return out;
}



