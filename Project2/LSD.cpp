#include "LSD.hpp"

Mat match_by_line(Mat inputa, Mat inputb)
{
	static float angle_back;
	bool useRefine = true;
	bool useCanny = false;
	Mat image = inputa.clone();
//	image = image(Rect(2, 0, 256, 128)).clone();
	Mat test = image.clone();
	Ptr<LineSegmentDetector> ls = useRefine ? createLineSegmentDetector(LSD_REFINE_STD) : createLineSegmentDetector(LSD_REFINE_NONE);

	vector<Vec4f> lines_std;
	ls->detect(image, lines_std);
	int max_cnt = 0;
	int max_length = 0;
	float max_theta = 0;
	vector<Vec4f> max_line;
	Point2f up_p;
	Point2f left_p;
	Point2f right_p;
	Point2f down_p;
	Vec4f multi_line;
	vector<Point2f> st;
	bool a = false;
	bool b = false;
	bool c = false;
	bool d = false;
	if (lines_std.size() == 0)
	{
		max_theta = 0;
	}
	else
	{
		for (int i = 0; i < lines_std.size(); i++)
		{
			int line_len = sqrt((lines_std[i](0) - lines_std[i](2))*(lines_std[i](0) - lines_std[i](2)) + (lines_std[i](1) - lines_std[i](3))*(lines_std[i](1) - lines_std[i](3)));
			if (line_len > max_length)
			{
				max_line.clear();
				max_length = line_len;
				max_cnt = i;

				left_p.x = 0;
				left_p.y = (-lines_std[i](0)) / (lines_std[i](2) - lines_std[i](0)) * (lines_std[i](3) - lines_std[i](1)) + lines_std[i](1);
				right_p.x = image.cols - 1;
				right_p.y = (right_p.x - lines_std[i](0)) / (lines_std[i](2) - lines_std[i](0)) * (lines_std[i](3) - lines_std[i](1)) + lines_std[i](1);
				up_p.y = 0;
				up_p.x = (-lines_std[i](1)) / (lines_std[i](3) - lines_std[i](1)) * (lines_std[i](2) - lines_std[i](0)) + lines_std[i](0);
				down_p.y = image.rows - 1;
				down_p.x = (down_p.y - lines_std[i](1)) / (lines_std[i](3) - lines_std[i](1)) * (lines_std[i](2) - lines_std[i](0)) + lines_std[i](0);
				st.clear();
				if ((0 <= left_p.y) && (left_p.y <= image.rows - 1))
				{
					c = true;
					Point2f p;
					p.x = 0;
					p.y = left_p.y;
					st.push_back(p);
				}
				if ((0 <= right_p.y) && (right_p.y <= image.rows - 1))
				{
					d = true;
					Point2f p;
					p.x = image.cols - 1;
					p.y = right_p.y;
					st.push_back(p);
				}
				if ((0 <= up_p.x) && (up_p.x <= image.cols - 1))
				{
					a = true;
					Point2f p;
					p.y = 0;
					p.x = up_p.x;
					st.push_back(p);
				}
				if ((0 <= down_p.x) && (down_p.x <= image.cols - 1))
				{
					b = true;
					Point2f p;
					p.y = image.rows - 1;
					p.x = down_p.x;
					st.push_back(p);
				}
				multi_line(0) = st[0].x;
				multi_line(1) = st[0].y;
				multi_line(2) = st[1].x;
				multi_line(3) = st[1].y;
				max_theta = atan(-((st[1].y - st[0].y) / (st[1].x - st[0].x)))*180.0 / CV_PI;
				max_line.push_back(multi_line);
			}
		}
	}


	Mat image1 = inputb.clone();
//	image1 = image1(Rect(2, 0, 256, 128));

	vector<Vec4f> lines_std1;

	ls->detect(image1, lines_std1);

	int max_cnt1 = 0;
	int max_length1 = 0;
	float max_theta1 = 0;
	Point2f match_down_p;
	Point2f match_up_p;
	Point2f match_left_p;
	Point2f match_right_p;
	vector<Vec4f> max_line1;
	Vec4f match_multi_line;
	vector<Point2f> en;
	int cnt_match = 0;
	for (int i = 0; i < lines_std1.size(); i++)
	{
		int line_len = sqrt((lines_std1[i](0) - lines_std1[i](2))*(lines_std1[i](0) - lines_std1[i](2)) + (lines_std1[i](1) - lines_std1[i](3))*(lines_std1[i](1) - lines_std1[i](3)));
		float theta = atan(-((lines_std1[i](3) - lines_std1[i](1)) / (lines_std1[i](2) - lines_std1[i](0))))*180.0 / CV_PI;
		if (abs(max_theta - theta) < ALG)
		{
			max_length1 = line_len;
			max_cnt1 = i;
			//			max_line1.push_back(lines_std1[i]);
			match_left_p.x = 0;
			match_left_p.y = (-lines_std1[i](0)) / (lines_std1[i](2) - lines_std1[i](0)) * (lines_std1[i](3) - lines_std1[i](1)) + lines_std1[i](1);
			match_right_p.x = image.cols - 1;
			match_right_p.y = (right_p.x - lines_std1[i](0)) / (lines_std1[i](2) - lines_std1[i](0)) * (lines_std1[i](3) - lines_std1[i](1)) + lines_std1[i](1);
			match_up_p.y = 0;
			match_up_p.x = (-lines_std1[i](1)) / (lines_std1[i](3) - lines_std1[i](1)) * (lines_std1[i](2) - lines_std1[i](0)) + lines_std1[i](0);
			match_down_p.y = image.rows - 1;
			match_down_p.x = (down_p.y - lines_std1[i](1)) / (lines_std1[i](3) - lines_std1[i](1)) * (lines_std1[i](2) - lines_std1[i](0)) + lines_std1[i](0);
			en.clear();
			if (a && (abs(match_up_p.x - up_p.x) < DIS) && (0 <= match_up_p.x) && (match_up_p.x <= image.rows - 1))
			{
				Point2f p;
				p.y = 0;
				p.x = match_up_p.x;
				en.push_back(p);
			}
			if (b && (abs(match_down_p.x - down_p.x) < DIS) && (0 <= match_down_p.x) && (match_down_p.x <= image.cols - 1))
			{
				Point2f p;
				p.y = image1.rows - 1;
				p.x = match_down_p.x;
				en.push_back(p);
			}
			if (c && (abs(match_left_p.y - left_p.y) < DIS) && (0 <= match_left_p.y) && (match_left_p.y <= image.rows - 1))
			{
				Point2f p;
				p.x = 0;
				p.y = match_left_p.y;
				en.push_back(p);
			}
			if (d && (abs(match_right_p.y - right_p.y) < DIS) && (0 <= match_right_p.y) && (match_right_p.y <= image.rows - 1))
			{
				Point2f p;
				p.x = image1.cols - 1;
				p.y = match_right_p.y;
				en.push_back(p);
			}
			if (en.size() == 2)
			{
				cnt_match++;
				match_multi_line(0) = en[0].x;
				match_multi_line(1) = en[0].y;
				match_multi_line(2) = en[1].x;
				match_multi_line(3) = en[1].y;
				max_theta1 = atan(-((en[1].y - en[0].y) / (en[1].x - en[0].x)))*180.0 / CV_PI;
				max_line1.push_back(match_multi_line);
			}
		}

	}
	float angle = max_theta1 - max_theta;
	if (!cnt_match)
	{
		angle = angle_back;
		max_theta1 = max_theta;
	}


	Mat m = Mat::zeros(image.size(), CV_8UC3);
	Mat n = Mat::zeros(image.size(), CV_8UC3);
	ls->drawSegments(m, max_line);
	ls->drawSegments(n, max_line1);
//	angle = -1.0;
	Mat matrix = getRotationMatrix2D(Point(320, 160), angle, 1);
	angle_back = angle;
	warpAffine(image, image, matrix, image.size(), 1);
	image.convertTo(image, CV_32FC1);
	image1.convertTo(image1, CV_32FC1);
	Mat matrix_1 = Mat::zeros(Size(3, 3), CV_64FC1);
	memcpy(matrix_1.data, matrix.data, sizeof(double) * 6);
	matrix_1.at<double>(2, 2) = 1.0;

	Point2f tr = phaseCorrelate(image, image1);
	Mat matrix_t = Mat::zeros(Size(3, 3), CV_64FC1);
	matrix_t.at<double>(0, 0) = 1.0;
	matrix_t.at<double>(0, 2) = -tr.x;
	matrix_t.at<double>(1, 1) = 1.0;
	matrix_t.at<double>(1, 2) = tr.y;
	matrix_t.at<double>(2, 2) = 1.0;
	Mat matrix_x = Mat::zeros(Size(3, 2), CV_64FC1);
	Mat matrix_y = Mat::zeros(Size(3, 2), CV_64FC1);
	Mat matrix_all = matrix_1 * matrix_t;
	memcpy(matrix_x.data, matrix_all.data, sizeof(double) * 6);
	memcpy(matrix_y.data, matrix_t.data, sizeof(double) * 6);
	Mat matrix_m = matrix.clone();
	matrix_m.at<double>(0, 2) = matrix_m.at<double>(0, 2) + tr.x;
	matrix_m.at<double>(1, 2) = matrix_m.at<double>(1, 2) + tr.y;

	Mat test1;
	warpAffine(image, image, matrix_y, image.size(), 1);
	warpAffine(test, test1, matrix_x, image.size(), 1);

	return matrix_x;
}