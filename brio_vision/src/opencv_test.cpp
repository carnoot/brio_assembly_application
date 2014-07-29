#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

void setLabel(cv::Mat& im, const std::string label,
		std::vector<cv::Point>& contour) {
	int fontface = cv::FONT_HERSHEY_SIMPLEX;
	double scale = 0.4;
	int thickness = 1;
	int baseline = 0;

	cv::Size text = cv::getTextSize(label, fontface, scale, thickness,
			&baseline);
	cv::Rect r = cv::boundingRect(contour);

	cv::Point pt(r.x + ((r.width - text.width) / 2),
			r.y + ((r.height + text.height) / 2));
	cv::rectangle(im, pt + cv::Point(0, baseline),
			pt + cv::Point(text.width, -text.height), CV_RGB(255, 255, 255),
			CV_FILLED);
	cv::putText(im, label, pt, fontface, scale, CV_RGB(0, 0, 0), thickness, 8);
}

vector<Point> tableDetect(cv::Mat imgRGB) {

	CvSeq* result;

	int iLowH = 0;
	int iHighH = 100;

	int iLowS = 0;
	int iHighS = 100;

	int iLowV = 0;
	int iHighV = 100;

	cv::Mat imgHSV, aux;


	imgRGB.copyTo(aux);

	cv::cvtColor(imgRGB, imgHSV, cv::COLOR_BGR2GRAY);
	GaussianBlur(imgHSV, imgHSV, Size(9, 9), 2, 2);

	cv::Mat imgThresholded;
	cv::inRange(imgHSV, cv::Scalar(iLowH, iLowS, iLowV),
			cv::Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

	//morphological opening (remove small objects from the foreground)
	cv::erode(imgThresholded, imgThresholded,
			cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
	cv::dilate(imgThresholded, imgThresholded,
			cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

	cv::dilate(imgThresholded, imgThresholded,
			cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
	cv::erode(imgThresholded, imgThresholded,
			getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

	//Calculate the moments of the thresholded image
	cv::Moments oMoments = cv::moments(imgThresholded);
	double dM01 = oMoments.m01;
	double dM10 = oMoments.m10;
	double dArea = oMoments.m00;

	int thresh = 100;
	cv::Mat canny_output;
	cv::vector<cv::vector<cv::Point> > contours;
	cv::vector<cv::Vec4i> hierarchy;

	/// Detect edges using canny
	cv::Canny(imgThresholded, canny_output, thresh, thresh * 2, 3);
	findContours(imgThresholded, contours, hierarchy, CV_RETR_TREE,
			CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

	cv::Mat drawing = cv::Mat::zeros(imgThresholded.size(), CV_8UC1);
	vector<Point> approx;

	cv::Mat contTest = cv::Mat::zeros(canny_output.size(), CV_8UC3);


	int i;
	for ( i = 0; i < contours.size(); i++) {
	cv::drawContours(contTest, contours, i, 100, 2, 8, hierarchy, 0, cv::Point());
	}

	imshow("Cont test",contTest);

	for (int i = 0; i < contours.size(); i++) {
		if (contourArea(contours[i]) > 30000) {
			cv::drawContours(drawing, contours, i, 100, 2, 8, hierarchy, 0,
					cv::Point());

			approxPolyDP(Mat(contours[i]), approx,
					arcLength(Mat(contours[i]) * 0.02, true), true);

		}
	}

	return approx;
}

std::vector<int> linePoints(int x1, int y1, int x2, int y2) {
	std::vector<int> linePoints;
	double slope;
	int y;
	int x;
	int b;

	//get all the points
	if (x1 == x2 && y1 == y2) {
		linePoints.push_back(x1);
		linePoints.push_back(y1);
		linePoints.push_back(x2);
		linePoints.push_back(y2);
	}

	if (x1 > x2 && y1 > y2) {
		for (x = x1; x >= x2; x--) {
			for (y = y1; y >= y2; y--) {
				slope = (y2 - y1) / (x2 - x1);
				b = y1 - slope * x1;
//				std::cout << "X: " << x << std::endl;
//				std::cout << "slope * x + b " << slope * x + b << std::endl;
//				std::cout << "Slopeeee: " << slope << std::endl;
//				std::cout << "Y: " << y << std::endl;
				if (y == round(slope * x + b)) {
					linePoints.push_back(x);
					linePoints.push_back(y);
				}

			}

		}
	}
	if (x1 > x2 && y1 < y2) {
		for (x = x1; x >= x2; x--) {
			for (y = y1; y <= y2; y++) {
				slope = (y2 - y1) / (x2 - x1);
				b = y1 - slope * x1;
				if (y == round(slope * x + b)) {
					linePoints.push_back(x);
					linePoints.push_back(y);
				}

			}

		}
	}

	if (x1 < x2 && y1 < y2) {
		for (x = x1; x <= x2; x++) {
			for (y = y1; y <= y2; y++) {
				slope = (y2 - y1) / (x2 - x1);
				b = y1 - slope * x1;
				if (y == round(slope * x + b)) {
					linePoints.push_back(x);
					linePoints.push_back(y);
				}

			}

		}
	}

	if (x1 < x2 && y1 > y2) {
		for (x = x1; x <= x2; x++) {
			for (y = y1; y >= y2; y--) {
				slope = (y2 - y1) / (x2 - x1);
				b = y1 - slope * x1;
				if (y == round(slope * x + b)) {
					linePoints.push_back(x);
					linePoints.push_back(y);
				}

			}

		}
	}

	if (x1 == x2 && y1 > y2) {
		x = x1;
		for (y = y1; y >= y2; y--) {
			slope = (y2 - y1) / (x2 - x1);
			b = y1 - slope * x1;
			if (y == round(slope * x + b)) {
				linePoints.push_back(x);
				linePoints.push_back(y);
			}

		}

	}

	if (x1 == x2 && y1 < y2) {
		x = x1;
		for (y = y1; y <= y2; y++) {
			slope = (y2 - y1) / (x2 - x1);
			b = y1 - slope * x1;
			if (y == round(slope * x + b)) {
				linePoints.push_back(x);
				linePoints.push_back(y);
			}

		}

	}

	if (x1 < x2 && y1 == y2) {
		y = y1;
		for (x = x1; x <= x2; x++) {
			slope = (y2 - y1) / (x2 - x1);
			b = y1 - slope * x1;
			if (y == round(slope * x + b)) {
				linePoints.push_back(x);
				linePoints.push_back(y);
			}

		}

	}

	if (x1 > x2 && y1 == y2) {
		y = y1;
		for (x = x1; x >= x2; x--) {
			slope = (y2 - y1) / (x2 - x1);
			b = y1 - slope * x1;
			if (y == round(slope * x + b)) {
				linePoints.push_back(x);
				linePoints.push_back(y);
			}

		}

	}

	return linePoints;
}

bool IsInBlackAreaLeft(Point pt, vector<int> lineLeft) {
	bool ok;
	vector<int>::iterator i = lineLeft.begin();
	int nPosition;
//  cout<< "I search ->>>>>>>>>  "<< pt.y<<endl;;
//	for(int x=0;x<lineLeft.size();x++)
//	{
//		cout<<lineLeft[x]<< " ";
//	}
//	cout<<endl;

	i = find(lineLeft.begin(), lineLeft.end(), pt.y);

	if (i != lineLeft.end()) {
		nPosition = std::distance(lineLeft.begin(), i);

	} else {
		ok = false;
	}

	if (lineLeft[nPosition - 1] > pt.x) {
		ok = true;
	} else {
		ok = false;
	}

	return ok;
}

bool IsInBlackAreaDown(Point pt, vector<int> lineDown) {
	bool ok = false;
	vector<int>::iterator i = lineDown.begin();
	int nPosition;

//	  cout<< "I search ->>>>>>>>>  "<< pt.x<<endl;;
//		for(int x=0;x<lineDown.size();x++)
//		{
//			cout<<lineDown[x]<< " ";
//		}
//		cout<<endl;

	i = find(lineDown.begin(), lineDown.end(), pt.x);

	if (i != lineDown.end()) {
		nPosition = std::distance(lineDown.begin(), i);
	} else {
		ok = false;
	}

	if (lineDown[nPosition + 1] < pt.y) {
		ok = true;
	} else {
		ok = false;
	}

	return ok;
}

bool IsInBlackAreaRight(Point pct, vector<int> line) {
	bool ok = false;
	vector<int>::iterator i = line.begin();
	int nPosition;
	i = find(line.begin(), line.end(), pct.y);
	if (i != line.end()) {
		nPosition = std::distance(line.begin(), i);
	} else {
		ok = false;
	}

	if (line[nPosition - 1] < pct.x) {
		ok = true;

	} else {
		ok = false;

	}

	return ok;

}

bool IsInBlackAreaTop(Point pct, vector<int> line) {

	bool ok = false;
	vector<int>::iterator i = line.begin();
	int nPosition;

	i = find(line.begin(), line.end(), pct.x);

	if (i != line.end()) {
		nPosition = std::distance(line.begin(), i);
	} else {
		ok = false;
	}

	if (line[nPosition + 1] > pct.y) {
		ok = true;

	} else {
		ok = false;

	}
	return ok;
}

Point maxY(vector<Point> contour) {
	Point max;
	max = contour[0];

	for (int i = 0; i < contour.size(); i++) {
		if (max.y < contour[i + 1].y) {
			max = contour[i + 1];
		}
	}

	return max;

}

Point minY(vector<Point> contour) {
	Point min;
	min = contour[0];

	for (int i = 0; i < contour.size(); i++) {
		if (min.y > contour[i + 1].y) {
			min = contour[i + 1];
		}
	}

	return min;

}

Point maxX(vector<Point> contour) {
	Point max;
	max = contour[0];

	for (int i = 0; i < contour.size(); i++) {
		if (max.x < contour[i + 1].x) {
			max = contour[i + 1];
		}
	}
	return max;

}

Point minX(vector<Point> contour) {
	Point min;
	min = contour[0];

	for (int i = 0; i < contour.size(); i++) {
		if (min.x > contour[i + 1].x) {
			min = contour[i + 1];

		}
	}

	return min;

}

int NormaliseArea(cv::vector<cv::Point>  contours) {
	int area;
	Point2f massCenter;
	Moments shape;



		area = round(contourArea(contours));

//		cout<< "AREEA "<< area<<endl;

		shape= moments(contours, false);

//		cout<<" ********Moments.M00 **** "<<shape.m00<< endl;
//		cout<<" ********Moments.M01 **** "<<shape.m01<< endl;
//		cout<<" ********Moments.M10 **** "<<shape.m10<< endl;
		massCenter= Point2f(shape.m10 / shape.m00,
				shape.m01 / shape.m00);

//		cout<< "Mass Center    "<< massCenter <<endl;

		if (massCenter.x < 620) {
			area = area + (massCenter.x - 620);
		} else {
			area = area - (massCenter.x - 620);
		}

		if (massCenter.y < 540) {
			area = area + ((massCenter.y - 540) * 3);
		} else {
			area = area - ((massCenter.y - 540) * 3);
		}

		cout<< "  /////////////////////////////////// "<< endl;

	return area;
}

void cloud_cb(const sensor_msgs::Image::Ptr input) {

	cv_bridge::CvImageConstPtr pCvColor = cv_bridge::toCvShare(input,
			sensor_msgs::image_encodings::BGR8);

	cv::Mat imgRGB;
	pCvColor->image.copyTo(imgRGB);

	//OpenCV algorithm
	//Vector that contains the black zone corners
	int originX;
	int originY;
	int heigth;
	int width;
	int offsetX;
	int offsetY;

	vector<Point> limits;
	limits = tableDetect(imgRGB);

	if (limits[1].y > limits[2].y) {
		heigth = limits[2].y;
	} else {
		heigth = limits[1].y;
	}

	if (limits[0].y > limits[3].y) {
		originY = limits[0].y;
		offsetY = limits[0].y;
	} else {
		originY = limits[3].y;
		offsetY = limits[3].y;
	}

	if (limits[0].x > limits[1].x) {
		originX = limits[0].x;
		offsetX = limits[0].x;
	} else {
		originX = limits[1].x;
		offsetX = limits[1].x;
	}

	if (limits[2].x > limits[3].x) {
		width = limits[3].x;
	} else {
		width = limits[2].x;
	}

//	cout << "Image dimensions :::" << width - offsetX << " x "
//			<< heigth - offsetY;

	cv::Rect myROI(offsetX, offsetY, width - offsetX, heigth - offsetY);
	imgRGB = imgRGB(myROI);

	imshow("Image", imgRGB);

	//find the up, left, right and down limit of black zone by corners

	std::vector<int> upLine;
	std::vector<int> leftLine;
	std::vector<int> downLine;
	std::vector<int> rightLine;

	upLine = linePoints(limits[0].x, limits[0].y, limits[3].x, limits[3].y);
	leftLine = linePoints(limits[0].x, limits[0].y, limits[1].x, limits[1].y);
	downLine = linePoints(limits[1].x, limits[1].y, limits[2].x, limits[2].y);
	rightLine = linePoints(limits[3].x, limits[3].y, limits[2].x, limits[2].y);


	int iLowH = 100;
	int iHighH = 255;

	int iLowS = 100;
	int iHighS = 255;

	int iLowV = 100;
	int iHighV = 255;

	cv::Mat realImg;
	pCvColor->image.copyTo(realImg);

	cv::Mat imgHSV, aux;

	cv::cvtColor(imgRGB, imgHSV, cv::COLOR_BGR2GRAY);
	GaussianBlur(imgHSV, imgHSV, Size(9, 9), 2, 2);

	cv::Mat imgThresholded;
	cv::inRange(imgHSV, cv::Scalar(iLowH, iLowS, iLowV),
			cv::Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

	//morphological opening (remove small objects from the foreground)
	cv::erode(imgThresholded, imgThresholded,
			cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
	cv::dilate(imgThresholded, imgThresholded,
			cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

	//morphological closing (fill small holes in the foreground)
	cv::dilate(imgThresholded, imgThresholded,
			cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
	cv::erode(imgThresholded, imgThresholded,
			getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

	cv::Mat imgLines = cv::Mat::zeros(imgRGB.size(), CV_8UC3);

	//Calculate the moments of the thresholded image
	cv::Moments oMoments = cv::moments(imgThresholded);

	double dM01 = oMoments.m01;
	double dM10 = oMoments.m10;
	double dArea = oMoments.m00;

	cv::imshow("Objects", imgThresholded);

///////////////////////////////GET CONTUR//////////////////////////////////////
	int thresh = 100;
	cv::Mat canny_output;
	cv::vector<cv::vector<cv::Point> > contourblack;
	cv::vector<cv::vector<cv::Point> > contours;
	cv::vector<cv::Vec4i> hierarchy;
	cv::Canny(imgThresholded, canny_output, thresh, thresh * 2, 3);

	findContours(canny_output, contours, hierarchy, CV_RETR_TREE,
			CV_CHAIN_APPROX_TC89_KCOS, cv::Point(0, 0));


	int n = 0;
	cv::vector<cv::Point> pointss;
	bool deleted = true;

	vector<int> nrPos;
	Point pt;

	cv::Mat cont = cv::Mat::zeros(canny_output.size(), CV_8UC3);
////	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////Test

	vector<Vec2f> lines;
	  HoughLines(canny_output, lines, 1, CV_PI/180, 75, 0, 0 );
	  cout<<"#####################################"<<endl;
	  for( size_t i = 0; i < lines.size(); i++ )
	  {
	  cout<<lines[i]<<endl;
	  }
	  for( size_t i = 0; i < lines.size(); i++ )
	  {
	     float rho = lines[i][0], theta = lines[i][1];
	     Point pt1, pt2;
	     double a = cos(theta), b = sin(theta);
	     double x0 = a*rho, y0 = b*rho;
	     pt1.x = cvRound(x0 + 1000*(-b));
	     pt1.y = cvRound(y0 + 1000*(a));
	     pt2.x = cvRound(x0 - 1000*(-b));
	     pt2.y = cvRound(y0 - 1000*(a));
	     line( imgRGB, pt1, pt2, Scalar(0,0,255), 3, CV_AA);
	  }




	 imshow("detected lines", imgRGB);

//
//	cv::Mat contTest = cv::Mat::zeros(canny_output.size(), CV_8UC3);
//	cv::Point pX, px, pY, py;
//	vector<int> position;
//	int nrDel = 0;
//
//	bool up, down, left, right;
//	for (int i = 0; i < contours.size(); i++) {
//		cout << "--------------------------------------------------" << endl;
//
////		cv::drawContours(contTest, contours, i, 100, 2, 8, hierarchy, 0,
////								cv::Point());
//		up = false;
//		down = false;
//		right = false;
//		left = false;
//
//		pY = maxY(contours[i]);
//		py = minY(contours[i]);
//		pX = maxX(contours[i]);
//		px = minX(contours[i]);
//
//		up = IsInBlackAreaTop(contours[i][i], upLine);
//		down = IsInBlackAreaDown(contours[i][i], downLine);
//		right = IsInBlackAreaRight(contours[i][i], rightLine);
//		left = IsInBlackAreaLeft(contours[i][i], leftLine);
//
//		if (right == false && left == false && down == false && up == false) {
//
//		} else {
//			position.push_back(i);
//
//		}
//	}
//
////	for (int x = 0; x < position.size(); x++) {
////
////		contours.erase(contours.begin() + position[x] - nrDel);
////		nrDel++;
////
////	}
//
	cv::Mat drawing = cv::Mat::zeros(canny_output.size(), CV_8UC1);

	vector<int> areas;
	vector<Moments> shape(contours.size());

	for (int i = 0; i < contours.size(); i++) {
		shape[i] = moments(contours[i], false);
	}

	for (int i = 0; i < contours.size() / 2; i++) {

//		int area = NormaliseArea(contours[i + i]);
		int area = round(contourArea(contours[i+i]));

		cout<< "AREEA ->>>>>>>>>>>>>>>>>>>"<<i<<"  --------- "<<area <<endl;
		if ((areas.size() <= 4)) {
			if (areas.size() == 0) {
				areas.push_back(area);

			}

			if (areas.size() == 1) {
				if (area > areas[0] + 500 || area < areas[0] - 500) {
					areas.push_back(area);

				}
			}
			if (areas.size() == 2) {
				if ((area > areas[0] + 500 || area < areas[0] - 500)
						&& (area > areas[1] + 500 || area < areas[1] - 500)) {
					areas.push_back(area);

				}
			}

			if (areas.size() == 3) {
				if ((area > areas[0] + 500 || area < areas[0] - 500)
						&& (area > areas[1] + 500 || area < areas[1] - 500)
						&& (area > areas[2] + 500 || area < areas[2] - 500)) {
					areas.push_back(area);

				}
			}

//			if (areas.size() == 4) {
//				if ((area > areas[0] + 100 || area < areas[0] - 100)
//						&& (area > areas[1] + 100 || area < areas[1] - 100)
//						&& (area > areas[2] + 100 || area < areas[2] - 100)
//						&& (area > areas[3] + 100 || area < areas[3] - 100)) {
//					areas.push_back(area);
//
//				}
//			}
//
		}

	}

///////////////////////////////////////////////////////////////Get the mass centers:

	vector<Point2f> massCenter(contours.size());
	for (int i = 0; i < contours.size(); i++) {
		massCenter[i] = Point2f(shape[i].m10 / shape[i].m00,
				shape[i].m01 / shape[i].m00);
		circle(cont, massCenter[i], 10, Scalar(0, 0, 255), 3, 8, 0);

	}

//	for (int i = 0; i < contours.size() / 2; i++) {
//		cout<< " AAAAAAAAAAAAA------>"<< areas[i]<< " "<<endl;
//	}


	for (int i=0; i<areas.size();i++)
	{
		cout<<areas.size()<< " |||| "<<areas[i]<<endl;
	}
//////////////////////////////////////////////////////////////////DETECT TYPE OF FIGURE
//
	for (int i = 0; i < contours.size()/2 ; i++) {


		int area = round(contourArea(contours[i+i]));
		cout<< " ------------------- "<<i<<" ///////// " <<area<<endl;

		if (area < areas[0] + 500 && area > areas[0] - 500) {

			cv::drawContours(cont, contours, i + i, 200, 2, 8, hierarchy, 0,
					cv::Point());
			setLabel(cont, "Piesa 1 ", contours[i + i]);
			cout << "area 1 - " << area << endl;

		}

		if (area < areas[1] + 500 && area > areas[1] - 500) {
			setLabel(cont, "Piesa 2 ", contours[i+i]);
			cv::drawContours(cont, contours, i + i, 100, 2, 8, hierarchy, 0,
					cv::Point());
			cout << "area 2 - " << area << endl;

		}

		if (area < areas[2] + 500 && area > areas[2] - 500) {
			setLabel(cont, "Piesa 3 ", contours[i + i]);
			cv::drawContours(cont, contours, i + i, 100, 2, 8, hierarchy, 0,
					cv::Point());
			cout << "area 3 - " << area << endl;

		}
		if (area < areas[3] + 500 && area > areas[3] - 500) {
			setLabel(cont, "Piesa 4 ", contours[i + i]);
			cv::drawContours(cont, contours, i + i, 100, 2, 8, hierarchy, 0,
					cv::Point());

			cout << "area 4 - " << area << endl;

		}

//		if (area < 6500 && area > 6400) {
//					setLabel(cont, "Piesa  ", contours[i + i]);
//					cv::drawContours(cont, contours, i+i , 100, 2, 8, hierarchy, 0,
//							cv::Point());
//
//					cout << "area 5 - " << area << endl;
//
//				}

		area = 0;

	}

	imshow("Cont1", cont);


	cv::Mat drawingg = cv::Mat::zeros(canny_output.size(), CV_8UC1);
	cv::drawContours(drawingg, contours, 0, 100, 2, 8, hierarchy, 0,
						cv::Point());
	cv::drawContours(drawingg, contours, 2, 150, 2, 8, hierarchy, 0,
							cv::Point());
	cv::drawContours(drawingg, contours, 4, 200, 2, 8, hierarchy, 0,
							cv::Point());
	cv::drawContours(drawingg, contours, 6, 200, 2, 8, hierarchy, 0,
								cv::Point());
//	cv::drawContours(drawingg, contours, 8, 200, 2, 8, hierarchy, 0,
//									cv::Point());
	cv::drawContours(drawingg, contours, 10, 200, 2, 8, hierarchy, 0,
										cv::Point());
	cv::drawContours(drawingg, contours, 12, 222, 2, 8, hierarchy, 0,
											cv::Point());
	cv::drawContours(drawingg, contours, 14, 500, 2, 8, hierarchy, 0,
												cv::Point());


	cout<< round(contourArea(contours[0]))<< " - "<< round(contourArea(contours[2]))<< " - "<< round(contourArea(contours[4]))<<" - "<<round(contourArea(contours[6]))<<" - "<< round(contourArea(contours[8]))<<" - "<< round(contourArea(contours[10]))<<" - "<< round(contourArea(contours[12]))<<" - "<< round(contourArea(contours[14]))<< " - "<< round(contourArea(contours[16]))<<endl;
	imshow("Draw", drawingg);



///////////////////////////////////////////////////////Find the circles
	vector<Vec3f> circles;
	Mat src_gray;

	bool ok = false;
	cvtColor(imgRGB, src_gray, CV_BGR2GRAY);
	GaussianBlur(src_gray, src_gray, Size(9, 9), 2, 2);
	HoughCircles(src_gray, circles, CV_HOUGH_GRADIENT, 1, src_gray.rows / 8, 40,
			11, 5, 10);
	vector<Point2f> circlesCenter;
	Point center;
	int radius;
	for (size_t i = 0; i < circles.size(); i++) {
		//	if (contours.size() == circles.size()) {
		ok = true;
		Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
		circlesCenter.push_back(center);
		radius = cvRound(circles[i][2]);
		circle(imgRGB, center, 3, Scalar(0, 255, 0), -1, 8, 0);
		circle(imgRGB, center, radius, Scalar(0, 0, 255), 3, 8, 0);

		//	}
	}

//imshow("Circ", imgRGB);

/////////////////////////////////////////////////////Find the end of the figure //////////////////////////////////

	vector<double> distances;
	vector<Point2f> points;

	if (ok == true) {
		for (int j = 0; j < massCenter.size() / 2; j++) {
			for (int i = 0; i < circlesCenter.size(); i++) {
				int dx = circlesCenter[i].x - massCenter[j + j].x;
				int dy = circlesCenter[i].y - massCenter[j + j].y;
				double distance = sqrt(dx * dx + dy * dy);

				if (i == 0) {
					distances.push_back(distance);
					points.push_back(circlesCenter[i]);
					points.push_back(massCenter[j + j]);

				} else {
					if (distance < distances[j]) {
						distances[j] = distance;
						points[j + j] = circlesCenter[i];
						points[j + j + 1] = massCenter[j + j];
					}
				}

			}
		}
	}

	int area1;
	int area2;
	if (distances.size() != 0) {
		for (int i; i < distances.size(); i++) {
			line(imgRGB, points[1 + 1], points[1 + 1 + 1], Scalar(255, 255, 0),
					5, 8, 0);

			cv::drawContours(imgRGB, contours, 1, 100, 2, 8, hierarchy, 0,
					cv::Point());
			cv::drawContours(imgRGB, contours, 2, 100, 2, 8, hierarchy, 0,
					cv::Point());

//			cout<<"area 1 "<<area1<<endl;

			cv::Point a;
			a.x = 0;
			a.y = 0;
			cv::Point b;
			b.x = 10;
			b.y = 110;

			line(imgRGB, a, b, Scalar(255, 255, 0), 5, 8, 0);

			///////////////////////////////////////////////////		cv::imshow("aaa", imgRGB);
			cv::Point c;
			c.x = points[1 + 1].x + 280;
			c.y = points[1 + 1].y + 300;
			cv::Point d;
			d.x = points[1 + 1 + 1].x + 280;
			d.y = points[1 + 1 + 1].y + 300;

			line(realImg, c, d, Scalar(255, 255, 0), 5, 8, 0);
			//cv::imshow("iamge real ", realImg);

		}
	}
	//383.371, 338.426
	circle(imgRGB, Point(405, 313), 3, Scalar(0, 255, 255), 3, 8, 0);

	imshow("Rgbb", imgRGB);

	cv::waitKey(3);

}

int main(int argc, char** argv) {
	ros::init(argc, argv, "image_converter");

	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("/kinect_head/rgb/image_color", 1,
			cloud_cb);

	ros::spin();

	return 0;
}
