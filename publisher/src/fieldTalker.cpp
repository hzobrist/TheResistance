

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "publisher/FieldPositions.h"
#include "publisher/fieldInfo.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define VAL_LOW 180
#define VAL_HIGH 255

#define ORANGE_LOW_H 0
#define ORANGE_HIGH_H 25
#define ORANGE_LOW_S 80
#define ORANGE_HIGH_S 190
#define ORANGE_LOW_V 180
#define ORANGE_HIGH_V 255

#define YELLOW_LOW_H 25
#define YELLOW_HIGH_H 40
#define YELLOW_LOW_S 30
#define YELLOW_HIGH_S 140
#define YELLOW_LOW_V 200
#define YELLOW_HIGH_V 255

#define GREEN_LOW_H 40
#define GREEN_HIGH_H 80
#define GREEN_LOW_S 20
#define GREEN_HIGH_S 180
#define GREEN_LOW_V 120
#define GREEN_HIGH_V 255

#define BLUE_LOW_H 90
#define BLUE_HIGH_H 125
#define BLUE_LOW_S 50
#define BLUE_HIGH_S 255
#define BLUE_LOW_V 180
#define BLUE_HIGH_V 255

#define VIOLET_LOW_H 125
#define VIOLET_HIGH_H 155
#define VIOLET_LOW_S 15
#define VIOLET_HIGH_S 255
#define VIOLET_LOW_V 180
#define VIOLET_HIGH_V 255

#define RED_LOW_H 160
#define RED_HIGH_H 179
#define RED_LOW_S 100
#define RED_HIGH_S 255
#define RED_LOW_V 180
#define RED_HIGH_V 255

#define BROWN_LOW_H 1
#define BROWN_HIGH_H 20
#define BROWN_LOW_S 50
#define BROWN_HIGH_S 120
//broken change to 190 ish if using brown
#define BROWN_LOW_V 250
#define BROWN_HIGH_V 250

#define BALL_LOW_H 160
#define BALL_HIGH_H 179
#define BALL_LOW_S 30
#define BALL_HIGH_S 255
#define BALL_LOW_V 120
#define BALL_HIGH_V 255


//these are the values used for the robot, so change according to robot jersey color
#define ROBOT_LOW_H BLUE_LOW_H
#define ROBOT_HIGH_H BLUE_HIGH_H
#define ROBOT_LOW_S BLUE_LOW_S
#define ROBOT_HIGH_S BLUE_HIGH_S

#define TRASH_IMAGES 2


using namespace std;
using namespace cv;


struct COLORTHRESH{
	int lowH;
	int highH;
	int lowS;
	int highS;
	int lowV;
	int highV;
	string color;
} ORANGE, YELLOW, GREEN, BLUE, VIOLET, RED, BROWN, BALL, 
	ROBOT, ALLY1, ALLY2, OP1, OP2;

struct robots{
	geometry_msgs::Pose2D O;
	geometry_msgs::Pose2D Y;
	geometry_msgs::Pose2D G;
	geometry_msgs::Pose2D B;
	geometry_msgs::Pose2D V;
	geometry_msgs::Pose2D BR;
	geometry_msgs::Pose2D R;
	geometry_msgs::Pose2D AL1;
	geometry_msgs::Pose2D AL2;
	geometry_msgs::Pose2D OP1;
	geometry_msgs::Pose2D OP2;
} ROBOT_POSITIONS;


RNG rng(12345);
ros::Publisher chatter_pub;
geometry_msgs::Pose2D rPos, bPos;
publisher::fieldInfo::Response field;

geometry_msgs::Pose2D ally1Pos, ally2Pos, opponent1Pos, opponent2Pos;
bool setRobotColors = true;
bool flippedField = false;
bool home = true;

int ball_thresh = 5;

// const int MAX_IMG_SAMPLES = 2;

/// Function headers
void publish();
bool getFieldInfo(publisher::fieldInfo::Request  &req, publisher::fieldInfo::Response &res);
geometry_msgs::Pose2D getCenterOfContours(const Mat& src_thresh, int threshold, bool getAngle);
void getRobotPosition(const Mat& src, COLORTHRESH ct, geometry_msgs::Pose2D *robot_pos);
void getBallPosition(const Mat& src, COLORTHRESH ct);
void applyThreshold(const Mat& src, Mat& dest, int iLowH, int iHighH, int iLowS, int iHighS, int iLowV, int iHighV);
void detectFieldInfo(const Mat& src);
void HoughDetection(const Mat& src_gray, const Mat& src_display, int cannyThreshold, int accumulatorThreshold);
void drawField(const Mat& src, publisher::fieldInfo::Response *fld);
void detectPlayers(const Mat& src);

void publish(){
	
	if (ros::ok()){
		// geometry_msgs::Pose2D msg;
		publisher::FieldPositions msg;

		// cout << "rx: " << rPos.x << " ry: " << rPos.y << " rt: " << rPos.theta << endl;
		// cout << "bx: " << bPos.x << " by: " << bPos.y << endl; 

		msg.robot.x = rPos.x;//-field.start;
		msg.robot.y = rPos.y;//-field.top;
		msg.robot.theta = rPos.theta;
		msg.ball.x = bPos.x;//-field.start;
		msg.ball.y = bPos.y;//-field.top;

		msg.ally1.x = ally1Pos.x;//-field.start;
		msg.ally1.y = ally1Pos.y;//-field.top;
		msg.ally1.theta = ally1Pos.theta;
		
		msg.ally2.x = ally2Pos.x;//-field.start;
		msg.ally2.y = ally2Pos.y;//-field.top;
		msg.ally2.theta = ally2Pos.theta;
		
		msg.opponent1.x = opponent1Pos.x;//-field.start;
		msg.opponent1.y = opponent1Pos.y;//-field.top;
		msg.opponent1.theta = opponent1Pos.theta;
		
		msg.opponent2.x = opponent2Pos.x;//-field.start;
		msg.opponent2.y = opponent2Pos.y;//-field.top;
		msg.opponent2.theta = opponent2Pos.theta;

		ROS_INFO("rx: %f ry: %f rtheta: %f bx: %f by: %f", msg.robot.x, msg.robot.y, msg.robot.theta, msg.ball.x, msg.ball.y);
		ROS_INFO("ally 1 rx: %f ry: %f rtheta: %f", msg.ally1.x, msg.ally1.y, msg.ally1.theta);
		ROS_INFO("ally 2 rx: %f ry: %f rtheta: %f", msg.ally2.x, msg.ally2.y, msg.ally2.theta);
		ROS_INFO("opponent 1 rx: %f ry: %f rtheta: %f", msg.opponent1.x, msg.opponent1.y, msg.opponent1.theta);
		ROS_INFO("opponent 2 rx: %f ry: %f rtheta: %f", msg.opponent2.x, msg.opponent2.y, msg.opponent2.theta);

		chatter_pub.publish(msg);
	} else
		exit(0);
}

bool getFieldInfo(publisher::fieldInfo::Request  &req, publisher::fieldInfo::Response &res){
	
	res.height = field.height;
	res.width = field.width;
	res.start = field.start;
	res.end = field.end;
	res.top = field.top;
	res.bottom = field.bottom;
	res.centerX = field.centerX-field.start;
	res.centerY = field.centerY-field.top;
	res.flipped = flippedField;

	ROS_INFO("request: refresh=%d", (int)req.refresh);
	ROS_INFO("sending back field info response: (WxH):(%ldx%ld), start: %ld end: %ld center: (%ld,%ld)", 
		(long int)res.width, (long int)res.height, (long int)res.start, 
		(long int)res.end, (long int)res.centerX, (long int)res.centerY);
	return true;
}

int main(int argc, char** argv)
{
	if(argc > 1){
		string str = argv[1];
		if(str.compare("away") == 0)
			home = false;
	}

	cout << "playing as: " << (home ? "Home" : "Away") << endl;

	// for(int i = 0; i < argc; i++){
	// 	cout << "argument " << i << ": " << argv[i] << endl;
	// }

	ORANGE.lowH = ORANGE_LOW_H; ORANGE.highH = ORANGE_HIGH_H;
	ORANGE.lowS = ORANGE_LOW_S; ORANGE.highS = ORANGE_HIGH_S;
	ORANGE.lowV = ORANGE_LOW_V; ORANGE.highV = ORANGE_HIGH_V;
	ORANGE.color = "ORANGE";

	YELLOW.lowH = YELLOW_LOW_H; YELLOW.highH = YELLOW_HIGH_H;
	YELLOW.lowS = YELLOW_LOW_S; YELLOW.highS = YELLOW_HIGH_S;
	YELLOW.lowV = YELLOW_LOW_V; YELLOW.highV = YELLOW_HIGH_V;
	YELLOW.color = "YELLOW";

	GREEN.lowH = GREEN_LOW_H; GREEN.highH = GREEN_HIGH_H;
	GREEN.lowS = GREEN_LOW_S; GREEN.highS = GREEN_HIGH_S;
	GREEN.lowV = GREEN_LOW_V; GREEN.highV = GREEN_HIGH_V;
	GREEN.color = "GREEN";

	BLUE.lowH = BLUE_LOW_H; BLUE.highH = BLUE_HIGH_H;
	BLUE.lowS = BLUE_LOW_S; BLUE.highS = BLUE_HIGH_S;
	BLUE.lowV = BLUE_LOW_V; BLUE.highV = BLUE_HIGH_V;
	BLUE.color = "BLUE";

	VIOLET.lowH = VIOLET_LOW_H; VIOLET.highH = VIOLET_HIGH_H;
	VIOLET.lowS = VIOLET_LOW_S; VIOLET.highS = VIOLET_HIGH_S;
	VIOLET.lowV = VIOLET_LOW_V; VIOLET.highV = VIOLET_HIGH_V;
	VIOLET.color = "VIOLET";

	RED.lowH = RED_LOW_H; RED.highH = RED_HIGH_H;
	RED.lowS = RED_LOW_S; RED.highS = RED_HIGH_S;
	RED.lowV = RED_LOW_V; RED.highV = RED_HIGH_V;
	RED.color = "RED";

	BROWN.lowH = BROWN_LOW_H; BROWN.highH = BROWN_HIGH_H;
	BROWN.lowS = BROWN_LOW_S; BROWN.highS = BROWN_HIGH_S;
	BROWN.lowV = BROWN_LOW_V; BROWN.highV = BROWN_HIGH_V;
	BROWN.color = "BROWN";

	BALL.lowH = BALL_LOW_H; BALL.highH = BALL_HIGH_H;
	BALL.lowS = BALL_LOW_S; BALL.highS = BALL_HIGH_S;
	BALL.lowV = BALL_LOW_V; BALL.highV = BALL_HIGH_V;
	BALL.color = "BALL";

	//use this to change the robot color
	ROBOT = BLUE;

	field.height = 0;
	field.width = 0;
	field.start = 0;
	field.end = 0;
	field.centerX = 0;
	field.centerY = 0;

	// Set up ros service
	ros::init(argc, argv, "getFieldInfoServer");
	ros::NodeHandle svcNode;

	ros::ServiceServer service = svcNode.advertiseService("getFieldInfo", getFieldInfo);
	ROS_INFO("Ready to send field info.");

	// Set up ros publisher
	ros::init(argc, argv, "talker");

	ros::NodeHandle n;

	chatter_pub = n.advertise<publisher::FieldPositions>("chatter", 1000);
	ros::Rate loop_rate(20);


	VideoCapture cap;
	cap.open("http://192.168.1.79:8080/stream?topic=/image&dummy=param.mjpg");

	if ( !cap.isOpened() )  // if not success, exit program
	{
		cout << "webcam 1 (192.168.1.79) failed" << endl;
		cap.open("http://192.168.1.78:8080/stream?topic=/image&dummy=param.mjpg");
		flippedField = true;
		if ( !cap.isOpened() )  // if not success, exit program
		{
			cout << "Cannot open the web cam" << endl;
			return -1;
		}
	}

	namedWindow("Control", CV_WINDOW_AUTOSIZE);

	cvCreateTrackbar("ballLowH", "Control", &BALL.lowH, 179);
	cvCreateTrackbar("ballHighH", "Control", &BALL.highH, 179);
	cvCreateTrackbar("ballLowS", "Control", &BALL.lowS, 255);
	cvCreateTrackbar("ballHighS", "Control", &BALL.highS, 255);
	cvCreateTrackbar("ballLowV", "Control", &BALL.lowV, 255);
	cvCreateTrackbar("ballHighV", "Control", &BALL.highV, 255);
	cvCreateTrackbar("ball_size_thresh", "Control", &ball_thresh, 100);

	Mat src;

	bool bSuccess = cap.read(src); // read a new frame from video
	if (!bSuccess) //if not success, break loop
	{
		cout << "Cannot read a frame from video stream" << endl;
		return -1;
	}

	// Get field info once and set it
	detectFieldInfo(src);

	detectPlayers(src);

	// infinite loop to display
	// and refresh the content of the output image
	int key = 0;
	int i,j,k;
	clock_t begin, end;
	double time_spent;

	// while(false){
	while(key != 'q'){
		//throw away 10 images
		begin = clock();
		// cout << "start time: " << sTime << endl;
		for(i=0; i < TRASH_IMAGES; i++)
			cap.grab();
		// Read the image
		bSuccess = cap.read(src); // read a new frame from video
		if (!bSuccess) //if not success, break loop
		{
			cout << "Cannot read a frame from video stream" << endl;
			return -1;
		}

		drawField(src, &field);

		int fontFace = FONT_HERSHEY_PLAIN;
		double fontScale = 1;
		int thickness = 1;
		Point pt(0,0);
		Scalar color = Scalar( 10, 255, 25 );

		// Find ball
		getBallPosition(src, BALL);
		pt.x = bPos.x;
		pt.y = bPos.y;
		circle(src, pt, 2, color, 1, 8, 0);
		putText(src, "BALL", pt, fontFace, fontScale, color, thickness, 8, false );

		if(ALLY1.highV != 0){
			getRobotPosition(src, ALLY1, &(ROBOT_POSITIONS.AL1));
			pt.x = ROBOT_POSITIONS.AL1.x;
			pt.y = ROBOT_POSITIONS.AL1.y;
			if(pt.x != 0){
				ally1Pos.x = pt.x;
				ally1Pos.y = pt.y;
				ally1Pos.theta = ROBOT_POSITIONS.AL1.theta;
			}
			// cout << ALLY1.color << endl;
			circle(src, pt, 2, color, 1, 8, 0);
			putText(src, "Ally 1", pt, fontFace, fontScale, color, thickness, 8, false );
		}

		if(ALLY2.highV != 0){
			getRobotPosition(src, ALLY2, &(ROBOT_POSITIONS.AL2));
			pt.x = ROBOT_POSITIONS.AL2.x;
			pt.y = ROBOT_POSITIONS.AL2.y;
			if(pt.x != 0){
				ally2Pos.x = pt.x;
				ally2Pos.y = pt.y;
				ally2Pos.theta = ROBOT_POSITIONS.AL2.theta;
			}
			// cout << ALLY2.color << endl;
			circle(src, pt, 2, color, 1, 8, 0);
			putText(src, "Ally 2", pt, fontFace, fontScale, color, thickness, 8, false );
		}

		if(OP1.highV != 0){
			getRobotPosition(src, OP1, &(ROBOT_POSITIONS.OP1));
			pt.x = ROBOT_POSITIONS.OP1.x;
			pt.y = ROBOT_POSITIONS.OP1.y;
			if(pt.x != 0){
				opponent1Pos.x = pt.x;
				opponent1Pos.y = pt.y;
				opponent1Pos.theta = ROBOT_POSITIONS.OP1.theta;
			}
			// cout << OP1.color << endl;
			circle(src, pt, 2, color, 1, 8, 0);
			putText(src, "Opponent 1", pt, fontFace, fontScale, color, thickness, 8, false );
		}

		if(OP2.highV != 0){
			getRobotPosition(src, OP2, &(ROBOT_POSITIONS.OP2));
			pt.x = ROBOT_POSITIONS.OP2.x;
			pt.y = ROBOT_POSITIONS.OP2.y;
			if(pt.x != 0){
				opponent2Pos.x = pt.x;
				opponent2Pos.y = pt.y;
				opponent2Pos.theta = ROBOT_POSITIONS.OP2.theta;
			}
			// cout << OP2.color << endl;
			circle(src, pt, 2, color, 1, 8, 0);
			putText(src, "Opponent 2", pt, fontFace, fontScale, color, thickness, 8, false );
		}

		publish();
		imshow("Original", src); //show the original image
		// get user key
		ros::spinOnce();
		loop_rate.sleep();
		key = waitKey(1);

		end = clock();
		time_spent = (double)(end - begin) / CLOCKS_PER_SEC * 1000;
		cout << "Elapsed time: " << time_spent << " ms" << endl;
	}

	return 0;
}

geometry_msgs::Pose2D getCenterOfContours(const Mat& src_thresh, int threshold, bool getAngle){

	geometry_msgs::Pose2D pos;
	Mat canny_output;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	/// Detect edges using canny
	Canny( src_thresh, canny_output, threshold, threshold*2, 3 );
	/// Find contours
	findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );


	int numContours = contours.size();
	// cout << "Number of contours:" << numContours << endl;
	if(numContours < 2 || numContours > 4 || numContours == 3)
		return pos;
	vector<Point2f> centers (numContours/2);
	vector<float> radii (numContours/2);
	Point2f moment;
	float radius;
	Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
	for( int i = 0; i< numContours; i++ )
	{
		Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
		drawContours( drawing, contours, i, color, 1, 8, hierarchy, 0, Point() );
		if(i%2 == 0){
			minEnclosingCircle(contours[i], centers[i/2], radii[i/2]);
		}
	}
	
	minEnclosingCircle(centers, moment, radius);
	float angle = 0.0;
	if(getAngle && centers.size()==2){
		int idx[2] = {1,0};
		// first contour is smaller
		if(radii[0] < radii[1]){
			idx[0] = 0;
			idx[1] = 1;
		}
		float x = centers[idx[0]].x - centers[idx[1]].x;
		//flip the y axis
		float y = centers[idx[1]].y - centers[idx[0]].y;
		angle = atan(y/x)*180.0/CV_PI;
		if(angle < 0)
			angle += 180.0;
		if(y < 0)
			angle += 180.0;

		angle = 360-angle;
	}
	pos.x = moment.x;
	pos.y = moment.y;
	pos.theta = angle;
	return pos;
}

void getRobotPosition(const Mat& src, COLORTHRESH ct, geometry_msgs::Pose2D *robot_pos){
	Mat thresh_src;
	applyThreshold(src, thresh_src, ct.lowH, ct.highH, ct.lowS, ct.highS, ct.lowV, ct.highV);
	geometry_msgs::Pose2D pos = getCenterOfContours(thresh_src, 20, true);
	// cout << "pos: " << pos.x << ", " << pos.y << endl;
	robot_pos->x = pos.x;
	robot_pos->y = pos.y;
	robot_pos->theta = pos.theta;

	if(ROBOT.lowH == ct.lowH){
		rPos = pos;
	}
}

void getBallPosition(const Mat& src, COLORTHRESH ct){
	Mat thresh_src;

	applyThreshold(src, thresh_src, ct.lowH, ct.highH, ct.lowS, ct.highS, ct.lowV, ct.highV);
	bPos = getCenterOfContours(thresh_src, ball_thresh, false);
}

void applyThreshold(const Mat& src, Mat& dest, int iLowH, int iHighH, int iLowS, int iHighS, int iLowV, int iHighV){
	Mat hsv;
	//Color Detect
	cvtColor(src, hsv, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

	inRange(hsv, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), dest); //Threshold the image
	 
	//morphological opening (remove small objects from the foreground)
	erode(dest, dest, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
	dilate( dest, dest, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

	//morphological closing (fill small holes in the foreground)
	dilate( dest, dest, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
	erode(dest, dest, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

}

void detectFieldInfo(const Mat& src){

	int rho = 1; // – Distance resolution of the accumulator in pixels.
	int thetaDiv = 180; //– Angle resolution of the accumulator in radians.
	int accThresh = 70; // – Accumulator threshold parameter. Only those lines are returned that get enough votes ( >\texttt{threshold} ).
	int minLineLength = 370; //– Minimum line length. Line segments shorter than that are rejected.
	int maxLineGap = 230;
	
	int iLowH = 0;
	int iHighH = 150;

	int iLowS = 0; 
	int iHighS = 30;

	int iLowV = 85;
	int iHighV = 155;

	int cannyThreshold = 50;
	int accumulatorThreshold = 70;

	Mat hsv, imgThresh, contourImg;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	int thresh = 20;
	//Color Detect
	cvtColor(src, hsv, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

	inRange(hsv, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresh); //Threshold the image
	 
	//morphological opening (remove small objects from the foreground)
	erode(imgThresh, imgThresh, getStructuringElement(MORPH_ELLIPSE, Size(7, 7)) );
	dilate( imgThresh, imgThresh, getStructuringElement(MORPH_ELLIPSE, Size(7, 7)) ); 

	//morphological closing (fill small holes in the foreground)
	dilate( imgThresh, imgThresh, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
	erode(imgThresh, imgThresh, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );


	/// Detect edges using canny
	Canny( imgThresh, contourImg, thresh, thresh*4, 3 );
	/// Find contours
	findContours( contourImg, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

	/// Draw contours
	Mat drawing = Mat::zeros( contourImg.size(), CV_8UC3 );
	cout << "contours: " << contours.size() << endl;
	vector<vector<Point> > contours_poly( contours.size() );
	vector<Rect> boundRect( contours.size() );
	int indexOfLargestRect = -1;
	for( int i = 0; i< contours.size(); i++ )
	{
		Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );

		approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
		boundRect[i] = boundingRect( Mat(contours_poly[i]) );

		// rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2, 8, 0 );
		if(indexOfLargestRect == -1){
			indexOfLargestRect = i;
		} else {
			int curDx = abs(boundRect[indexOfLargestRect].tl().x - boundRect[indexOfLargestRect].br().x);
			int dx = abs(boundRect[i].tl().x - boundRect[i].br().x);
			// cout << "curDx: " << curDx << " dx: " << dx << endl;
			if(dx > curDx)
				indexOfLargestRect = i;
		}
	}

	Point tl, br;
	tl = boundRect[indexOfLargestRect].tl();
	br = boundRect[indexOfLargestRect].br();

	field.end = br.x;
	field.start = tl.x;
	field.top = tl.y;
	field.bottom = br.y;
	field.width = br.x - tl.x;
	field.height = br.y - tl.y;

	Mat src2 = src.clone();
	Mat src_gray2;
	
	// Convert it to gray
	cvtColor( src2, src_gray2, COLOR_BGR2GRAY );

	// Reduce the noise so we avoid false circle detection
	GaussianBlur( src_gray2, src_gray2, Size(9, 9), 2, 2 );
	// those paramaters cannot be =0
	// so we must check here
	cannyThreshold = std::max(cannyThreshold, 1);
	accumulatorThreshold = std::max(accumulatorThreshold, 1);

	//runs the detection, and update the display
	HoughDetection(src_gray2, src2, cannyThreshold, accumulatorThreshold);

	// Mat dst, cdst;
	// // Hough line
	// Canny(imgThresh, dst, 20, 80, 3);
	// // imshow("canny", dst);
	// cvtColor(dst, cdst, CV_GRAY2BGR);

	// vector<Vec4i> lines;

	// HoughLinesP(dst, lines, rho, CV_PI/thetaDiv, accThresh, minLineLength, maxLineGap );
	// // cout << "lines: " << lines.size() << endl;
	// for( size_t i = 0; i < lines.size(); i++ ){
	// 	Vec4i l = lines[i];
	// 	line( cdst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
	// 	int dx = abs(l[0]-l[2]);
	// 	int dy = abs(l[1]-l[3]);
	// 	int xPos = ((l[0]+l[2])/2);
	// 	int yPos = ((l[1]+l[3])/2);
	// 	if(dx > dy){
	// 		// cout << "line at y = " << yPos << endl;
	// 		if(yPos > 200)
	// 			field.height = yPos;
	// 	} else {
	// 		// cout << "line at x = " << xPos << endl;
	// 		if(xPos > 400){
	// 			field.end = xPos;
	// 			field.width = xPos-field.start;
	// 		} else {
	// 			if(field.width != 0 && field.start == 0){
	// 				field.width = field.width - xPos;
	// 			}
	// 			field.start = xPos;
	// 		}
	// 	}
	// }
	
	// imshow("source", src);
	// imshow("Houghlines", cdst);

	cout << "Center: (" << field.centerX << ", " << field.centerY << ")" << endl;
	cout << "Field: (" << field.width << " x " << field.height << ")" << endl;
	cout << "Field start: " << field.start << " end: " << field.end << endl;
	cout << "Field top: " << field.top << " bottom: " << field.bottom << endl;

}


void HoughDetection(const Mat& src_gray, const Mat& src_display, int cannyThreshold, int accumulatorThreshold)
{
	// will hold the results of the detection
	std::vector<Vec3f> circles;
	// runs the actual detection
	HoughCircles( src_gray, circles, HOUGH_GRADIENT, 1, src_gray.rows/8, cannyThreshold, accumulatorThreshold, 0, 0 );
	// int centerX = 0, centerY = 0;
	// clone the colour, input image for displaying purposes
	Mat display = src_display.clone();
	size_t i;
	for( i = 0; i < circles.size(); i++ )
	{
		Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
		int radius = cvRound(circles[i][2]);
		// circle center
		circle( display, center, 3, Scalar(0,255,0), -1, 8, 0 );
		// circle outline
		circle( display, center, radius, Scalar(0,0,255), 3, 8, 0 );
		// cout << "round centerX: " << cvRound(circles[i][0]) << " round centerY: " << cvRound(circles[i][1]) << endl;
		field.centerX = center.x;
		field.centerY = center.y;
	}
	// cout << "centerX: " << field.centerX << " centerY: " << field.centerY << endl;
	
	// shows the results
	// imshow( "Hough Circle Detection", display);
}

void drawField(const Mat& src, publisher::fieldInfo::Response *fld){

	Point leftBottom, leftTop, rightTop, rightBottom, center;
	
	leftBottom.x = fld->start;
	leftBottom.y = fld->bottom;
	leftTop.x = fld->start;
	leftTop.y = fld->top;
	rightTop.x = fld->end;
	rightTop.y = fld->top;
	rightBottom.x = fld->end;
	rightBottom.y = fld->bottom;
	center.x = fld->centerX;
	center.y = fld->centerY;

	// cout << "Center: (" << field.centerX << ", " << field.centerY << ")" << endl;
	// cout << "Field: (" << field.width << " x " << field.height << ")" << endl;
	// cout << "Field start: " << field.start << " end: " << field.end << endl;
	
	line(src, leftBottom, leftTop, Scalar(0,0,255), 1, 8, 0);
	line(src, leftTop, rightTop, Scalar(0,0,255), 1, 8, 0);
	line(src, rightTop, rightBottom, Scalar(0,0,255), 1, 8, 0);
	line(src, leftBottom, rightBottom, Scalar(0,0,255), 1, 8, 0);
	circle(src, center, 3, Scalar(0,0,255), -1, 8, 0 );
}

void detectPlayers(const Mat& src){

	COLORTHRESH colorThreshes [] = {ORANGE, GREEN, BLUE, VIOLET, YELLOW};
	geometry_msgs::Pose2D pos;
	Mat thresh_src;
	COLORTHRESH ct;
	COLORTHRESH opponentColors [5];
	COLORTHRESH allyColors [5];
	int opIndex = 0;
	int allyIndex = 0;
	// cout << "size of colorThreshes: " << sizeof(colorThreshes)/sizeof(COLORTHRESH) << endl;
	for(int i = 0; i < sizeof(colorThreshes)/sizeof(COLORTHRESH); i++){
		ct = colorThreshes[i];
		applyThreshold(src, thresh_src, ct.lowH, ct.highH, ct.lowS, ct.highS, ct.lowV, ct.highV);
		pos = getCenterOfContours(thresh_src, 20, true);
		if(pos.x != 0){
			if(home){
				if(pos.x > field.centerX){
					// this color is an opponent
					opponentColors[opIndex] = ct;
					opIndex++;
				} else {
					// this color is an ally
					allyColors[allyIndex] = ct;
					allyIndex++;
				}
			} else {
				if(pos.x <= field.centerX){
					// this color is an opponent
					opponentColors[opIndex] = ct;
					opIndex++;
				} else {
					// this color is an ally
					allyColors[allyIndex] = ct;
					allyIndex++;
				}
			}
		}
	}

	ALLY1 = allyColors[0];
	ALLY2 = allyColors[1];
	OP1 = opponentColors[0];
	OP2 = opponentColors[1];

	cout << "ALLY 1 color: " << ALLY1.color << endl;
	cout << "ALLY 2 color: " << ALLY2.color << endl;
	cout << "Opponent 1 color: " << OP1.color << endl;
	cout << "Opponent 2 color: " << OP2.color << endl;

	cout << "ALLY 1 color: " << ALLY1.highV << endl;
	cout << "ALLY 2 color: " << ALLY2.highV << endl;
	cout << "Opponent 1 color: " << OP1.highV << endl;
	cout << "Opponent 2 color: " << OP2.highV << endl;

}

