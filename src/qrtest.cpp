#include <stdio.h> 
#include <vector> 
#include <opencv2/highgui/highgui.hpp> 
#include <opencv2/imgproc/imgproc.hpp> 
#include "qrReader.h" 

using namespace std; 

using namespace cv; 

int main() {
	cv::VideoCapture capture = VideoCapture(1);
	qrReader qr = qrReader();

	if(!capture.isOpened()){
		printf("Unable to open camera");
	}

	Mat image;
	Mat imgBW;
	while(true){
		capture >> image;

		cvtColor(image, imgBW, CV_BGR2GRAY);
		threshold(imgBW, imgBW, 128, 255, THRESH_BINARY);

		bool found = qr.find(imgBW);
		if(found){
			qr.drawFinders(imgBW);
		}
		imshow("image", imgBW);
		waitKey(30);
	}
	waitKey(0);
	return 0;
}
