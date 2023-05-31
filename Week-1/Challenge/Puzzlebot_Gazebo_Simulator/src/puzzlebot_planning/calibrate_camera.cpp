#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>

int main(){
	cv::Mat image, image_gray;
	cv::VideoCapture input(0);

	std::vector<cv::Point2f> corners_image;
	std::vector< std::vector<cv::Point2f> > points_image;

	cv::Size pattern_size(8, 6);

	for(;;){
		if(!input.read(image)){
			break;
		}

		cv::cvtColor(image, image_gray, cv::COLOR_BGR2GRAY);
		bool found = cv::findChessboardCorners(image_gray, pattern_size, corners_image);
		if(found){
			cv::drawChessboardCorners(image, pattern_size, corners_image, found);
			points_image.push_back(corners_image);
		}

		cv::imshow("image", image);
		char c = cv::waitKey(0) & 0xFF;
		if(c==27){
			break;
		}
	}

	std::vector<cv::Point3f> corners_world;
	for(int i = 0; i < pattern_size.height; i++){
		for(int j = 0; j < pattern_size.width; j++){
			corners_world.push_back(cv::Point3f(j, i, 0));
		}
	}

	std::vector<std::vector<cv::Point3f>> points_world(points_image.size(), corners_world);

	cv::Mat K, D;
	std::vector<cv::Mat> Rs, Ts;
	double rms = cv::calibrateCamera(points_world, points_image, image_gray.size(), K, D, Rs, Ts);
	std::cout << "Reprojection error: " << rms << std::endl;

	cv::Mat undistorted;
	cv::undistort(image_gray, undistorted, K, D);
	cv::imshow("undistorted", undistorted);
	cv::waitKey();

	return 0;
}
