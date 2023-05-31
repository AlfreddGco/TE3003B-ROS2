#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/persistence.hpp>
#include <iostream>

void printMatrix(cv::Mat& mat, std::string prefix = "K"){
	cv::MatIterator_<double> _it = mat.begin<double>();
	std::cout << prefix << " = [";
	for(; _it != mat.end<double>(); _it++){
    		std::cout << *_it << " ";
	}
	std::cout << "]" << std::endl;
}

int main(){
	cv::Mat image, image_gray;
	const char *pipeline = " tcambin serial=15810833 ! video/x-raw, format=BGRx, width=1280,height=960, framerate=25/1 ! videoconvert ! appsink";
	cv::VideoCapture input(0);

	std::vector<cv::Point2f> corners_image;
	std::vector< std::vector<cv::Point2f> > points_image;

	cv::Size pattern_size(7, 5);

	for(;;){
		if(!input.read(image)){
			break;
		}

		cv::cvtColor(image, image_gray, cv::COLOR_BGR2GRAY);
		cv::imshow("gray", image_gray);
		bool found = cv::findChessboardCorners(image_gray, pattern_size, corners_image);
		if(found){
			cv::drawChessboardCorners(image, pattern_size, corners_image, found);
			points_image.push_back(corners_image);
		}

		cv::imshow("image", image);

		char c = cv::waitKey(1) & 0xFF;
		if(c == 27){
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
	
	
	printMatrix(K, "K");
	printMatrix(D, "D");
	
	for(auto &r : Rs){
		printMatrix(r, "Rs");
	}

	for(auto &t : Ts){
		printMatrix(t, "Ts");
	}

	printf("Done Calibration\n");

	cv::Mat undistorted;
	cv::undistort(image_gray, undistorted, K, D);
	cv::imshow("undistorted", undistorted);
	cv::imwrite("./undistored.png", undistorted);
	cv::waitKey(0);

	return 0;
}
