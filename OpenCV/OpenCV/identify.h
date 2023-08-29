#ifndef OPENCV_IDENTIFY_H_
#define OPENCV_IDENTIFY_H_

#include<vector>
#include<string>
#include<opencv2/opencv.hpp>
class identification {
private:
	std::vector<std::pair<cv::Scalar, cv::Scalar>> armored_plate_parameter_array;	//用于存放四种针对不同颜色和曝光度的参数
	std::vector<std::pair<cv::Scalar, cv::Scalar>> center_parameter_array;
	cv::Point2f rotatePoint(cv::Point2f cur_Point, cv::Point2f center, float cur_distance);	//计算点绕中心旋转后的坐标
	void save(std::string path, std::vector<float> info);	//将记录的速度写入csv文件
public:
	void identify(std::string path, int type_number);	//识别的外部接口，参数为视频的地址以及视频的类型编号
	identification();
};

#endif // !OPENCV_IDENTIFY_H