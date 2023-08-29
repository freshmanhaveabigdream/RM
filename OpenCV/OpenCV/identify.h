#ifndef OPENCV_IDENTIFY_H_
#define OPENCV_IDENTIFY_H_

#include<vector>
#include<string>
#include<opencv2/opencv.hpp>
class identification {
private:
	std::vector<std::pair<cv::Scalar, cv::Scalar>> armored_plate_parameter_array;	//���ڴ��������Բ�ͬ��ɫ���ع�ȵĲ���
	std::vector<std::pair<cv::Scalar, cv::Scalar>> center_parameter_array;
	cv::Point2f rotatePoint(cv::Point2f cur_Point, cv::Point2f center, float cur_distance);	//�������������ת�������
	void save(std::string path, std::vector<float> info);	//����¼���ٶ�д��csv�ļ�
public:
	void identify(std::string path, int type_number);	//ʶ����ⲿ�ӿڣ�����Ϊ��Ƶ�ĵ�ַ�Լ���Ƶ�����ͱ��
	identification();
};

#endif // !OPENCV_IDENTIFY_H