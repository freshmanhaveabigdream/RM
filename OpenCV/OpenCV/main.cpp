// 说明
// 本套代码对低曝光的视频识别效果比标准曝光的要好，故标准曝光使用10rpm的固定速度预测，而低曝光使用实时转速预测
// 代码形成的速度文件，保存到了csv中，可以使用excel画图
// 经过验证，低曝光的视频存在每两帧相同的情况，所以跳过偶数帧解决这个问题
// 保存的视频转为了mp4格式再上传

#include "identify.h"

int main() {
	identification ident;
	ident.identify("Resources/北理珠2021正弦能量机关视频/4低曝光蓝.mp4", 4);	// 第二个参数为文件名的第一个数字
	return 0;
}
