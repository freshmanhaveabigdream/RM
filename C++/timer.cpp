#include<iostream>
#include<functional>
#include<numeric>
#include<windows.h>
#include<vector>
#include<math.h>
#include<random>

float sumFun(std::vector<float>& v){    // 求和函数
    return accumulate(begin(v), end(v), 0.0);
}

float avgFun(std::vector<float>& v){   // 求平均值函数
    return sumFun(v) / v.size();    // 利用求和函数求和，再求均值
}

float varFun(std::vector<float>& v){   // 求方差函数
    float mean = avgFun(v);
    float variance  = 0.0;
    for (auto digit : v) {
        variance = variance + pow(digit-mean,2);
    }
    variance = variance / v.size();
    return variance;
}

float mulFun(std::vector<float>& v){   // 求累乘函数
    float result = 1;
    for (auto digit : v) {
        result *= digit;
    }
    return result;
}

void createTimerLoop(int period, std::vector<float>& data, 
                    std::vector<std::function<float (std::vector<float>&)>>& operations){
    std::random_device rd;
    while(true){
        Sleep(period);  // 暂停period（ms）
        srand(rd());    // 初始化随机数种子
        int flag = rand() % 4;  // 生成一个0-3的数，对应于四种操作
        switch (flag)
        {
        case 0:
            std::cout << "这随机的100个数的和为：" << operations[0](data) << std::endl;
            break;
        case 1:
            std::cout << "这随机的100个数的平均值为：" << operations[1](data) << std::endl;
            break;
        case 2:
            std::cout << "这随机的100个数的方差为：" << operations[2](data) << std::endl;
            break;
        case 3:
            std::cout << "这随机的100个数的累乘乘积为：" << operations[3](data) << std::endl;
            break;
        default:
            break;
        }
    }
}

int main(int argc, char *argv[]){
    SetConsoleOutputCP(CP_UTF8);
    std::vector<std::function<float (std::vector<float>&)>> operations = { std::bind(sumFun, std::placeholders::_1),
                                                                            std::bind(avgFun, std::placeholders::_1),
                                                                            std::bind(varFun, std::placeholders::_1),
                                                                            std::bind(mulFun, std::placeholders::_1) };
                                                                            // 创建一个存储四种操作（普通函数）的数组
    std::vector<float> v;
    v.resize(100);      // 生成一个大小为100的float类型vector数组
    std::default_random_engine rand_gen(time(nullptr));
    std::uniform_real_distribution<float> distrib(0, 5); // 使用0到5的浮点数均匀分布
    for (int i = 0; i < 100; ++i) { // 用随机数给数组赋值
         v[i] =  distrib(rand_gen);
    }
    createTimerLoop(500, v, operations);   // 调用循环函数，定时周期为500ms
    return 0;
}