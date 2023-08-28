#include<iostream>
#include<functional>
#include<numeric>
#include<windows.h>
#include<vector>
#include<math.h>
#include<random>

void multiplyBigNumber(std::vector<int>& bigNumber, int multiplier) {   // 大数乘法，用于解决100个数相乘现有数据类型无法保存结果的问题
    int n = bigNumber.size();
    int carry = 0;
    for (int i = n - 1; i >= 0; --i) {
        int product = bigNumber[i] * multiplier + carry;
        bigNumber[i] = product % 10;
        carry = product / 10;
    }
    while (carry > 0) {
        bigNumber.insert(bigNumber.begin(), carry % 10);
        carry /= 10;
    }
}

template<class T>
float add(std::vector<T>& v){    // 求和函数
    return accumulate(begin(v), end(v), static_cast<T>(0));
}

template<class T>
float get_mean(std::vector<T>& v){   // 求平均值函数
    return static_cast<float>(add(v)) / 100;    // 利用求和函数求和，并将结果强制转换类型为float类型，便于求均值
}

template<class T>
float get_var(std::vector<T>& v){   // 求方差函数
    double mean = get_mean(v);
    double variance  = 0.0;
    for (int i = 0 ; i < v.size() ; i++)
    {
        variance = variance + pow(v[i]-mean,2);
    }
    variance = variance / v.size();
    return variance;
}

template<class T>
float accumulative_multiplication(std::vector<T>& v){   // 求累乘乘积函数，由于累乘结果可能很大，现有数据类型无法存储，故采用大数乘法
    int carry = 0;
    std::vector<T> temp = {1};
    for (size_t i = 0; i < v.size(); i++) {
        multiplyBigNumber(temp, v[i]);
    }
    for (int digit : temp) {
        std::cout << digit;
    }
    std::cout << std::endl;
    return 0;
}

template<typename T>
float createTimerLoop(int period, std::vector<T>& data, 
                    std::vector<std::function<float (std::vector<T>&)>>& operations){
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
            std::cout << "这随机的100个数的累乘乘积为：";
            operations[3](data);
            break;
        default:
            break;
        }
    }
}

int main(int argc, char *argv[]){
    SetConsoleOutputCP(CP_UTF8);
    std::vector<std::function<float (std::vector<int>&)>> operations = { std::bind(add<int>, std::placeholders::_1),
                                                                            std::bind(get_mean<int>, std::placeholders::_1),
                                                                            std::bind(get_var<int>, std::placeholders::_1),
                                                                            std::bind(accumulative_multiplication<int>, std::placeholders::_1) };
                                                                            // 创建一个存储四种操作（普通函数）的数组
    std::vector<int> v;
    v.resize(100);      // 生成一个大小为100的int类型vector数组
    std::default_random_engine rand_gen;
    std::uniform_int_distribution<int> distrib(1, 100); // 使用1到100的整数分布
    for (int i = 0; i < 100; ++i) { // 用随机数给数组赋值
         v[i] =  distrib(rand_gen);
    }
    createTimerLoop<int>(500, v, operations);   // 调用循环函数，定时周期为500ms
    return 0;
}