//#include "Interpolator.h"
#include <iostream>
#include <vector>
#include <array>
//#include <boost/math/interpolators/cardinal_quintic_b_spline.hpp>
#include <boost/math/interpolators/cardinal_cubic_b_spline.hpp>

using namespace std;

//感觉根本就不需要写这么个类，但是我真的只想把这两个函数找个地方捏一起

class Interpolator {
    private:
        static const int MIN_SEQ = 5;  // 定义最小序列数，5阶的插值器最少是8个。我读书少，别问我为什么
        static const int DIMENSION = 6; // 定义维度
        static const int N = 50; //每个间隔中的插值个数
        double S = N*0.001; //每个间隔的时间长度
        
    public:

        //左边的导数和二阶导数，公共成员变量，通过Calculate_Mat函数更新，
        //std::vector<std::pair<double, double>> left_endpoint_derivatives;
        std::array<double, DIMENSION> left_endpoint_derivatives;
        std::vector<std::vector<double>> Move_Mat;
        double t= 0;

        Interpolator(const std::vector<double>& First_line){
        std::fill(left_endpoint_derivatives.begin(), left_endpoint_derivatives.end(), 0.0);//初始化设置为0.0
        for (int i = 0; i < MIN_SEQ; ++i) {
            Move_Mat.push_back(First_line);
        }
        }

        std::vector<std::array<double,DIMENSION>> Step(std::vector<double>& One_line){//整合了一下，调用这个函数就得了
            Move_Mat.erase(Move_Mat.begin());
            Move_Mat.push_back(One_line); //队列更新

            t=0;
            std::vector<std::array<double,DIMENSION>> data_mat; //用来存插值后的数据
            std::array<double, DIMENSION> one_move{};;//其中一层// std::array 默认构造时元素都会被初始化为 0
            std::vector<boost::math::interpolators::cardinal_cubic_b_spline<double>> splines;//初始化插值器

            // 从输入矩阵中提取每一列的数据点，并创建插值器
            for (int i = 0; i < DIMENSION; ++i) {
                std::vector<double> data_points;
                for (const auto& row : Move_Mat) {
                    data_points.push_back(row[i]);
                    }
                //这里创建DIMENSION个5阶插值器
                splines.emplace_back(data_points.data(), data_points.size(), 0, S, left_endpoint_derivatives[i]);
            }

            for (int i=0;i<DIMENSION;++i){//更新一个step后的导数们
                left_endpoint_derivatives[i] = splines[i].prime(S);
            }

            for(int i=0;i<N;i++){//循环N次，就一个间隔，再生成后面的就不行了，会不丝滑
                for(int j=0; j<DIMENSION;j++){
                    one_move[j]=splines[j](t); 
                    }
                data_mat.push_back(one_move);//这里记录了全部的插值点，用来检查的
                t+=0.001;//时间后移            
            }
            return data_mat;
        }
};

/*
using namespace std;

#define DIMENSION 6 //每一行的元素个数
#define MIN_SEQ 8 //最短序列，3阶插值需要5个，5阶需要8个

// 读取双精度矩阵的函数
std::vector<std::vector<double>> read_file(const std::string& file_path) {
    std::vector<std::vector<double>> matrix;
    std::ifstream file(file_path);

    if (!file.is_open()) {
        std::cerr << "无法打开文件: " << file_path << std::endl;
        return matrix;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        double value;
        std::vector<double> row;
        while (iss >> value) {
            row.push_back(value);
            // 检查是否还有更多元素在本行
            if (iss.peek() == ' ' || iss.peek() == ',') {
                iss.ignore(); // 忽略分隔符
            }
        }
        matrix.push_back(row);
    }

    file.close();
    return matrix;
}

// 定义 interpolate 函数
//输入一个5行6列数组，表示要插值的5个空间位置，t0是起始时间，h是每组空间位置的间隔，总长就是5*h，to do加入两边导数
std::vector<boost::math::interpolators::cardinal_quintic_b_spline<double>> interpolate(
        const std::vector<std::vector<double>>& Move_Space_Arr, double t0, double h,
        std::vector<std::pair<double, double>> left_endpoint_derivatives,
        std::vector<std::pair<double, double>> right_endpoint_derivatives) {
    // 检查输入矩阵是否有足够的行和列
    if (Move_Space_Arr.size() < MIN_SEQ || Move_Space_Arr[0].size() != DIMENSION) {
        throw std::invalid_argument("输入矩阵的行数或列数不符合要求");
    }

    // 创建一个 vector 来存储插值器
    // std::vector<boost::math::interpolators::cardinal_quintic_b_spline<double>> Interpolators_vector(DIMENSION);
    std::vector<boost::math::interpolators::cardinal_quintic_b_spline<double>> Interpolators_vector;

    // 从输入矩阵中提取每一列的数据点，并创建插值器
    for (int i = 0; i < DIMENSION; ++i) {
        std::vector<double> data_points;
        for (const auto& row : Move_Space_Arr) {
            data_points.push_back(row[i]);
        }
        // Interpolators_vector[i] = boost::math::interpolators::cardinal_cubic_b_spline<double>(data_points.data(), data_points.size(), t0, h);
        // Interpolators_vector[i] = boost::math::interpolators::cardinal_quintic_b_spline<double>(data_points.data(),data_points.size(), t0, h);
        Interpolators_vector.emplace_back(data_points.data(), data_points.size(), t0, h, left_endpoint_derivatives[i]);

    }
    return Interpolators_vector;
}



int main() {
    // 测试调用read_file函数
    const std::string file_path = "../move.txt"; // 替换为您的文件路径
    auto data = read_file(file_path);

    int n = 20; //每个间隔值之间插值个数
    double start_bias = 0.0; //初始偏移，从第0段开始，最大400mm延迟
    double t=start_bias;//初始时刻,跳过序列中的前两个，从第三个点移动到第四个点
    double h=0.001*n;
    std::vector<std::vector<double>> seq_mat(MIN_SEQ, std::vector<double>(data[0])); //插值矩阵初始值选用第一个
    std::vector<std::pair<double, double>> left_endpoint_derivatives(DIMENSION, std::make_pair(0.0, 0.0));
    std::vector<std::pair<double,double>>right_endpoint_derivatives(DIMENSION, std::make_pair(0.0, 0.0));

    std::vector<std::array<double,DIMENSION>> data_mat; //用来存插值后的数据
    std::array<double, DIMENSION> one_line{};;//其中一层// std::array 默认构造时元素都会被初始化为 0


    for(int round=0;round<data.size();round++){
        seq_mat.erase(seq_mat.begin());
        seq_mat.push_back(data[round]);
        // std::cout<<"第"<<round<<"个序列"<<std::endl;
        // for(int i=0;i<MIN_SEQ;i++){//打印出来测试一下
        //     std::cout<<seq_mat[i][0]<<" "<<seq_mat[i][1]<<" "<<seq_mat[i][2]<<" "<<seq_mat[i][3]<<" "<<seq_mat[i][4]<<" "<<seq_mat[i][5]<<std::endl;
        // }
        // 调用 interpolate 函数并获取插值器向量
        std::vector<boost::math::interpolators::cardinal_quintic_b_spline<double>> splines = interpolate(seq_mat, t-start_bias, h,
        left_endpoint_derivatives,right_endpoint_derivatives);
        //计算当前段的导数
        for (int i=0;i<DIMENSION;++i){
            left_endpoint_derivatives[i].first = splines[i].prime(t+h);
            left_endpoint_derivatives[i].second = splines[i].double_prime(t+h);
        }
        // 测试插值器
        for(int i=0;i<n;i++){//注意这个i=5要改的，要改成h/dt,比如h是0.05 dt是0.01那么i就小于5，这个间隔需要插入5个点，当然你可以写成一个自动计算的，随你
            //这里写机械臂的控制函数，反正是
            for(int j=0; j<DIMENSION;j++){
                one_line[j]=splines[j](t); 
                }
            //std::cout << "插值点 " << t << " " << splines[0](t)<< " " << splines[1](t)<< " " << splines[2](t)<< " " << splines[3](t)<< " " << splines[4](t)<< " " << splines[5](t) << std::endl;
            data_mat.push_back(one_line);//这里记录了全部的插值点，用来检查的
            t+=0.001;            
        }
    }
    ofstream write_file("points.txt", ios::out);
    for (int i = 0; i <data_mat.size(); ++i) {
        for(int j=0; j<DIMENSION;++j){
            write_file<<data_mat[i][j]<<" ";
        }
    write_file<<"\n";            
    }

    return 0;
}

*/