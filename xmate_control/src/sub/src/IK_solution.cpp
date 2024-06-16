#include <cmath>
#include <vector>
#include <typeinfo>
#include <type_traits>
#include <Eigen/Geometry> // 用于替代 scipy.spatial.transform 的 Rotation
#include <Eigen/Dense> //我们可以使用Eigen库来处理矩阵和向量运算，这与Python中的NumPy库类似。

#define NMB 7 //我也不知道该怎么提前把关节数量传进来，只能先这么搞着
                //PS: 本来是用N的，但是变量N和别的某个包冲突了，怒从心头起，恶向胆边生，就叫NMB了

using namespace std;
using namespace Eigen;

class IK_solution {
private:
    Eigen::Matrix<double, NMB, 4> DH;
    std::vector<double> joints_lower_bond;
    std::vector<double> joints_upper_bond;
    // std::vector<double> joint_range;
    // std::vector<double> joints_mid_pos;
    Eigen::Matrix4d ee_mat;
    Eigen::Matrix4d first_bias;
    Eigen::Matrix4d ee_bias;
    double max_joint_v;
    int joints_num;

public:
    // 构造函数
    IK_solution(const Eigen::Matrix<double, NMB, 4>& DH_Mat,
                const std::vector<double>& lower_bond,
                const std::vector<double>& upper_bond,
                const Eigen::Matrix4d& first_bias = Eigen::Matrix4d::Identity(),
                const Eigen::Matrix4d& ee_bias = Eigen::Matrix4d::Identity(), 
                double max_joint_v = 0.1)
        : DH(DH_Mat), first_bias(first_bias), ee_bias(ee_bias), max_joint_v(max_joint_v), joints_num(NMB) {
        
        // 将角度从度转换为弧度
        // joints_lower_bond = lower_bond;
        // joints_upper_bond = upper_bond;
        // for (auto& bond : joints_lower_bond) {
        //     bond *= M_PI / 180.0;
        // }
        // for (auto& bond : joints_upper_bond) {
        //     bond *= M_PI / 180.0;
        // }
        
        // 计算关节范围和中间位置
        // joint_range.resize(joints_num);
        // joints_mid_pos.resize(joints_num);
        // for (int i = 0; i < joints_num; ++i) {
        //     joint_range[i] = joints_upper_bond[i] - joints_lower_bond[i];
        //     joints_mid_pos[i] = (joints_lower_bond[i] + joints_upper_bond[i]) / 2.0;
        // }
        
        // 初始化末端执行器矩阵为单位矩阵
        ee_mat = Eigen::Matrix4d::Identity();
    }


    Eigen::Matrix4d FK(const VectorXd& angles) {
        assert(DH.rows() == angles.size());

        // 初始化变换矩阵T0为first_bias
        Eigen::Matrix4d T0 = first_bias;

        for (int i = 0; i < joints_num; ++i) {
            // 根据DH参数构建变换矩阵T
            Eigen::Matrix4d T;
            double theta = angles[i];//关节角度theta，全旋转轴机械臂
            double alpha = DH(i,1);
            double s_theta = std::sin(theta);
            double c_theta = std::cos(theta);
            double s_alpha = std::sin(alpha);
            double c_alpha = std::cos(alpha);
            T << c_theta, -s_theta, 0, DH(i, 0),
                 s_theta*c_alpha, c_theta * c_alpha, -s_alpha, -s_alpha*DH(i,2),
                 s_theta*s_alpha, c_theta*s_alpha, c_alpha, c_alpha*DH(i,2),
                 0, 0, 0, 1;

            // 累积变换
            T0 = T0 * T;
        }

        // 如果需要，将末端执行器的偏移应用于最终的变换矩阵
        // 由于代码中没有明确指出何时使用ee_bias，这里假设在累积完所有变换后使用
        T0 = T0 * ee_bias;
        //ee_mat=T0; //更新末端执行器位置

        return T0;
    }
    
    Eigen::Matrix<double, 6, NMB> Ja(const VectorXd& angles) {
        //cout<<"DH size"<<DH.rows()<<"  angles size"<<angles.size()<<endl;
        assert(DH.rows() == angles.size());
        Eigen::Matrix<double, 6, NMB> J(6, NMB); // 6xN的雅可比矩阵
        Eigen::Matrix<double, NMB, 3>Z0i;
        Eigen::Matrix<double, NMB, 3>o0i;
        Eigen::Matrix4d T0=first_bias;

        for (int i = 0; i < joints_num; ++i) {
            // 根据DH参数构建变换矩阵T
            Eigen::Matrix4d T;
            double theta = angles(i);//关节角度theta，全旋转轴机械臂
            double alpha = DH(i,1);
            double s_theta = std::sin(theta);
            double c_theta = std::cos(theta);
            double s_alpha = std::sin(alpha);
            double c_alpha = std::cos(alpha);
            T << c_theta, -s_theta, 0, DH(i, 0),
                 s_theta*c_alpha, c_theta * c_alpha, -s_alpha, -s_alpha*DH(i,2),
                 s_theta*s_alpha, c_theta*s_alpha, c_alpha, c_alpha*DH(i,2),
                 0, 0, 0, 1;

            T0 = T0 * T;//从前一个关节坐标乘以T转换到当前关节
            if (i==joints_num-1){
                T0=T0 * ee_bias; //最后一个乘一个偏移量，这样可以实现RCM运动
            }

            // Z0i(i,0) = T0(0,2);
            // Z0i(i,1) = T0(1,2);
            // Z0i(i,2) = T0(2,2);
            // o0i(i,0) = T0(0,3);
            // o0i(i,1) = T0(1,3);
            // o0i(i,2) = T0(2,3);
            Z0i.block<1, 3>(i, 0) = T0.block<3, 1>(0, 2);//换一个更骚的写法
            o0i.block<1, 3>(i, 0) = T0.block<3, 1>(0, 3);
        }
        ee_mat=T0; //更新末端执行器位置
        // 计算雅可比矩阵
        Eigen::Vector3d o0n = T0.block<3, 1>(0, 3);
        for (int i = 0; i < joints_num; ++i) {
            J.block<3, 1>(3, i) = Z0i.row(i); // 线性速度部分
            Eigen::Vector3d Jvi = Z0i.row(i).cross(o0n.transpose() - o0i.row(i));
            J.block<3, 1>(0, i) = Jvi; // 角速度部分
        }

        return J;
    }

    Eigen::Matrix3d hat(const Eigen::Vector3d& k) {
        /*这函数没用了，Kimi一招大道至简，用不上了        
            HAT
            khat = hat(k)

            matrix cross-product for a 3 x 3 vector

                    [  0 -k3  k2]
            khat =  [ k3   0 -k1]
                    [-k2  k1   0]
            :param k:
            :return:
        */
        // 创建3x3的零矩阵
        Eigen::Matrix3d khat;
        khat << 0, -k(2), k(1),
                k(2),  0, -k(0),
                -k(1), k(0),  0;
        return khat;
    }
    Eigen::Matrix4d quatproduct(const Eigen::Vector4d& q) {
        /* 
        我让Kimi写的简洁而优雅一点，事实上，确实写非常简洁而优雅，hat那个函数都不用了
        果然是大道至简啊...

        QUATPRODUCT

        Q = quatproduct(q)

        generates matrix representation of a Hamilton quaternion product
        operator

        in: q = [q0;qv];  matlab默认行向量
        out: Q = [q0 -qv'; qv q0*eye(3)+ cross(qv)] //corss是上边的hat函数
        */
        // 计算q的伴随矩阵（即q的Hamilton积）
        Eigen::Matrix4d Q;
        Q <<  q[0], -q[1], -q[2], -q[3],
            q[1],  q[0], -q[3],  q[2],
            q[2],  q[3],  q[0], -q[1],
            q[3], -q[2],  q[1],  q[0];

        return Q;
        }

    Eigen::Matrix<double, 3, 4> quatjacobian(const Eigen::Vector4d& q) {
        /*
        学Kimi，大道至简，搞定
        QUATJACOBIAN

        Jq = quatjacobian(q)
        in: (4 x 1) unit quaternion q = [q0;qv]
        out: (4 x 3) Jacobian Jq that relates (3 x 1) angular velocity
                    to (4 x 1) unit quaternion velocity
        */
        // double q0 = q[0];
        // Eigen::Vector3d qv = q.tail<3>();
        // double scale = 0.5;

        Eigen::Matrix<double, 3, 4> Q;
        Q << -q[1],q[0],q[3],-q[2],
            -q[2],-q[3],q[0],q[1],
            -q[3],q[2],-q[1],q[0];
        Q = Q*0.5;
        return Q;
        }

    Eigen::Vector4d quatcomplement(const Eigen::Vector4d& q) {
        // 生成四元数的补数，即qc = [q0; -qv]
        return Eigen::Vector4d(q[0], -q[1], -q[2], -q[3]);
        }

    Eigen::VectorXd IK_solver(
        const Eigen::VectorXd& angles,
        const Eigen::Vector3d& target_pos,
        const Eigen::Vector3d& target_euler){  // 假设ik_solver是您的逆运动学求解器实例
        
        // 计算雅可比矩阵的位置和角度部分
        Eigen::Matrix<double, 6, NMB> Ja_mat = Ja(angles);

        // 获取雅可比矩阵的位置部分（前3行）
        Eigen::Matrix<double, 3,  NMB> Jp = Ja_mat.block<3, NMB>(0, 0);

        // 获取雅可比矩阵的角度部分（后3行）
        Eigen::Matrix<double, 3,  NMB> Jw = Ja_mat.block<3, NMB>(3, 0);

        Eigen::Vector3d ee_pos;
        ee_pos << ee_mat(0, 3), ee_mat(1, 3), ee_mat(2, 3);//当前平移位置
        Eigen::Vector3d delta_pos = target_pos - ee_pos;//平移差值

        // 将目标欧拉角转换为四元数
        Eigen::Vector4d roTd = eular_to_quaternion(target_euler);

        // 获取当前末端执行器的方向四元数
        Eigen::Vector4d roT = matrix_to_quaternion(ee_mat);

        // 计算四元数的误差
        Eigen::Vector4d er = quatproduct(roTd) * quatcomplement(roT);

        // 计算四元数雅可比矩阵
        // Eigen::Matrix<double, 3, 4> Jr = quatjacobian(er);

        Eigen::Matrix<double, 4, NMB> Jrv = quatjacobian(er).transpose() * Jw;

        // 构建完整的雅可比矩阵
        // Eigen::Matrix<double, 6, Eigen::Dynamic> Ja_mat(6, angles.size() + 3);
        // Ja_mat << Jp, Jr.col(1).eval();  // 使用.eval()来避免不必要的复制

        Eigen::Matrix<double, 6, NMB> Ja_mat_cat;
        Ja_mat_cat.block<3, NMB>(0, 0) = Jp;
        Ja_mat_cat.block<3, NMB>(3, 0) = Jrv.block<3, NMB>(1,0);

        // 构建目标向量
        // Eigen::VectorXd delta_x(angles.size() + 3);
        // delta_x << delta_pos, er.vec();
        Eigen::VectorXd delta_x(6);
        delta_x <<delta_pos, er.tail<3>();

        // 使用伪逆乘以向量b
        // 求伪逆的pseudoInverse()折腾了半天都没搞懂返回的是个什么玩意，给你机会你也不中用啊Kimi！
        // 所以最后没有使用伪逆，用了Eigen库提供（建议）的另一种方法，solve()方法，minimize||AX-B||，给出A和B用solve()直接计算X、
        // 说是更稳定更有效，所以就用这个了，求伪逆的方法有点太数学了，这个更工程一点
        Eigen::VectorXd action = Ja_mat_cat.completeOrthogonalDecomposition().solve(delta_x);

        return action;
    }

    Eigen::Vector4d eular_to_quaternion(const Eigen::Vector3d& euler) {
        // 根据欧拉角创建一个旋转轴-角度表示
        Eigen::AngleAxisd z_axis(euler(2), Eigen::Vector3d::UnitZ());
        Eigen::AngleAxisd y_axis(euler(1), Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd x_axis(euler(0), Eigen::Vector3d::UnitX());

        // 组合旋转：首先绕Z轴旋转，然后绕Y轴旋转，最后绕X轴旋转
        Eigen::Quaterniond q(x_axis * y_axis * z_axis);
        
        // 调整四元数的顺序为 (w, x, y, z)
        Eigen::Vector4d order_q {q.w(),q.x(),q.y(),q.z()};
        return order_q;
        }
    Eigen::Vector4d matrix_to_quaternion(const Eigen::Matrix4d& R) {
        // 将旋转矩阵转换为四元数
        Eigen::Quaterniond q;

        // Eigen 自动处理四元数的创建，其中 .w() 是四元数的实部 (scalar part)
        // .x(), .y(), .z() 分别是四元数的虚部 (vector part)
        q = Eigen::Quaterniond(R.block<3, 3>(0, 0));

        // 由于Eigen的四元数已经按照 (x, y, z, w) 的顺序存储，即 (0, 1, 2, 3)，
        // 我们可以直接返回 quaternion.coeffs() 来获取 (x, y, z, w)，无需重新排序
        // 不管不管，我偏要排序！
        Eigen::Vector4d order_q {q.w(),q.x(),q.y(),q.z()};
        return order_q;
    }
    Eigen::Vector3d matrix_to_eular(const Eigen::Matrix4d& R) {
        // 将旋转矩阵转换为欧拉角，顺序为zyx
        Eigen::Matrix3d rotationMatrix = R.block<3, 3>(0, 0);

        // 将旋转矩阵转换为欧拉角
        // 注意：这里的参数(2, 1, 0)表示ZYX的旋转顺序，您可以根据需要更改它
        Eigen::Vector3d eulerAngles = rotationMatrix.eulerAngles(2, 1, 0);

        return eulerAngles; //返回欧拉角
    }

    Eigen::Vector3d get_eular(const VectorXd& angles) {
        //根据关节角度获取末端执行器的欧拉角，大概也就开始时候用那么一下
        Eigen::Matrix4d m = FK(angles);
        Eigen::Matrix3d rotationMatrix = m.block<3, 3>(0, 0);

        // 将旋转矩阵转换为欧拉角
        // 注意：这里的参数(2, 1, 0)表示ZYX的旋转顺序，您可以根据需要更改它
        // Eigen::Vector3d eulerAngles = rotationMatrix.eulerAngles(2,1,0);

        // Eigen::Vector3d eularAngles = rotationMatrix.eulerAngles(2,1,0);
        // 放弃了用Eigen库里自带的那个，用了下面这个地址找到的：
        // https://blog.csdn.net/weixin_43658047/article/details/134064070
        Eigen::Vector3d eularAngles;
        double sy=sqrt(m(0,0) * m(0,0) +  m(0,1) * m(0,1));
        //ZYX顺序
        // if (sy>0.00001)//判断万向锁奇异
        // {
        //     eularAngles(0) = std::atan2(m(2, 1), m(2, 2));
        //     eularAngles(1) = std::atan2(m(2, 0), sy);
        //     eularAngles(2) = std::atan2(-m(1, 0), m(0, 0));
        // }
        // else//奇异
        // {
        //     eularAngles(0) = std::atan2(-m(1, 2), m(1, 1));;
        //     eularAngles(1) = std::atan2(m(2, 0), sy);
        //     eularAngles(2) = 0;
        // }
        //XYZ顺序
        if (sy>0.00001)//判断万向锁奇异
        {
            eularAngles(0) = std::atan2(-m(1, 2), m(2, 2));
            eularAngles(1) = std::atan2(m(0, 2), sy);
            eularAngles(2) = std::atan2(-m(0, 1), m(0, 0));
        }
        else//奇异
        {
            eularAngles(0) = 0;
            eularAngles(1) = std::atan2(m(0, 2), sy);
            eularAngles(2) = std::atan2(-m(1, 0), m(2, 0));
        }
        // eulerAngles[1]=-eulerAngles[1];
        // for(int i=0;i<3;++i){
        //     if(eulerAngles[i]<-M_PI / 2.0){
        //         eulerAngles[i]=eulerAngles[i]+M_PI;
        //     }
        //     if(eulerAngles[i]>M_PI / 2.0){ 
        //         eulerAngles[i]=eulerAngles[i]-M_PI; 
        //     }
        // }

        // eulerAngles = rotationMatrix.eulerAngles(0,1,2);
        // cout<<"0,1,2 "<<eulerAngles(0)<<endl;
        // eulerAngles = rotationMatrix.eulerAngles(0,2,1);
        // cout<<"0,2,1 "<<eulerAngles(0)<<endl;
        // eulerAngles = rotationMatrix.eulerAngles(1,0,2);
        // cout<<"1,0,2 "<<eulerAngles(0)<<endl;
        // eulerAngles = rotationMatrix.eulerAngles(1,2,0);
        // cout<<"1,2,0 "<<eulerAngles(0)<<endl;
        // eulerAngles = rotationMatrix.eulerAngles(2,0,1);
        // cout<<"2,0,1 "<<eulerAngles(0)<<endl;
        // eulerAngles = rotationMatrix.eulerAngles(2,1,0);
        // cout<<"2,1,0 "<<eulerAngles(0)<<endl; 
        // double a = eularAngles[0];
        // eularAngles[0] = eularAngles[1];
        // eularAngles[1] = a;

        return eularAngles; //返回欧拉角
    }

    Eigen::Vector3d get_position(const VectorXd& angles){
        //根据关节角度获取末端执行器的位置，大概也就开始时候用那么一下
        Eigen::Matrix4d ee_mat = FK(angles);
        return Eigen::Vector3d(ee_mat(0,3), ee_mat(1,3), ee_mat(2,3));
    }
    //todo： upperbound和lowerbond上下界限，但是我不知道怎么做啊？用angel判断一下？

};
