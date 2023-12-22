#include <iostream>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv4/opencv2/core/hal/interface.h>
#include <opencv4/opencv2/core/mat.hpp>
#include <opencv4/opencv2/core/types.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>

Eigen::Matrix4d create_outer_mat (Eigen::Quaterniond ori, Eigen::Vector3d camera_position){
    Eigen::Matrix4d outer_mat = Eigen::Matrix4d::Zero();
    outer_mat.block(0, 0, 3, 3) = ori.toRotationMatrix().transpose();
    outer_mat.block(0, 3, 3, 1) = -ori.toRotationMatrix().transpose() * camera_position;
    outer_mat(3, 3) = 1;
    return outer_mat;
}

int main() {
    cv::Mat img(2800, 2800, CV_8UC3);

    Eigen::Quaterniond Orientation(-0.5, 0.5, 0.5, -0.5);//姿态四元数,注意格式
    const Eigen::Vector3d camera_position(2.0,2.0,2.0);
    Eigen::Matrix4d outer_mat = Eigen::Matrix4d::Zero();
    outer_mat = create_outer_mat(Orientation, camera_position);//相机外部参数

    Eigen::Matrix<double, 3, 4> inner_mat;
    inner_mat << 400., 0., 190., 0.,
                 0., 400., 160., 0.,
                 0., 0., 1., 0.;//相机内部参数
    
    freopen("points.txt", "r", stdin);
    int num;    
    std::cin >> num;
    for(int i = 0; i < num; i++) {
        double x,y,z;
        std::cin >> x >> y >> z;
        Eigen::Vector4d world_coordination(x,y,z,1);
        Eigen::Vector3d pixel_coordination = inner_mat*outer_mat*world_coordination;
        cv::Point pixel(pixel_coordination.x()/pixel_coordination.z(), pixel_coordination.y()/pixel_coordination.z());
        cv::circle(img,pixel,1,cv::Scalar(0,0,255),-1);
    }
    cv::imshow("Point", img);
    cv::waitKey(0);
    return 0;
}
//g++ ImitateCamera.cpp -o ImitateCamera `pkg-config opencv4 --cflags --libs`