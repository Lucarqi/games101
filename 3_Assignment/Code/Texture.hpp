//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;
    Eigen::Vector3f getColorBilinear(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        int xmin,xmax,ymin,ymax;
        xmin=int(u_img);
        xmax=xmin+1;
        ymin=int(v_img);
        ymax=ymin+1;
        //此时需要注意image.data.at(V,U)是先V再U
        float s=u_img-xmin,t=v_img-ymin;
        //std::cout<<s<<"  "<<t<<std::endl;
        cv::Vec3b u01,u11,u00,u10,u00_01,u10_11;
        u00=image_data.at<cv::Vec3b>(ymax,xmin);
        u01=image_data.at<cv::Vec3b>(ymax,xmax);
        u10=image_data.at<cv::Vec3b>(ymin,xmin);
        u11=image_data.at<cv::Vec3b>(ymin,xmax);
        u00_01=u00+s*(u01-u00);
        u10_11=u10+s*(u11-u10);
        auto color=u10_11+t*(u00_01-u10_11);
        //auto color=u10;
        //auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        
        return Eigen::Vector3f(color[0],color[1],color[2]);
    }

    Eigen::Vector3f getColor(float u, float v)
    {
        auto u_img = u * (width-1); //texture修正
        auto v_img = (1 - v) * (height-1);
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

};
#endif //RASTERIZER_TEXTURE_H
