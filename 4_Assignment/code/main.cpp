#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}
//阶乘函数
int jc(int k)
{
    if(k == 0 || k == 1) return 1;
    size_t f = 1;
    for(int i=1;i<=k;i++)
    {
        f *=i;
    }
    return f;
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm
    //直接公式求解
    /*auto point = cv::Point2f(0.0,0.0);
    int n = control_points.size()-1;
    for(int i=0;i<=n;i++)
    {
        point += std::pow(t,i)*std::pow(1-t,n-i)*(jc(n)/(jc(i)*jc(n-i)))*control_points.at(i);
    }
    return point;
    */
    //递归求解
    if(control_points.size() == 2)
    {
        cv::Point2f point = control_points.at(0)*(1-t) + control_points.at(1)*t;
        return point;
    }
    else 
    {
        std::vector<cv::Point2f> new_points;
        for(int i=1;i<control_points.size();i++)
        {
            cv::Point2f new_point = (1-t)*control_points.at(i-1)+t*control_points.at(i);
            new_points.push_back(new_point);
        }
        return recursive_bezier(new_points,t);
    }
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    for(double t = 0.0 ;t <= 1.0 ;t += 0.001)
    {
        auto point = recursive_bezier(control_points,t);
        //直接实现
        //window.at<cv::Vec3b>(point.y,point.x)[1] = 255; 
        //实现反采样2X2
        float p1_x = std::floor(point.x);
        float p1_y = std::floor(point.y);
        float p2_x = p1_x + 1;
        float p2_y = p1_y;
        float p3_x = p1_x;
        float p3_y = p1_y + 1;
        float p4_x = p1_x + 1;
        float p4_y = p1_y + 1;
        float stand = std::sqrt(2);
        float d1 = stand - std::sqrt(std::pow(point.x-p1_x,2) + std::pow(point.y-p1_y,2));
        float d2 = stand - std::sqrt(std::pow(point.x-p2_x,2) + std::pow(point.y-p2_y,2));
        float d3 = stand - std::sqrt(std::pow(point.x-p3_x,2) + std::pow(point.y-p3_y,2));
        float d4 = stand - std::sqrt(std::pow(point.x-p4_x,2) + std::pow(point.y-p4_y,2));
        float d_sum = d1 + d2 + d3 + d4;
        float k1 = d1 / d_sum;
        float k2 = d2 / d_sum;
        float k3 = d3 / d_sum;
        float k4 = d4 / d_sum;
        window.at<cv::Vec3b>(p1_y,p1_x)[1] = std::min(255.0,window.at<cv::Vec3b>(p1_y,p1_x)[1]+k1*255.0);
        window.at<cv::Vec3b>(p2_y,p2_x)[1] = std::min(255.0,window.at<cv::Vec3b>(p2_y,p2_x)[1]+k2*255.0);//多个位置重叠最大取255;
        window.at<cv::Vec3b>(p3_y,p3_x)[1] = std::min(255.0,window.at<cv::Vec3b>(p3_y,p3_x)[1]+k3*255.0);
        window.at<cv::Vec3b>(p4_y,p4_x)[1] = std::min(255.0,window.at<cv::Vec3b>(p4_y,p4_x)[1]+k4*255.0);
    }
}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4) 
        {
            naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
