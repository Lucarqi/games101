// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    //此时_v已经是变换后的点，忽略z-axis，只计算y-axis和z-axis
    Eigen::Vector3f D(x,y,1);
    float k[3];
    for(int i=0;i<3;i++)
    {
        Eigen::Vector3f p1,p2;
        p1<<_v[i].x(),_v[i].y(),1;
        p2<<_v[(i+1)%3].x(),_v[(i+1)%3].y(),1;
        k[i]=((D-p1).cross(p2-p1)).z();//取各自计算的z值
    }
    if(k[0]*k[1]<0||k[0]*k[2]<0||k[1]*k[2]<0) return false;    
    return true;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
    for(int i=0;i<height;i++)
    {
        for(int j=0;j<height;j++)
        {
            Eigen::Vector3f color(0,0,0);
            for(int p=0;p<3;p++)
            {
                int index=4*(height*i+j)+p;
                color+=color_buff[index];
            }
            Eigen::Vector3f point(i,j,0);
            set_pixel(point,color);
        }
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();//v是容器，含3个点的4维表示。v.at(i)获得i处的值,x,y,z=int
    //确定寻找范围
    int l=INT_MAX,r=INT_MIN,h=INT_MIN,b=INT_MAX;
    for(int i=0;i<3;i++)
    {
        int x=v.at(i)[0],y=v.at(i)[1];
        if(x>r) r=x;
        if(x<l) l=x;
        if(y>h) h=y;
        if(y<b) b=y;
    }
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle
    r=r+1;l=l-1;b=b-1;h=h+1;
    if(r>700) r=700;
    if(l<0) l=0;
    if(h>700) h=700;
    if(b<0) b=0; //限制范围
    //实现简单的rasterization光栅化,要点是判断像素点中心(x+0.5,y+0.5)是否在三角形内部。
    //但是，在set_pixel时用(x,y)int类型，即用(x,y)表示像素点，实际运算用(x+0.5,y+0.5)
    /*for(int i=l;i<r;i++)
    {
        float x=i+0.5;
        for(int j=b;j<=h;j++)
        {
            float y=j+0.5;
            if(insideTriangle(x,y,t.v)==true)
            {
                //auto[alpha,beta,gamma_]=computeBarycentric2D(float(x), float(y), t.v);    //该语句无法正常执行，改写如下
                std::tuple<float,float,float> R;
                R=computeBarycentric2D(x, y, t.v);
                float alpha=std::get<0>(R),beta=std::get<1>(R),gamma_=std::get<2>(R);   //使用了std::tuple()接收多个返回值的方法
                float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma_ / v[2].w());
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma_ * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;

                if(z_interpolated<depth_buf[i*height+j])//深度差值的判断.std::Vector depth_buff.resize(w*h)
                {
                    depth_buf[i*height+j]=z_interpolated;
                    Eigen::Vector3f point(i,j,0);   //屏幕上的点
                    set_pixel(point,t.getColor());  //这里没有颜色差值，三角形的颜色是统一的
                    continue;
                }
                continue;
            }
            else continue;
        }
    }*/

    //实现MASS(2X2)super sample。要点：此时每个像素采样是4个点，需要维护一个4倍大小的depth_buff，再设置4个采样点的颜色值，最后再加起来。
    //此时颜色设置是全局变量
    float x_[4]={0.25,-0.25,-0.25,0.25};
    float y_[4]={0.25,0.25,-0.25,-0.25};

    for(int i=l;i<r;i++)
    {
        float x=i+0.5;
        for(int j=b;j<h;j++)
        {
            float y=j+0.5;
            int sum=0;
            for(int p=0;p<4;p++) 
            {
                if(insideTriangle(x+x_[p],y+y_[p],t.v)==true) 
                {
                    sum++;
                    std::tuple<float,float,float> R;
                    R=computeBarycentric2D(float(x), float(y), t.v);
                    float alpha=std::get<0>(R),beta=std::get<1>(R),gamma_=std::get<2>(R);   //使用了std::tuple()接收多个返回值的方法
                    float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma_ / v[2].w());
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma_ * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;
                    int index=4*(height*i+j)+p;//深度差值的索引
                    
                    if(z_interpolated<depth_buf[index])
                    {
                        depth_buf[index]=z_interpolated;
                        color_buff[index]=t.getColor()*0.25;
                        continue;
                    }
                    continue;
                }
            }
        }
    }
    // If so, use the following code to get the interpolated z value.
    //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //z_interpolated *= w_reciprocal;

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());  //深度差值初始化为无穷
    }
    Eigen::Vector3f zero(0,0,0);
    std::fill(color_buff.begin(),color_buff.end(),zero); //颜色差值初始化
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    //MASS(2X2)此时应该是4倍大小
    depth_buf.resize(4 * w * h);
    //设置颜色全局变量
    color_buff.resize(4 * w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on