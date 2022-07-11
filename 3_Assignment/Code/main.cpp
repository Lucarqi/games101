#include <iostream>
#include <opencv2/opencv.hpp>

#include "global.hpp"
#include "rasterizer.hpp"
#include "Triangle.hpp"
#include "Shader.hpp"
#include "Texture.hpp"
#include "OBJ_Loader.h"
#include <math.h>


Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1,0,0,-eye_pos[0],
                 0,1,0,-eye_pos[1],
                 0,0,1,-eye_pos[2],
                 0,0,0,1;

    view = translate*view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float angle)
{
    Eigen::Matrix4f rotation;
    angle = angle * MY_PI / 180.f;
    rotation << cos(angle), 0, sin(angle), 0,
                0, 1, 0, 0,
                -sin(angle), 0, cos(angle), 0,
                0, 0, 0, 1;

    Eigen::Matrix4f scale;
    scale << 2.5, 0, 0, 0,
              0, 2.5, 0, 0,
              0, 0, 2.5, 0,
              0, 0, 0, 1;

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    return translate * rotation * scale;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    // TODO: Use the same projection matrix from the previous assignments
    //eye_pos转化为弧度制
    Eigen::Matrix4f projection;
    float o=eye_fov*(MY_PI/180)/2,n=zNear,f=zFar;
    float l=f-n;//cube长度
    float h=2*n*tan(o);//cube高度
    float w=h*aspect_ratio;//cube宽度
    Eigen::Matrix4f trans1,trans2,trans3;//表示透视到正交，正交平移，正交缩放
    trans1<<n,0,0,0,
            0,n,0,0,
            0,0,n+f,-(n*f),
            0,0,-1,0;
    trans2<<1,0,0,0,
            0,1,0,0,
            0,0,1,-(n+f)/2,  //正交平移的x,y轴平移?默认对称
            0,0,0,1;
    trans3<<2/w,0,0,0,
            0,2/h,0,0,
            0,0,2/l,0,
            0,0,0,1;
    projection=trans3*trans2*trans1*projection;
    return projection;
}

Eigen::Vector3f vertex_shader(const vertex_shader_payload& payload)
{
    return payload.position;
}

Eigen::Vector3f normal_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f return_color = (payload.normal.head<3>().normalized() + Eigen::Vector3f(1.0f, 1.0f, 1.0f)) / 2.f;
    Eigen::Vector3f result;
    result << return_color.x() * 255, return_color.y() * 255, return_color.z() * 255;
    return result;
}

static Eigen::Vector3f reflect(const Eigen::Vector3f& vec, const Eigen::Vector3f& axis)
{
    auto costheta = vec.dot(axis);
    return (2 * costheta * axis - vec).normalized();
}

struct light
{
    Eigen::Vector3f position;
    Eigen::Vector3f intensity;
};

Eigen::Vector3f texture_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f return_color = {0, 0, 0};
    if (payload.texture)
    {
        // TODO: Get the texture value at the texture coordinates of the current fragment
        float u=payload.tex_coords.x(),v=payload.tex_coords.y();
        //return_color=payload.texture->getColor(u,v);
        return_color=payload.texture->getColorBilinear(u,v);
    }
    //std::cout<<return_color<<std::endl;
    Eigen::Vector3f texture_color;
    texture_color << return_color.x(), return_color.y(), return_color.z();

    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = texture_color / 255.f;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = texture_color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    Eigen::Vector3f result_color = {0, 0, 0};

    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.

        //此时需要Hadamard乘积
        Eigen::Vector3f la,ld,ls,k=light.position-point;
        normal.normalize();
        float r=0;
        la=amb_light_intensity.array()*ka.array();
        for(int i=0;i<3;i++) r+=pow(k[i],2);
        r=sqrt(r);//求距离
        //此时需要注意k，v的标准化！！！
        ld=kd.array()*(light.intensity.array()/pow(r,2)) * std::max(0.f,normal.dot(k.normalized()));
        Eigen::Vector3f v=eye_pos-point,h;
        h=(v.normalized()+k.normalized()).normalized();
        ls=ks.array()*(light.intensity.array()/pow(r,2))*pow((std::max(0.f,normal.dot(h))),p);
        result_color+=la;
        result_color+=ld;
        result_color+=ls;
    }

    return result_color * 255.f;
}

Eigen::Vector3f phong_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};    //环境光强度
    Eigen::Vector3f eye_pos{0, 0, 10};      //眼睛位置

    float p = 150;

    Eigen::Vector3f color = payload.color;
    Eigen::Vector3f point = payload.view_pos;   //尚未透视投影的点空间位置
    Eigen::Vector3f normal = payload.normal;    //点的法线方向

    Eigen::Vector3f result_color = {0, 0, 0};
    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.
        Eigen::Vector3f la,ld,ls,k=light.position-point;
        normal.normalize();
        float r=0;
        la=amb_light_intensity.array()*ka.array();
        for(int i=0;i<3;i++) r+=pow(k[i],2);
        r=sqrt(r);//求距离
        //此时需要注意k，v的标准化！！！
        ld=kd.array()*(light.intensity.array()/pow(r,2)) * std::max(0.f,normal.dot(k.normalized()));
        Eigen::Vector3f v=eye_pos-point,h;
        h=(v.normalized()+k.normalized()).normalized();
        ls=ks.array()*(light.intensity.array()/pow(r,2))*pow((std::max(0.f,normal.dot(h))),p);
        result_color+=la;
        result_color+=ld;
        result_color+=ls;
    }

    return result_color * 255.f;
}



Eigen::Vector3f displacement_fragment_shader(const fragment_shader_payload& payload)
{
    
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color; 
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    float kh = 0.2, kn = 0.1;
    
    // TODO: Implement displacement mapping here
    // Let n = normal = (x, y, z)
    // Vector t = (x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z))
    // Vector b = n cross product t
    // Matrix TBN = [t b n]
    // dU = kh * kn * (h(u+1/w,v)-h(u,v))
    // dV = kh * kn * (h(u,v+1/h)-h(u,v))
    // Vector ln = (-dU, -dV, 1)
    // Position p = p + kn * n * h(u,v)
    // Normal n = normalize(TBN * ln)
    float x=normal.x(),y=normal.y(),z=normal.z();
    Eigen::Vector3f n=normal,ln,t,b;
    n.normalize();
    t<<x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z);
    b<<n.cross(t);
    Eigen::Matrix3f TBN;
    TBN<<t,b,n;//矩阵的拼接?
    float dU,dV,u=payload.tex_coords[0],v=payload.tex_coords[1];
    int h=payload.texture->height,w=payload.texture->width;
    float u_=u+1.0/w,v_=v+1.0/h;
    if(u_>1) u_=1.0;
    if(v_>1) v_=1.0;
    dU=kh * kn * ((payload.texture->getColor(u_,v)).norm()-(payload.texture->getColor(u,v)).norm());
    dV=kh * kn * ((payload.texture->getColor(u,v_)).norm()-(payload.texture->getColor(u,v)).norm());
    ln<<-dU,-dV,1.0;
    point=point + kn*n*(payload.texture->getColor(u,v)).norm();
    Eigen::Vector3f normal_;
    normal_=(TBN*ln).normalized();

    Eigen::Vector3f result_color = {0, 0, 0};
    //result_color = normal_;
    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.
        Eigen::Vector3f la,ld,ls,k=light.position-point;
        normal.normalize();
        float r=0;
        la=amb_light_intensity.array()*ka.array();
        for(int i=0;i<3;i++) r+=pow(k[i],2);
        r=sqrt(r);//求距离
        //此时需要注意k，v的标准化！！！
        ld=kd.array()*(light.intensity.array()/pow(r,2)) * std::max(0.f,normal.dot(k.normalized()));
        Eigen::Vector3f v=eye_pos-point,h;
        h=(v.normalized()+k.normalized()).normalized();
        ls=ks.array()*(light.intensity.array()/pow(r,2))*pow((std::max(0.f,normal.dot(h))),p);
        result_color+=la;
        result_color+=ld;
        result_color+=ls;
    }

    return result_color * 255.f;
}


Eigen::Vector3f bump_fragment_shader(const fragment_shader_payload& payload)
{
    
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color; 
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;


    float kh = 0.2, kn = 0.1;

    // TODO: Implement bump mapping here
    // Let n = normal = (x, y, z)
    // Vector t = (x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z))
    // Vector b = n cross product t
    // Matrix TBN = [t b n]
    // dU = kh * kn * (h(u+1/w,v)-h(u,v))
    // dV = kh * kn * (h(u,v+1/h)-h(u,v))
    // Vector ln = (-dU, -dV, 1)
    // Normal n = normalize(TBN * ln)
    /*
    (1) bump mapping 部分的 h(u,v)=texture_color(u,v).norm, 其中 u,v 是 tex_coords, w,h 是 texture 的宽度与高度
    (2) rasterizer.cpp 中 v = t.toVector4()
    (3) get_projection_matrix 中的 eye_fov 应该被转化为弧度制
    (4) bump 与 displacement 中修改后的 normal 仍需要 normalize
    (5) 可能用到的 eigen 方法：norm(), normalized(), cwiseProduct()
    (6) 实现 h(u+1/w,v) 的时候要写成 h(u+1.0/w,v)
    (7) 正规的凹凸纹理应该是只有一维参量的灰度图，而本课程为了框架使用的简便性而使用了一张 RGB 图作为凹凸纹理的贴图，
        因此需要指定一种规则将彩色投影到灰度，而我只是「恰好」选择了 norm 而已。为了确保你们的结果与我一致，我才要求你们都使用 norm 作为计算方法。
    (8) bump mapping & displacement mapping 的计算的推导日后将会在光线追踪部分详细介绍，目前请按照注释实现。
    */
    float x=normal.x(),y=normal.y(),z=normal.z();
    Eigen::Vector3f n=normal,ln,t,b;
    n.normalize();
    t<<x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z);
    b<<n.cross(t);
    Eigen::Matrix3f TBN;
    TBN<<t,b,n;//矩阵的拼接?
    float dU,dV,u=payload.tex_coords[0],v=payload.tex_coords[1];
    int h=payload.texture->height,w=payload.texture->width;
    float u_=u+1.0/w,v_=v+1.0/h;
    if(u_>1) u_=1.0;
    if(v_>1) v_=1.0;
    dU=kh * kn * ((payload.texture->getColor(u_,v)).norm()-(payload.texture->getColor(u,v)).norm());
    dV=kh * kn * ((payload.texture->getColor(u,v_)).norm()-(payload.texture->getColor(u,v)).norm());
    ln<<-dU,-dV,1.0;
    Eigen::Vector3f normal_;
    normal_=(TBN*ln).normalized();

    Eigen::Vector3f result_color = {0, 0, 0};
    result_color = normal_;
    return result_color * 255.f;
}

int main(int argc, const char** argv)
{
    std::vector<Triangle*> TriangleList;

    float angle = 140.0;
    bool command_line = false;

    std::string filename = "output.png";
    objl::Loader Loader;
    std::string obj_path = "../models/spot/";
    //td::string obj_path = "/home/lucar/work/3_Assignment/Code/models/spot";

    // Load .obj File
    bool loadout = Loader.LoadFile("../models/spot/spot_triangulated_good.obj");
    for(auto mesh:Loader.LoadedMeshes)
    {
        for(int i=0;i<mesh.Vertices.size();i+=3)
        {
            Triangle* t = new Triangle();
            for(int j=0;j<3;j++)
            {
                t->setVertex(j,Vector4f(mesh.Vertices[i+j].Position.X,mesh.Vertices[i+j].Position.Y,mesh.Vertices[i+j].Position.Z,1.0));
                t->setNormal(j,Vector3f(mesh.Vertices[i+j].Normal.X,mesh.Vertices[i+j].Normal.Y,mesh.Vertices[i+j].Normal.Z));
                t->setTexCoord(j,Vector2f(mesh.Vertices[i+j].TextureCoordinate.X, mesh.Vertices[i+j].TextureCoordinate.Y));
            }
            TriangleList.push_back(t);
        }
    }

    rst::rasterizer r(700, 700);

    auto texture_path = "hmap.jpg";
    r.set_texture(Texture(obj_path + texture_path));

    std::function<Eigen::Vector3f(fragment_shader_payload)> active_shader = phong_fragment_shader;

    if (argc >= 2)
    {
        command_line = true;
        filename = std::string(argv[1]);

        if (argc == 3 && std::string(argv[2]) == "texture")
        {
            std::cout << "Rasterizing using the texture shader\n";
            active_shader = texture_fragment_shader;
            texture_path = "spot_texture.png";
            r.set_texture(Texture(obj_path + texture_path));
        }
        else if (argc == 3 && std::string(argv[2]) == "normal")
        {
            std::cout << "Rasterizing using the normal shader\n";
            active_shader = normal_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "phong")
        {
            std::cout << "Rasterizing using the phong shader\n";
            active_shader = phong_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "bump")
        {
            std::cout << "Rasterizing using the bump shader\n";
            active_shader = bump_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "displacement")
        {
            std::cout << "Rasterizing using the bump shader\n";
            active_shader = displacement_fragment_shader;
        }
    }

    Eigen::Vector3f eye_pos = {0,0,10};

    r.set_vertex_shader(vertex_shader);
    r.set_fragment_shader(active_shader);

    int key = 0;
    int frame_count = 0;

    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);
        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));

        r.draw(TriangleList);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imwrite(filename, image);

        return 0;
    }

    while(key != 27)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));

        //r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
        r.draw(TriangleList);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imshow("image", image);
        cv::imwrite(filename, image);
        key = cv::waitKey(10);

        if (key == 'a' )
        {
            angle -= 0.1;
        }
        else if (key == 'd')
        {
            angle += 0.1;
        }

    }
    return 0;
}
