// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>

using namespace std;
std::vector<Eigen::Vector2f> super_sample_step
    {
        {0.25,0.25},
        {0.75,0.25},
        {0.25,0.75},
        {0.75,0.75},
    };


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


static bool insideTriangle(int x, int y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) 
    //is inside the triangle represented by _v[0], _v[1], _v[2]
    Eigen::Vector2f V01;
    V01<<_v[1].x()-_v[0].x(),_v[1].y()-_v[0].y();
    Eigen::Vector2f V12;
    V12<<_v[2].x()-_v[1].x(),_v[2].y()-_v[1].y();
    Eigen::Vector2f V20;
    V20<<_v[0].x()-_v[2].x(),_v[0].y()-_v[2].y();
    Eigen::Vector2f V0n;
    V0n<<x-_v[0].x(),y-_v[0].y();
    Eigen::Vector2f V1n;
    V1n<<x-_v[1].x(),y-_v[1].y();
    Eigen::Vector2f V2n;
    V2n<<x-_v[2].x(),y-_v[2].y();
    float ans1=V01.x()*V0n.y()-V01.y()*V0n.x();
    float ans2=V12.x()*V1n.y()-V12.y()*V1n.x();
    float ans3=V20.x()*V2n.y()-V20.y()*V2n.x();
    if((ans1<0 && ans2<0 && ans3<0)||(ans1>0 && ans2>0 && ans3>0))
    return true;
    else  
    return false;

}
static float Cal_weight(int x, int y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) 
    //is inside the triangle represented by _v[0], _v[1], _v[2]
    float weight = 0;
    if(insideTriangle(x-0.25,y+0.25,_v))
        weight+=0.25;
    if(insideTriangle(x-0.25,y-0.25,_v))
        weight+=0.25;
    if(insideTriangle(x+0.25,y-0.25,_v))
        weight+=0.25;
    if(insideTriangle(x+0.25,y+0.25,_v))
        weight+=0.25;
    return weight;

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
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    
    float min_x = min(v[0].x(),min(v[1].x(),v[2].x()));
    float min_y = min(v[0].y(),min(v[1].y(),v[2].y()));
    float max_x = max(v[0].x(),max(v[1].x(),v[2].x()));
    float max_y = max(v[0].y(),max(v[1].y(),v[2].y()));

    
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle
    bool anti=true;
    if(anti)
    {
        for(int x=min_x;x<=max_x;x++){
    for(int y=min_y;y<max_y;y++){
int judge = 0;
for (int i = 0; i < 4; i++)
            {
                if (insideTriangle(x + super_sample_step[i][0], y + super_sample_step[i][1], t.v))
                {
                    auto tup = computeBarycentric2D(x + super_sample_step[i][0], y + super_sample_step[i][1], t.v);
                    float alpha;
                    float beta;
                    float gamma;
                    std::tie(alpha, beta, gamma) = tup;
                    float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;
                    if (super_depth_buf[get_super_index(x*2 + i % 2, y*2 + i / 2)] > z_interpolated)
                    {
                        judge = 1;
                        // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
                        //深度存入缓存
                        super_depth_buf[get_super_index(x*2 + i % 2, y*2 + i / 2)] = z_interpolated;
                        //颜色存入缓存
                        super_frame_buf[get_super_index(x*2 + i % 2, y*2 + i / 2)] = t.getColor();
                    }
                }
            
            if (judge)
            //若像素的四个样本中有一个通过了深度测试，就需要对该像素进行着色，因为有一个通过就说明有颜色，就需要着色。
            {
                Vector3f point = { (float)x,(float)y,0 };
                Vector3f color = (super_frame_buf[get_super_index(x*2 , y*2)]+ super_frame_buf[get_super_index(x*2+1, y*2)]+ super_frame_buf[get_super_index(x*2, y*2+1)]+ super_frame_buf[get_super_index(x*2+1, y*2+1)])/4;
                //着色
                set_pixel(point, color);
            }
        }

}   
}
    }
    else{
for(int x=min_x;x<=max_x;x++){
    for(int y=min_y;y<max_y;y++){
        if(insideTriangle(x,y,t.v))
        {
     // If so, use the following code to get the interpolated z value.
    auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    z_interpolated *= w_reciprocal;
    
    int index = get_index(x,y);
    if(depth_buf[index]>z_interpolated) //update color
    {// TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
        Eigen::Vector3f point;
        point <<x,y,z_interpolated;
        depth_buf[index]=z_interpolated;
        set_pixel(point,t.getColor());
    }
    }
}   
}
    }

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
        std::fill(super_frame_buf.begin(), super_frame_buf.end(), Eigen::Vector3f{0, 0, 0});

    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
        std::fill(super_depth_buf.begin(), super_depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    super_frame_buf.resize(w * h * 4);
    super_depth_buf.resize(w * h * 4);

}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}
int rst::rasterizer::get_super_index(int x, int y)
{
    return (height*2 - 1 - y) * width*2 + x;
}
void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on