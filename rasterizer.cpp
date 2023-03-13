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

//---------------------------------------------------------------inside triangle-------------------------------------------------------
static bool insideTriangle(int x, int y, const Vector3f* _v)
{   
    Vector3f ans1, ans2, ans3, vec1, vec2, vec3;

    vec1 = _v[1] - _v[0];
    ans1 = vec1.cross(Vector3f(x - _v[0][0], y - _v[0][1], 0));     

    vec2 = _v[2] - _v[1];
    ans2 = vec2.cross(Vector3f(x - _v[1][0], y - _v[1][1], 0));   

    if(ans1.transpose()*ans2<0){
        return false;
    }  

    vec3 = _v[0] - _v[2];
    ans3 = vec3.cross(Vector3f(x - _v[2][0], y - _v[2][1], 0));

    if(ans1.transpose()*ans3 < 0){
        return false;
    }

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
}

//-------------------------------------------Screen space rasterization--------------------------------------------------
void rst::rasterizer::rasterize_triangle(const Triangle& t) 
{

    auto v = t.toVector4();
    float alpha, beta, gamma;
    std::tuple<float, float, float> angle(alpha, beta, gamma);
    
    //bounding box
    int x_min = MIN(v[0].x(), MIN(v[1].x(),v[2].x()));
    int x_max = MAX(v[0].x(), MAX(v[1].x(),v[2].x()));

    int y_min = MIN(v[0].y(), MIN(v[1].y(),v[2].y()));
    int y_max = MAX(v[0].y(), MAX(v[1].y(),v[2].y()));
    
    //define sample from 0
    int n=1;
    int m=1;

    for(int i = x_min; i <= x_max; i++){
        for(int j = y_min; j <= y_max; j++)
        {
            std::vector<Vector3f> onePixelColor;
            std::vector<float> pixel_z;
            for(int a=0;a<n;++a)
            {
                for(int b=0;b<m;++b)
                {
                    auto x_msaa=i+((1/(a+1))/2)*(2*a+1);
                    auto y_msaa=j+1-((1/(b+1))/2)*(2*b+1);

                    if(insideTriangle(x_msaa,y_msaa,t.v))
                    {
                        auto angle=computeBarycentric2D(x_msaa,y_msaa,t.v);
                        float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                        float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                        z_interpolated *= w_reciprocal;

                        if(z_interpolated<msaa_depth_buf[get_index(i,j)])
                        {
                            Vector3f color=t.getColor();
                            Vector3f point(3);
                            point << (float)x_msaa, (float)y_msaa, z_interpolated;
                            msaa_depth_buf[get_index(x_msaa,y_msaa)] = z_interpolated;
                            onePixelColor.push_back(color);
                            pixel_z.push_back(z_interpolated);
                        }
                    }
                }
            }
            auto count=onePixelColor.size();
            Vector3f c;
            for(int q=0;q<count;++q)
            {
                c << c[0]+onePixelColor[q][0], c[1]+onePixelColor[q][1], c[2]+onePixelColor[q][2];
            }

            auto angle=computeBarycentric2D(i+0.5,j+0.5,t.v);
            float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
            float z_interpolat = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
            z_interpolat *= w_reciprocal;

            if(z_interpolat<depth_buf[get_index(i,j)])
            {
                depth_buf[get_index(i,j)]=z_interpolat;
            }
            Vector3f pix;
            pix << i,j,z_interpolat;

            set_pixel(pix, c/4);
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
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
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