
#ifndef RASTERIZER_TRIANGLE_H 
#define RASTERIZER_TRIANGLE_H 

#include <eigen3/Eigen/Eigen> 


using namespace Eigen; 
class Triangle{

public:
   
    Vector3f v[3]; 
    Vector3f color[3]; 
    Vector2f tex_coords[3]; 
    Vector3f normal[3]; 
    Triangle();

    void setVertex(int ind, Vector3f ver);
    void setNormal(int ind, Vector3f n); 
    void setColor(int ind, float r, float g, float b); 
    Vector3f getColor() const { return color[0]*255; } 
    void setTexCoord(int ind, float s, float t); 
    std::array<Vector4f, 3> toVector4() const;
};






#endif //RASTERIZER_TRIANGLE_H