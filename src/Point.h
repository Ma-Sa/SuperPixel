#ifndef POINT_H
#define POINT_H

#include "Parameters.h"
#include <Eigen/Dense>
#include <vector>

namespace superpixel {

    struct point{
        unsigned int px,py; //pixel position in image
        float depth;
        Eigen::Vector3f color;
        Eigen::Vector3f position;
        float depth() { return position[2];} //pixel depth
        Eigen::Vector3f surfaceNormal;

    };
    Eigen::Vector3f Create3dPoint(unsigned int x, unsigned int y, float d){
       point point3D;
       point3D.px = x;
       point3D.py = y;
       point3D.depth = d;
       point3D.position = new Eigen::Vector3f(x,y,d);
       return point3D;
    }


}



#endif // POINT_H
