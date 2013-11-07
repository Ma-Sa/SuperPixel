#ifndef POINT_H
#define POINT_H

#include "Parameters.h"

#include "cv.h"
#include "highgui.h"

#include <Eigen/Dense>
#include <vector>

namespace superpixel {

class Point{

public:
    Point();

    struct point{
        unsigned short px,py,pd; //pixel position in image
        float pr; //radius of circle around pixel
        Eigen::Vector3d color;
        Eigen::Vector3f position;
        unsigned short depth() { return position[2];}
        bool valid;

    };

    static point Create3dPoint(cv::Mat depth, int i, int j );
};

}



#endif // POINT_H
