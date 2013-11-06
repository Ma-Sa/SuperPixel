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

        Eigen::Vector3d color;
        Eigen::Vector3i position;
        unsigned short depth() { return position[2];} //pixel depth
        Eigen::Vector3f surfaceNormal;

    };

    static point Create3dPoint(cv::Mat depth, int i, int j );
};

}



#endif // POINT_H
