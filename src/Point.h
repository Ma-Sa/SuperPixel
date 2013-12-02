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
        unsigned int px,py; //pixel position in image
        float pd;
        float pr; //radius of circle around pixel
        Eigen::Vector3d color;
        Eigen::Vector3f position;
        Eigen::Vector3f normal;
        float weight;
        unsigned short depth() { return position[2];}
        bool valid;

        /** Sets the normal and assures that it points towards the camera (=origin) */
        void setNormal(const Eigen::Vector3f& n) {
            normal = n;
            // force normal to look towards the camera
            // check if point to camera direction and normal are within 90 deg
            // enforce: normal * (cam_pos - pos) > 0
            // do not need to normalize (cam_pos - pos) as only sign is considered
            const float q = normal.dot(-position);
            if(q < 0) {
                normal *= -1.0f;
            }
            else if(q == 0) {
                // this should not happen ...
                normal = Eigen::Vector3f(0,0,-1);
            }
        }

        /** Sets normal from gradient */
        void setNormalFromGradient(const Eigen::Vector2f& g) {
            const float gx = g.x();
            const float gy = g.y();
            const float scl = 1.0f / std::sqrt(gx*gx + gy*gy + 1.0f);
            setNormal(Eigen::Vector3f(scl*gx, scl*gy, -scl));

        }
    };

   point Create3dPoint(cv::Mat& depth, int i, int j){
       Point::point point3D;
       point3D.pd = depth.at<unsigned short>(i,j);
       point3D.px = i;
       point3D.py = j;
       return point3D;
    }


};

}



#endif // POINT_H
