#include "Point.h"

namespace superpixel{

Point::point Create3dPoint(cv::Mat depth, int i, int j ){
   Point::point point3D;
   point3D.pd = depth.at<unsigned short>(i,j);
   point3D.px = i;
   point3D.py = j;
   //point3D.position = new Eigen::Vector3f((float)point3D.px,(float)point3D.py, point3D.pd);
   return point3D;
}

}

