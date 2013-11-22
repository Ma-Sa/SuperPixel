#include "Point.h"

namespace superpixel{

Point::point Point::Create3dPoint(cv::Mat& depth, int i, int j){
   Point::point point3D;
   point3D.pd = depth.at<unsigned short>(i,j);
   point3D.px = i;
   point3D.py = j;
   return point3D;
}

}

