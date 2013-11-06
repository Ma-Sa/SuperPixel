#include "Density.h"


namespace superpixel {

Eigen::Vector2f ComputeDepthGradient(Point::point p, float radius) {
    float dw = radius/2.0f;
    float dp; //consider a point in 3d and then back prject the dw in the image plane and find the size
    Eigen::Vector2f gradient;
    gradient.x();
    gradient.y();
    return gradient / (2.0f * dw);
}

//define inversesquared

float ComputeArea(Point::point p, float radius) {
    float pi = 3.14;
   // return (radius*radius*pi)/inversesquared(ComputeDepthGradient(p,radius)+1);
    return 1.0;
}



vector<vector<float> > Density::ComputeDensity(cv::Mat depth, unsigned int radius) {
    Point::point p;
    vector<vector<float> > cnt(depth.rows, vector<float> (depth.cols));
    for(size_t ii=0; ii < depth.rows ; ++ii)
        for(size_t jj=0; jj < depth.cols ; ++jj){
            p = Point::Create3dPoint(depth, ii, jj);
            cnt[ii][jj] = 1.0f/ComputeArea(p,radius);
        }

    return cnt;
}





}

