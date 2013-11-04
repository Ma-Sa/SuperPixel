#include "Density.h"
#include <Eigen/Eigen>
#include "Point.h"


namespace SuperPixel {

float ComputeDensity(Point::point p, float radius) {
    return 1.0f/ComputeArea(p,radius);
}

float ComputeArea(point p, float radius) {
    float pi = 3.14;
    return radius*radius*pi / ComputeDepthGradient(p,radius);

}

Eigen::Vector2f ComputeDepthGradient(point p, float radius) {
    float dw = radius/2.0f;
    float dp; //consider a point in 3d and then back prject the dw in the image plane and find the size
    Eigen::Vector2f gradient;
    gradient.x = depth(p.x+dp, p.y) - depth(p.x-dp, p.y);
    gradient.y = depth(p.x, p.y+dp) - depth(p.x, p.y-dp);
    return gradient / 2.0f * dw;
}

}

