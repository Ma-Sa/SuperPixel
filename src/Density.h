#ifndef DENSITY_H
#define DENSITY_H
#include <stdio.h>
#include <Eigen/Eigen>
#include <cmath>
#include "Point.h"
#include "Camera.h"

using namespace std;
namespace superpixel{

class Density
{
public:
    Density();
    static Eigen::MatrixXf ComputeDensity(std::vector<Point::point>& imagePoints, unsigned int rows, unsigned int cols );
    static cv::Mat PlotDensity(Eigen::MatrixXf density);
    cv::Mat GetDepth() { return mDepth;}
    void SetDepth(cv::Mat depth) { mDepth = depth; }

private:
    cv::Mat mDepth;
};

}


#endif // DENSITY_H
