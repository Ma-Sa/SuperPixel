#ifndef DENSITY_H
#define DENSITY_H
#include <stdio.h>
#include <Eigen/Eigen>
#include "Point.h"
#include "Camera.h"

using namespace std;
namespace superpixel{

class Density
{
public:
    Density();
    static vector<vector<float> > ComputeDensity(cv::Mat depth, unsigned int Radius);
    static cv::Mat PlotDensity(vector<vector<float> > density);
    cv::Mat GetDepth() { return mDepth;}
    void SetDepth(cv::Mat depth) { mDepth = depth; }

private:
    cv::Mat mDepth;
};

}


#endif // DENSITY_H
