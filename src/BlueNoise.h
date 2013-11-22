#ifndef BLUENOISE_H
#define BLUENOISE_H

#include "Point.h"
#include "Pyramid.h"
#include <vector>
#include <Eigen/Eigen>

namespace superpixel{
class BlueNoise
{
public:
    BlueNoise();
    struct node{
        int x,y,q;
        float scale, weight;
    };

    static const unsigned int D = 2;
    static std::vector<node> Compute(Eigen::MatrixXf density);
    static float ComputeKernelFunction(float input);
    static std::vector<BlueNoise::node> SplitPoints(std::vector<BlueNoise::node>& points, Eigen::MatrixXf density);
    static Eigen::MatrixXf ComputeApproximation(std::vector<node> points,Eigen::MatrixXf density);
    static cv::Mat PlotBlueNoise(std::vector<node> points);


};
}

#endif // BLUENOISE_H
