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
        unsigned int x,y,q;
        float scale, weight;
    };

    static const unsigned int D = 2;
    static std::vector<BlueNoise::node> Compute(Eigen::MatrixXf density);
    float ComputeKernel();
    node CreateNode(std::vector<Eigen::MatrixXf> mipmaps, unsigned int idx, unsigned int p);
    Eigen::MatrixXf ComputeApproximation(std::vector<node> points);






};
}

#endif // BLUENOISE_H
