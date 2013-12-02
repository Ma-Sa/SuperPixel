#ifndef PYRAMID_H
#define PYRAMID_H

#include <Eigen/Eigen>
#include <iostream>
#include <vector>

namespace superpixel{

class Pyramid
{
public:
    Pyramid();
    static Eigen::MatrixXf ImagePadding(Eigen::MatrixXf& img_big);
    static Eigen::MatrixXf CreateSmallerImage(Eigen::MatrixXf &img_big);
    static Eigen::MatrixXf CreateBiggerImage(Eigen::MatrixXf smallImage);
    static std::vector<Eigen::MatrixXf> ComputeMipmaps(Eigen::MatrixXf &image, unsigned int smallest);

};

}

#endif // PYRAMID_H
