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
    Eigen::MatrixXf ImagePadding(Eigen::MatrixXf density);
    Eigen::MatrixXf CreateSmallerImage(Eigen::MatrixXf bigImage);
    Eigen::MatrixXf CreateBiggerImage(Eigen::MatrixXf smallImage);
    static std::vector<Eigen::MatrixXf> ComputeMipmaps(Eigen::MatrixXf image, unsigned int smallest);

};

}

#endif // PYRAMID_H
