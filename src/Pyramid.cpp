#include "Pyramid.h"
#include <math.h>

namespace superpixel{

Pyramid::Pyramid()
{
}


bool IsPowerOfTwo (unsigned int x)
{
    return ((x != 0) && !(x & (x - 1)));
}


unsigned int ClosestPowerOfTwo (unsigned int x){

    unsigned int  cnt = std::ceil(std::log2(x));
    unsigned int power = 1;
    for(size_t i = 0; i < cnt; i++)
        power = power << 1;
    return power;
}

unsigned int PowerOfTwoExponent(unsigned int x){
    return std::ceil(std::log2(x));
}


Eigen::MatrixXf Pyramid::ImagePadding(Eigen::MatrixXf density){
    unsigned int rows = density.rows();
    unsigned int cols = density.cols();

    unsigned int max = std::max(rows,cols);
    unsigned int paddingSize = max;
    if(!IsPowerOfTwo(max))
        paddingSize = ClosestPowerOfTwo(max);
    Eigen::MatrixXf bigImg(paddingSize,paddingSize);
    bigImg.fill(0.0f);
    for(size_t ii=0; ii<density.rows(); ++ii)
        for(size_t jj=0; jj<density.cols(); ++jj){
            bigImg(ii,jj) = density(ii,jj);
        }
    return bigImg;

}

Eigen::MatrixXf Pyramid::CreateSmallerImage(Eigen::MatrixXf bigImage){
    unsigned int bigWidth = bigImage.cols();
    unsigned int bigHeight = bigImage.rows();

    unsigned int smallWidth = bigWidth / 2;
    unsigned int smallHeight = bigHeight / 2;

    Eigen::MatrixXf smallImage(smallHeight,smallWidth);
    smallImage.fill(0.0f);
    unsigned int x,y;

    for(size_t jj=0; jj < smallHeight; ++jj){
    for(size_t ii=0; ii < smallWidth; ++ii){
            x = jj*2;
            y = ii*2;

            smallImage(jj,ii) = bigImage(x,y) + bigImage(x+1,y) + bigImage(x,y+1) + bigImage(x+1,y+1);
         //   smallImage(jj,ii) /= 4.0;

        }

    }

    return smallImage;

}

Eigen::MatrixXf Pyramid::CreateBiggerImage(Eigen::MatrixXf smallImage){
    unsigned int smallWidth = smallImage.cols();
    unsigned int smallHeight = smallImage.rows();

    unsigned int bigWidth = smallWidth * 2;
    unsigned int bigHeight = smallHeight * 2;

    Eigen::MatrixXf bigImage(bigHeight,bigWidth);
    unsigned int x,y;

    for(size_t ii=0; ii < smallWidth; ++ii){
        y = ii * 2;
        for(size_t jj=0; jj < smallHeight; ++jj){
            x = jj * 2;
            float val = smallImage(jj,ii);
            bigImage(x,y) = val;
            bigImage(x+1,y) = val;
            bigImage(x,y+1) = val;
            bigImage(x+1,y+1) = val;

        }
    }

    return bigImage;


}



std::vector<Eigen::MatrixXf> Pyramid::ComputeMipmaps(Eigen::MatrixXf image, unsigned int smallest){

    unsigned int maxMipmap = PowerOfTwoExponent(std::max(image.rows(),image.cols()));
    unsigned int minMipmap = PowerOfTwoExponent(smallest);
    unsigned int numberOfMipmaps = maxMipmap - minMipmap + 1;
    std::cout<<numberOfMipmaps<<std::endl;
    std::vector<Eigen::MatrixXf> mipmaps(numberOfMipmaps);
    mipmaps[0] = ImagePadding(image);

    for(size_t ii=1; ii<numberOfMipmaps; ++ii){
        mipmaps[ii] = CreateSmallerImage(mipmaps[ii-1]);    
    }
    return mipmaps;

}


}
