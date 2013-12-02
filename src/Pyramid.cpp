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


Eigen::MatrixXf Pyramid::ImagePadding(Eigen::MatrixXf &img_big){
    //unsigned int rows = density.rows();
    //unsigned int cols = density.cols();

    //unsigned int max = std::max(rows,cols);
    //unsigned int paddingSize = max;
    //if(!IsPowerOfTwo(max))
      //  paddingSize = ClosestPowerOfTwo(max);
    //Eigen::MatrixXf bigImg(paddingSize,paddingSize);
    //bigImg.fill(0.0f);

    size_t w_big = img_big.rows();
    size_t h_big = img_big.cols();
    // the computed mipmap will have 2^i size
    unsigned int size = ClosestPowerOfTwo(std::max(w_big, h_big));
    Eigen::MatrixXf img_small(size / 2, size / 2);
    img_small.fill(0.0f);
    // only the part where at least one of the four pixels lies in the big image is iterated
    // the rest was set to 0 with the fill op
    size_t w_small = w_big / 2 + ((w_big % 2 == 0) ? 0 : 1);
    size_t h_small = h_big / 2 + ((h_big % 2 == 0) ? 0 : 1);
    for(size_t y = 0; y < h_small; y++) {
        size_t y_big = y * 2;
        for(size_t x = 0; x < w_small; x++) {
            size_t x_big = x * 2;
            // We sum over all four pixels in the big image (if they are valid).
            // May by invalid because the big image is considered to be enlarged
            // to have a size of 2^i.
            float sum = 0.0f;
            // Since we only test the part where at least one pixel is in also in the big image
            // we do not need to test that (x_big,y_big) is a valid pixel in the big image.
            const float* p_big = &img_big(x_big, y_big);
            sum += *(p_big);
            if(x_big + 1 < w_big) {
                sum += *(p_big + 1);
            }
            if(y_big + 1 < h_big) {
                sum += *(p_big + w_big);
                if(x_big + 1 < w_big) {
                    sum += *(p_big + w_big + 1);
                }
            }
            img_small(x, y) = sum;
        }
    }
    return img_small;


}

Eigen::MatrixXf Pyramid::CreateSmallerImage(Eigen::MatrixXf& img_big){
    // size of original image
    const unsigned int w_big = img_big.rows();
    const unsigned int h_big = img_big.cols();
    // size of reduced image
    const unsigned int w_sma = w_big / 2;
    const unsigned int h_sma = h_big / 2;
    // the computed mipmap will have 2^i size

    Eigen::MatrixXf img_small(w_sma, h_sma);
    for(unsigned int y=0; y<h_sma; ++y) {
        const unsigned int y_big = 2*y;
        for(unsigned int x=0; x<w_sma; ++x) {
            const unsigned int x_big = 2*x;
            float sum = 0.0f;
            for(unsigned int i=0; i<2; ++i) {
                for(unsigned int j=0; j<2; ++j) {
                    sum += img_big(x_big+j, y_big+i);
                }
            }
            img_small(x, y) = sum;
        }
    }
    return img_small;
//    unsigned int bigWidth = bigImage.cols();
//    unsigned int bigHeight = bigImage.rows();

//    unsigned int smallWidth = bigWidth / 2;
//    unsigned int smallHeight = bigHeight / 2;

//    Eigen::MatrixXf smallImage(smallHeight,smallWidth);
//    smallImage.fill(0.0f);
//    unsigned int x,y;

//    for(size_t jj=0; jj < smallHeight; ++jj){
//    for(size_t ii=0; ii < smallWidth; ++ii){
//            x = jj*2;
//            y = ii*2;

//            smallImage(jj,ii) = bigImage(x,y) + bigImage(x+1,y) + bigImage(x,y+1) + bigImage(x+1,y+1);
//         //   smallImage(jj,ii) /= 4.0;

//        }

//    }

//    return smallImage;

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



std::vector<Eigen::MatrixXf> Pyramid::ComputeMipmaps(Eigen::MatrixXf& image, unsigned int smallest){


    unsigned int maxMipmap = PowerOfTwoExponent(std::max(image.rows(),image.cols()));
    unsigned int minMipmap = PowerOfTwoExponent(smallest);
    unsigned int numberOfMipmaps = maxMipmap - minMipmap;
    std::cout<<numberOfMipmaps<<std::endl;
    std::cout<<maxMipmap<<" , "<<minMipmap << std::endl;
    std::vector<Eigen::MatrixXf> mipmaps(numberOfMipmaps);
    mipmaps[0] = ImagePadding(image);


    for(size_t ii=1; ii<numberOfMipmaps; ++ii){
        mipmaps[ii] = CreateSmallerImage(mipmaps[ii-1]);    
    }


    return mipmaps;

}


}
