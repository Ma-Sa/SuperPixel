#include "Density.h"


namespace superpixel {






float ComputeArea(Point::point& p) {

//    Eigen::Vector2f g = LocalDepthGradient(depth, p,z_f, 0.5*p.pr);
//    const float gx = g.x();
//    const float gy = g.y();
//    const float scl = 1.0f / std::sqrt(gx*gx + gy*gy + 1.0f);
//    return p.pr*p.pr*M_PI*scl;
    return p.pr * p.pr * M_PI;

}


Eigen::MatrixXf Density::ComputeDensity(std::vector<Point::point>& imagePoints, unsigned int rows, unsigned int cols) {
    Point::point p;
    const float NZ_MIN = 0.174f;
    Eigen::MatrixXf cnt;
    cnt.resize(rows, cols);
    for(size_t ii; ii < imagePoints.size(); ++ii){
        p = imagePoints[ii];
        if(p.valid) {
            cnt(p.px,p.py) = 1 / ComputeArea(p);
            cnt(p.px,p.py) /= std::max(NZ_MIN, std::abs(p.normal.z()));

        }
        else {

            cnt(p.px,p.py) = 0.0f;
        }
    }

    return cnt;
}

cv::Mat Density::PlotDensity(Eigen::MatrixXf density) {

    cv::Mat color = cv::Mat::ones(480,640,CV_8UC3);
    float max = density.maxCoeff();
    for(size_t ii=0; ii < density.cols(); ++ii)
    {
        for(size_t jj=0; jj < density.rows() ; ++jj){
            color.at<cv::Vec3b>(jj,ii)[0] = static_cast<unsigned char> (255.0 * density(jj,ii) / (max));
            color.at<cv::Vec3b>(jj,ii)[1] = static_cast<unsigned char> (255.0 * density(jj,ii) / (max));
            color.at<cv::Vec3b>(jj,ii)[2] = static_cast<unsigned char> (255.0 * density(jj,ii) / (max));


        }
    }

    return color;

}





}

