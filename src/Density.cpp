#include "Density.h"


namespace superpixel {

template<typename K>
inline float LocalFiniteDifferencesKinect(K v0, K v1, K v2, K v3, K v4)
{
    if(v0 == 0 && v4 == 0 && v1 != 0 && v3 != 0) {
        return float(v3 - v1);
    }

    bool left_invalid = (v0 == 0 || v1 == 0);
    bool right_invalid = (v3 == 0 || v4 == 0);
    if(left_invalid && right_invalid) {
        return 0.0f;
    }
    else if(left_invalid) {
        return float(v4 - v2);
    }
    else if(right_invalid) {
        return float(v2 - v0);
    }
    else {
        float a = static_cast<float>(std::abs(v2 + v0 - static_cast<K>(2)*v1));
        float b = static_cast<float>(std::abs(v4 + v2 - static_cast<K>(2)*v3));
        float p, q;
        if(a + b == 0.0f) {
            p = q = 0.5f;
        }
        else {
            p = a / (a + b);
            q = b / (a + b);
        }
        return q * static_cast<float>(v2 - v0) + p * static_cast<float>(v4 - v2);
    }
}

inline Eigen::Vector2f LocalDepthGradient(cv::Mat& depth, Point::point p, float z_over_f, float window)
{
    // compute w = base_scale*f/d
    unsigned int w = std::max(static_cast<unsigned int>(window + 0.5f), 4u);
    if(w % 2 == 1) w++;

    // can not compute the gradient at the border, so return 0
    if(p.px < w || depth.rows - w <= p.px || p.py < w || depth.cols - w <= p.py) {
        return Eigen::Vector2f::Zero();
    }

    float dx = 0.001f * LocalFiniteDifferencesKinect<int>(
        depth.at<unsigned short>(p.px-w,p.py),
        depth.at<unsigned short>(p.px-w/2,p.py),
        depth.at<unsigned short>(p.px,p.py),
        depth.at<unsigned short>(p.px+w/2,p.py),
        depth.at<unsigned short>(p.px+w,p.py)
    );

    float dy = 0.001f * LocalFiniteDifferencesKinect<int>(
        depth.at<unsigned short>(p.px,p.py-w),
        depth.at<unsigned short>(p.px,p.py-w/2),
        depth.at<unsigned short>(p.px,p.py),
        depth.at<unsigned short>(p.px,p.py+w/2),
        depth.at<unsigned short>(p.px,p.py+w)
    );


    float scl = 1.0f / (float(w) * z_over_f);
    return scl * Eigen::Vector2f(dx,dy);
}





float ComputeArea(cv::Mat depth, Point::point p, float z_f) {

//    Eigen::Vector2f g = LocalDepthGradient(depth, p,z_f, 0.5*p.pr);
//    const float gx = g.x();
//    const float gy = g.y();
//    const float scl = 1.0f / std::sqrt(gx*gx + gy*gy + 1.0f);
//    return p.pr*p.pr*M_PI*scl;
    return p.pr * p.pr * M_PI;

}


Eigen::MatrixXf Density::ComputeDensity(cv::Mat& depth, float radius) {
    Point::point p;
    float z_f;
    Camera camera;
    camera.cx = 235.79f;
    camera.cy = 314.16f;
    camera.focal = 535.8f;
    camera.z_slope = 0.001f;
    const float NZ_MIN = 0.174f;
    Eigen::MatrixXf cnt;
    cnt.resize((depth.rows), (depth.cols));
    for(size_t ii=0; ii < depth.rows ; ++ii)
        for(size_t jj=0; jj < depth.cols ; ++jj){
            //p = Point::Create3dPoint(depth, ii, jj);
            p.px = ii;
            p.py = jj;
            p.pd = depth.at<unsigned short>(ii,jj) * 0.001f;
            z_f = p.pd / camera.focal;
            if( depth.at<unsigned short>(ii,jj) != 0 )
                p.valid = true;
            else
                p.valid = false;
            if(p.valid) {
                p.pr = radius * (camera.focal / p.pd) ;
                p.position = camera.unprojectImpl(
                    static_cast<float>(p.px), static_cast<float>(p.py),
                    z_f);
                Eigen::Vector2f gradient = LocalDepthGradient(depth, p, z_f, 0.5f*p.pr);
                p.setNormalFromGradient(gradient);
                cnt(ii,jj) = 1 / ComputeArea(depth,p,z_f);
                cnt(ii,jj) /= std::max(NZ_MIN, std::abs(p.normal.z()));

            }
            else {
                p.pr = 0.0f;
                p.normal = Eigen::Vector3f(0,0,-1);
                cnt(ii,jj) = 0.0f;
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

