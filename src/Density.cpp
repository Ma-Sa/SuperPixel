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

inline Eigen::Vector2f LocalDepthGradient(cv::Mat depth, Point::point p, float z_over_f, float window)
{
    // compute w = base_scale*f/d
    unsigned int w = std::max(static_cast<unsigned int>(window + 0.5f), 4u);
    if(w % 2 == 1) w++;

    // can not compute the gradient at the border, so return 0
    if(p.py < w || depth.rows - w <= p.py || p.px < w || depth.cols - w <= p.px) {
        return Eigen::Vector2f::Zero();
    }

    float dx = LocalFiniteDifferencesKinect<int>(
        depth.at<unsigned short>(p.px-w,p.py),
        depth.at<unsigned short>(p.px-w/2,p.py),
        depth.at<unsigned short>(p.px,p.py),
        depth.at<unsigned short>(p.px+w/2,p.py),
        depth.at<unsigned short>(p.px+w,p.py)
    );

    float dy = LocalFiniteDifferencesKinect<int>(
        depth.at<unsigned short>(p.px,p.py-w),
        depth.at<unsigned short>(p.px,p.py-w/2),
        depth.at<unsigned short>(p.px,p.py),
        depth.at<unsigned short>(p.px,p.py+w/2),
        depth.at<unsigned short>(p.px,p.py+w)
    );


    float scl = 1.0f / (float(w) * z_over_f);

    return scl * Eigen::Vector2f(dx,dy);
}





float FastInverseSqrt(float x) {
    uint32_t i = *((uint32_t *)&x);			// evil floating point bit level hacking
    i = 0x5f3759df - (i >> 1);				// use a magic number
    float s = *((float *)&i);				// get back guess
    return s * (1.5f - 0.5f * x * s * s);	// one newton iteration
}


float ComputeArea(cv::Mat depth, Point::point p, float z_f) {
    const float pi = 3.14;
    Eigen::Vector2f g = LocalDepthGradient(depth, p,z_f, 0.5*p.pr);
    const float gx = g.x();
    const float gy = g.y();
    const float scl = FastInverseSqrt(gx*gx + gy*gy + 1.0f);
    return p.pr*p.pr*pi*scl;

}



vector<vector<float> > Density::ComputeDensity(cv::Mat depth, unsigned int Radius) {
    Point::point p;
    float z_f;
    Camera camera;
    camera.cx = 318.39f;
    camera.cy = 271.99f;
    camera.focal = 528.01f;
    camera.z_slope = 0.001f;
    vector<vector<float> > cnt;
    cnt.resize((depth.rows), vector<float>(depth.cols,0));
    for(size_t ii=0; ii < depth.rows ; ++ii)
        for(size_t jj=0; jj < depth.cols ; ++jj){
            p = Point::Create3dPoint(depth, ii, jj);
            z_f = p.pd / camera.focal;
            if( p.pd == 0 )
                p.valid = false;
            if(p.valid) {
                p.pr = Radius * (camera.focal / p.pd) ;
                p.position = camera.unprojectImpl(
                    static_cast<float>(p.px), static_cast<float>(p.py),
                    z_f);
                cnt[ii][jj] = 1.0f/ComputeArea(depth,p,z_f);
            }

        }

    return cnt;
}

cv::Mat Density::PlotDensity(vector<vector<float> > density) {
    cv::Mat color = cv::Mat::ones(480,640,CV_8UC3);
    cout<<density[0].size() <<" * "<< density.size() <<endl;
    for(size_t ii=0; ii < density[0].size(); ++ii)
    {
        for(size_t jj=0; jj < density.size() ; ++jj){
            color.at<cv::Vec3b>(jj,ii)[0] = static_cast<unsigned char> (255.0 * density[jj][ii]);
            color.at<cv::Vec3b>(jj,ii)[1] = static_cast<unsigned char> (255.0 * density[jj][ii]);
            color.at<cv::Vec3b>(jj,ii)[2] = static_cast<unsigned char> (255.0 * density[jj][ii]);


        }
    }

    return color;

}





}

