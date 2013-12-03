#ifndef POINT_H
#define POINT_H

#include "Parameters.h"
#include "Camera.h"
#include "cv.h"
#include "highgui.h"

#include <Eigen/Dense>
#include <vector>

namespace superpixel {

class Point{

public:
    Point();

    struct point{
        unsigned int px,py; //pixel position in image
        float pd;
        float pr; //radius of circle around pixel
        Eigen::Vector3d color;
        Eigen::Vector3f position;
        Eigen::Vector3f normal;
        bool valid;

        /** Sets the normal and assures that it points towards the camera (=origin) */
        void setNormal(const Eigen::Vector3f& n) {
            normal = n;
            // force normal to look towards the camera
            // check if point to camera direction and normal are within 90 deg
            // enforce: normal * (cam_pos - pos) > 0
            // do not need to normalize (cam_pos - pos) as only sign is considered
            const float q = normal.dot(-position);
            if(q < 0) {
                normal *= -1.0f;
            }
            else if(q == 0) {
                // this should not happen ...
                normal = Eigen::Vector3f(0,0,-1);
            }
        }

        /** Sets normal from gradient */
        void setNormalFromGradient(const Eigen::Vector2f& g) {
            const float gx = g.x();
            const float gy = g.y();
            const float scl = 1.0f / std::sqrt(gx*gx + gy*gy + 1.0f);
            setNormal(Eigen::Vector3f(scl*gx, scl*gy, -scl));

        }
    };

    template<typename K>
    static inline float LocalFiniteDifferencesKinect(K v0, K v1, K v2, K v3, K v4)
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

    static inline Eigen::Vector2f LocalDepthGradient(cv::Mat& depth, point p, float z_over_f, float window)
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

    static std::vector<point> CreateImagePoints(cv::Mat& depth, float radius, Camera camera){
        std::vector<point> imagePoints;
        imagePoints.reserve(depth.rows*depth.cols);
        point p;
        for(size_t ii=0; ii < depth.rows; ++ii) {
            for(size_t jj=0; jj < depth.cols; ++jj) {
                p.px = ii;
                p.py = jj;
                p.pd = depth.at<unsigned short>(ii,jj) * 0.001f;
                float z_f = p.pd / camera.focal;
                if( depth.at<unsigned short>(ii,jj) != 0 )
                    p.valid = true;
                else
                    p.valid = false;
                if(p.valid) {
                    p.pr = radius *  (camera.focal / p.pd) ;
                    p.position = camera.unprojectImpl(
                        static_cast<float>(p.px), static_cast<float>(p.py),
                        z_f);
                    Eigen::Vector2f gradient = LocalDepthGradient(depth, p, z_f, 0.5f*p.pr);
                    p.setNormalFromGradient(gradient);
                }
                else {
                    p.pr = 0.0f;
                    p.normal = Eigen::Vector3f(0,0,-1);
                }

                imagePoints.push_back(p);

            }
        }

        return imagePoints;

    }


};

}



#endif // POINT_H
