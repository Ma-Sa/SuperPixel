#ifndef CAMERA_H
#define CAMERA_H

#include <Eigen/Eigen>

namespace superpixel{
struct Camera
{
    float cx, cy;
    float focal;


    /** Projects a 3D point into the image plane */
    Eigen::Vector2f project(const Eigen::Vector3f& p) const {
        return Eigen::Vector2f(p[0] / p[2] * focal + cx, p[1] / p[2] * focal + cy);
    }

    /** Computes a 3D point from pixel position and z/focal */
    Eigen::Vector3f unprojectImpl(float px, float py, float z_over_f) const {
        return z_over_f * Eigen::Vector3f(px - cx, py - cy, focal);
    }

    /** Computes a 3D point from pixel position and depth */
    Eigen::Vector3f unproject(int x, int y, uint16_t depth) const {
        return unprojectImpl(
            static_cast<float>(x), static_cast<float>(y),
            convertKinectToMeter(depth) / focal
        );
    }

    float convertKinectToMeter(float d) const {
        return d * 0.001f;
    }




};

}


#endif // CAMERA_H
