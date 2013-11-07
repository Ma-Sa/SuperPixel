#ifndef CAMERA_H
#define CAMERA_H

namespace superpixel{
struct Camera
{
    float cx, cy;
    float focal;
    float z_slope;


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

    /** Gets kinect depth for a 3D point */
    uint16_t depth(const Eigen::Vector3f& p) const {
        return convertMeterToKinect(p[2]);
    }

    /** Convert kinect depth to meter */
    float convertKinectToMeter(uint16_t d) const {
        return static_cast<float>(d) * z_slope;
    }

    float convertKinectToMeter(int d) const {
        return static_cast<float>(d) * z_slope;
    }

    float convertKinectToMeter(float d) const {
        return d * z_slope;
    }

    /** Convert meter to kinect depth */
    uint16_t convertMeterToKinect(float z) const {
        return static_cast<uint16_t>(z / z_slope);
    }


};

}


#endif // CAMERA_H
