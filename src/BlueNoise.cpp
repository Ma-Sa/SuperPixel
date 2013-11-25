#include "BlueNoise.h"


namespace superpixel{

BlueNoise::BlueNoise()
{
}

std::vector<unsigned int> RandomScan(Eigen::MatrixXf image){
    std::vector<unsigned int> random(image.size());
    for(size_t ii=0; ii < image.size(); ++ii)
        random[ii] = ii;
    std::random_shuffle(random.begin(),random.end());
    return random;

}

BlueNoise::node CreateNode(Eigen::MatrixXf& mipmap, unsigned int idx, unsigned int p){
    BlueNoise::node node;
    //eigen matrix data are stored columnwise
    node.x = idx % mipmap.rows();
    node.y = idx / mipmap.rows();
    float rho = mipmap.data()[idx];
    node.q = p - (rho < 1 ? 0 : std::ceil(std::log2(rho)/2));
    node.weight = 1 << (2 * (p-node.q));
    node.scale = 1.0f / std::sqrt(rho / node.weight);
    std::cout<<"node information >>>>>>>>"<<node.x <<" "<<node.y<<" "<<node.scale<<" "<<node.weight<<" "<<node.q<<std::endl;

    return node;

}

float ComputeKernelFunction(float input){

    return std::exp(-1 * std::pow(2.0f/3.0f * input,2));

}




Eigen::MatrixXf ComputeApproximation(std::vector<BlueNoise::node> points,Eigen::MatrixXf density){
    Eigen::MatrixXf approximation = Eigen::MatrixXf::Constant(density.rows(), density.cols(), 0.0f);

    for(size_t ii=0; ii<points.size(); ++ii){
    for(size_t x=0; x < approximation.rows(); ++x)
        for(size_t y=0; y < approximation.cols(); ++y){
            if(density(x,y) > 0){
                BlueNoise::node node = points[ii];
                float x_j = node.x;
                float y_j = node.y;
                float weight = node.weight;
                float scale = node.scale;
                float dx = x_j - x;
                float dy = y_j - y;
                float dist = std::sqrt(dx * dx + dy * dy);
                approximation(x,y) += (1.0f / (scale * scale)) * ComputeKernelFunction(dist/scale);
            }
            }

        }

    return approximation;
}


float ComputeEnergy(Eigen::MatrixXf density, std::vector<BlueNoise::node> points){
    std::cout<<"apx "<<ComputeApproximation(points,density)<<std::endl;
    std::cout<<"density "<< density<<std::endl;
    return (ComputeApproximation(points,density)-density).cwiseAbs().sum();
}

float ZeroBorderAccess(const Eigen::MatrixXf& density, int x, int y) {
    if(0 <= x && x < int(density.rows()) && 0 <= y && y < int(density.cols())) {
        return density(x, y);
    }
    else {
        return 0.0f;
    }
}

std::vector<BlueNoise::node> BlueNoise::SplitPoints(std::vector<BlueNoise::node>& points, Eigen::MatrixXf density){
    std::vector<BlueNoise::node> splittedPoints;
    for(size_t ii=0; ii < points.size(); ++ii){
        BlueNoise::node node = points[ii];
        if(node.weight > 1.0f){
            float A = 0.3*0.70710678f;
            float Delta[4][2] = {{-A, -A}, {+A, -A}, {-A, +A}, {+A, +A}};
            for(size_t i=0; i<4; i++){
                BlueNoise::node point = node;
                point.weight = node.weight / (2 * 2);
                point.x = int(2 * node.x + node.scale * Delta[i][0]) ;
                point.y = int(2 * node.y + node.scale * Delta[i][1]) ;
                float roh = ZeroBorderAccess(density, point.x, point.y);
            //    std::cout<<node.x << " "<<node.y<<" "<<point.x<<" "<<point.y<< " "<<density.rows()<<" "<<density.cols()<<std::endl;
              if(roh > 0){
                point.scale = 1.0 / std::sqrt(density(point.x,point.y)/point.weight);
                splittedPoints.push_back(point);

             }
          //      std::cout<<"size of points after splitting >>"<<splittedPoints.size()<<std::endl;

            }

        }
        else {
            BlueNoise::node point = node;
            point.x = 2 * node.x;
            point.y = 2 * node.y;
            point.weight = 1.0f;
            float roh = ZeroBorderAccess(density, point.x, point.y);
            if(roh > 0){
            point.scale = 1.0 / std::sqrt(density(point.x,point.y)/point.weight);
            splittedPoints.push_back(point);
    //        std::cout<<"size of points after no splitting >>"<<splittedPoints.size()<<std::endl;
           }




        }
    }

    return splittedPoints;

}




std::vector<BlueNoise::node> BlueNoise::Compute(Eigen::MatrixXf density){
    std::vector<BlueNoise::node> points;
    std::vector<Eigen::MatrixXf> mipmaps = Pyramid::ComputeMipmaps(density,8);

    int level = int(mipmaps.size()) - 1;
    std::cout<<level<<std::endl;
    for(int p=level; p >= 0; --p){
        if(p==level){
            float currentEnergy = ComputeEnergy(mipmaps[p],points);
            std::cout<<"energy "<<currentEnergy<<" "<<points.size()<<std::endl;

            std::vector<unsigned int> randomScan = RandomScan(mipmaps[p]);
            for(size_t ii=0; ii<randomScan.size(); ++ii){
                unsigned int index = randomScan[ii];
                if (mipmaps[p].data()[index] > 1e-4) {
                    BlueNoise::node y = CreateNode(mipmaps[p],index,p);
                    points.push_back(y);
                    //if it reduces the energy
                    float apxEnergy = ComputeEnergy(mipmaps[p],points);
                    std::cout<<"apx energy "<<apxEnergy<<" "<<std::endl;
                    if( apxEnergy > currentEnergy ){
                        points.pop_back();

                    } else {
                        std::cout<< "point added" <<std::endl;
                        currentEnergy = apxEnergy;
                    }
                }
            }
       }
        else {
            std::cout<<"level" << p << "  splitting: >>"<<points.size()<< " points" << std::endl;
            points = SplitPoints(points,mipmaps[p]);
       }
            std::cout<<"size of points >>"<<points.size()<<std::endl;

      }

    //exit(1);
    return points;
}

cv::Mat BlueNoise::PlotBlueNoise(std::vector<BlueNoise::node> points) {

    cv::Mat color = cv::Mat::ones(480,640,CV_8UC3);
    for(size_t ii=0; ii < points.size(); ++ii)
    {

        BlueNoise::node point = points[ii];
 //       std::cout<<point.x << " ' " << point.y<<std::endl;
        color.at<cv::Vec3b>(point.x, point.y)[0] = static_cast<unsigned char> (255);
        color.at<cv::Vec3b>(point.x, point.y)[1] = static_cast<unsigned char> (0);
        color.at<cv::Vec3b>(point.x, point.y)[2] = static_cast<unsigned char> (0);
    }

    return color;

}




}




