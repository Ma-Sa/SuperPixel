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

BlueNoise::node CreateNode(std::vector<Eigen::MatrixXf> mipmaps, unsigned int idx, unsigned int p){
    BlueNoise::node node;
    Eigen::MatrixXf densityAtP = mipmaps[p];
    node.x = idx / densityAtP.rows();
    node.y = idx % densityAtP.rows();
    node.q = std::ceil(-1 * std::log(densityAtP.data()[idx])/BlueNoise::D);
    node.weight = std::pow(2,BlueNoise::D * (p-node.q));
    Eigen::MatrixXf densityAtQ = mipmaps[node.q];
    node.scale = 1.0f / std::sqrt(densityAtQ.data()[idx] / node.weight);

    return node;

}

float ComputeKernel(){
}


Eigen::MatrixXf ComputeApproximation(std::vector<BlueNoise::node> points){
    Eigen::MatrixXf approximation;
    for(size_t ii=0; ii<points.size(); ++ii){
        BlueNoise::node node = points[ii];
        float x = node.x;
        float y = node.y;
        float weight = node.weight;
        float scale = node.scale;
        float radius = 3 * scale;




    }


}

std::vector<BlueNoise::node> Compute(Eigen::MatrixXf density){
    std::vector<BlueNoise::node> points;
    std::vector<Eigen::MatrixXf> mipmaps = Pyramid::ComputeMipmaps(density,8);

    unsigned int level = mipmaps.size() - 1;

    for(size_t p=level; p>=0; p--){
        std::vector<unsigned int> randomScan = RandomScan(mipmaps[p]);
        for(size_t ii=0; ii<randomScan.size(); ++ii){
            BlueNoise::node y = CreateNode(mipmaps,ii,p);
            //if it reduces the energy
            if(){
                points.push_back(y);
            }
        }


    }

   return points;
}




}



}
