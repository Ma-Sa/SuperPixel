#include "BlueNoise.h"


namespace superpixel{

BlueNoise::BlueNoise()
{
}

const float KernelRange = 2.5f;

const float cMaxRefinementScale = 10.0f;

const float cPi = 3.141592654f;

const unsigned MAX_DEPTH = 0;
const unsigned LANGEVIN_STEPS = 20;
const bool MH_TEST = false;
const float STEPSIZE = 0.3f;
const float TEMPERATURE = 0.03f;

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
    node.x = float(idx % mipmap.rows());
    node.y = float(idx / mipmap.rows());
    float rho = mipmap(node.x,node.y);

    node.q = p - (rho < 1 ? 0 : std::ceil(std::log2(rho)/2));
    node.weight = float(1 << (2 * (p-node.q)));
    node.scale = 1.0f / std::sqrt(rho / node.weight);
    //std::cout<<"node information >>>>>>>>"<<idx <<" " <<mipmap.data()[idx]<<" "<< node.x <<" "<<node.y<<" "<<rho<<" "<<node.scale<<" "<<node.weight<<" "<<node.q<<std::endl;
    //exit(1);
    return node;


}

float ComputeKernelFunction(float input){

    return std::exp(-1 * std::pow(2.0f/3.0f * input,2));

}




Eigen::MatrixXf ComputeApproximation(std::vector<BlueNoise::node> points,Eigen::MatrixXf density){
    Eigen::MatrixXf approximation = Eigen::MatrixXf::Constant(density.rows(), density.cols(), 0.0f);

    for(size_t ii=0; ii<points.size(); ++ii){
        BlueNoise::node node = points[ii];
        float x_j = node.x;
        float y_j = node.y;
        float weight = node.weight;
        float scale = node.scale;
        const float cMaxRange = 1.482837414f; // eps = 0.001
        float radius = cMaxRange * scale;
        float x_min = std::max(0, int(std::floor(x_j - radius)));
        float x_max = std::min(int(approximation.rows()) - 1, int(std::ceil(x_j + radius)));
        float y_min = std::max(0, int(std::floor(y_j - radius)));
        float y_max = std::min(int(approximation.cols()) - 1, int(std::ceil(y_j + radius)));

        for(unsigned int y=y_min; y<=y_max; y++) {
            for(unsigned int x=x_min; x<=x_max; x++) {
                float ux = float(x);
                float uy = float(y);
                float dx = ux - x_j;
                float dy = uy - y_j;


                float dist = std::sqrt(dx * dx + dy * dy);
                approximation(x,y) += (weight / (scale * scale)) * ComputeKernelFunction(dist/scale);
            }
            }

        }

    return approximation;
}


float ComputeEnergy(Eigen::MatrixXf density, std::vector<BlueNoise::node> points){
   // std::cout<<"apx "<<ComputeApproximation(points,density)<<std::endl;
   // std::cout<<"density "<< density<<std::endl;
    return (ComputeApproximation(points,density)-density).cwiseAbs().sum();
}

float EnergyApproximation(const std::vector<BlueNoise::node>& pnts, float x, float y)
{
    float sum = 0.0f;
    for(size_t ii=0; ii<pnts.size(); ++ii) {
        BlueNoise::node p = pnts[ii];
        float dx = p.x - x;
        float dy = p.y - y;
        float d = dx*dx + dy*dy;
        float scl = p.scale * p.scale;
        if(d < KernelRange * KernelRange * scl) {
            float dist = std::sqrt(dx * dx + dy * dy);
            sum += p.weight / scl * ComputeKernelFunction(std::sqrt(dist) / p.scale);
        }
    }
    return sum;
}

float EnergyDerivative(const std::vector<BlueNoise::node>& pnts, const Eigen::MatrixXf& density, unsigned int i, float& result_dE_x, float& result_dE_y)
{
    float dE_x = 0.0f;
    float dE_y = 0.0f;

    const BlueNoise::node& pi = pnts[i];
    float px = pi.x;
    float py = pi.y;
    float ps = pi.scale;
    float ps_scl = 1.0f / (ps * ps);
    // find window (points outside the window do not affect the kernel)
    // range = sqrt(ln(1/eps)/pi)
    const float cMaxRange = 1.482837414f; // eps = 0.001
    float radius = cMaxRange * ps;
    float x_min = std::max(0, int(std::floor(px - radius)));
    float x_max = std::min(int(density.rows()) - 1, int(std::ceil(px + radius)));
    float y_min = std::max(0, int(std::floor(py - radius)));
    float y_max = std::min(int(density.cols()) - 1, int(std::ceil(py + radius)));
    // sum over window
    for(unsigned int y=y_min; y<=y_max; y++) {
        for(unsigned int x=x_min; x<=x_max; x++) {
            float ux = float(x);
            float uy = float(y);
            float dx = ux - px;
            float dy = uy - py;
//			float k_arg = std::sqrt(dx*dx + dy*dy) * ps_scl;
//			float k_val = KernelFunctor(k_arg);
            float dist = std::sqrt(dx * dx + dy * dy);
            float k_val = ps_scl * ComputeKernelFunction(dist/ps);
            //float k_arg_square = (dx*dx + dy*dy) * ps_scl;
            //float k_val = KernelFunctorSquare(k_arg_square);
            float apx = EnergyApproximation(pnts, ux, uy);
            float roh = density(x, y);
            if(apx < roh) {
                k_val = -k_val;
            }
            //k_val = 0.0f;
            dE_x += k_val * dx;
            dE_y += k_val * dy;
        }
    }
    float A = 2.0f * cPi / std::pow(ps, float(2 + 1));
    result_dE_x = A * dE_x;
    result_dE_y = A * dE_y;
    return radius;
}

float ZeroBorderAccess(const Eigen::MatrixXf& density, int x, int y) {
    if(0 <= x && x < int(density.rows()) && 0 <= y && y < int(density.cols())) {
        return density(x, y);
    }
    else {
        return 0.0f;
    }
}


void Refine(std::vector<BlueNoise::node>& points, const Eigen::MatrixXf& density, unsigned int iterations)
{
    static boost::mt19937 rng;
    static boost::normal_distribution<float> rnd(0.0f, 1.0f); // standard normal distribution
    static boost::variate_generator<boost::mt19937&, boost::normal_distribution<float> > die(rng, rnd);

    for(unsigned int k=0; k<iterations; k++) {
        float energy;
        if(MH_TEST) {
            energy = ComputeEnergy(density, points);
        }

#ifdef VERBOSE
        if(!MH_TEST) {
            energy = ComputeEnergy(density, points);
        }
        std::cout << "\tit=" << k << ", e=" << energy << std::endl;
#endif

        for(unsigned int i=0; i<points.size(); i++) {
            // random vector
            float rndx = die();
            float rndy = die();
            // compute next position
            BlueNoise::node p = points[i];
            if(p.scale > cMaxRefinementScale) {
                // omit low frequency kernels
                continue;
            }
            // dx = -STEPSIZE*s/2*dE + sqrt(TEMPERATURE*s*STEPSIZE)*rnd()
            float c0 = STEPSIZE * p.scale;
            float cA = c0 * 0.5f;
            float dx, dy;
            float R = EnergyDerivative(points, density, i, dx, dy);
//			std::cout << i << ": (" << dx << "," << dy << "), s=" << p.scale << std::endl;
//			r_min = std::min(R, r_min);
//			r_max = std::max(R, r_max);
//			std::cout << i << " p1: (" << p.x << "," << p.y << ")" << std::endl;
            p.x -= cA * dx;
            p.y -= cA * dy;
            float cB = std::sqrt(TEMPERATURE * c0);
            p.x += cB * rndx;
            p.y += cB * rndy;
            float roh = ZeroBorderAccess(density, p.x, p.y);
            if(roh > 0) {
                p.scale = 1.0 / std::sqrt(roh/p.weight);
            }
            else {
                // reject
                continue;
            }
//			std::cout << i << " p2: (" << p.x << "," << p.y << ")" << std::endl;
            // check if we want to keep the point
            points[i] = p;
            if(MH_TEST) {
                BlueNoise::node pold = p;
                float dxn, dyn;
                EnergyDerivative(points, density, i, dxn, dyn);
                float h = cB / (2.0f*TEMPERATURE);
                float hx = h*(dx + dxn) + rndx;
                float hy = h*(dy + dyn) + rndy;
                float g1 = -0.5f*(hx*hx + hy*hy);
                float g2 = -0.5f*(rndx*rndx + rndy*rndy);
                float energy_new = ComputeEnergy(density, points);
                float P = std::exp((energy-energy_new)/TEMPERATURE + g1 - g2);
//				std::cout << energy << " -> " << energy_new << ", P=" << P << std::endl;
                if(die() <= P) {
                    // accept
                    energy = energy_new;
                }
                else {
                    // reject
                    points[i] = pold;
                }
            }
        }
    }
//	std::cout << r_min << " " << r_max << std::endl;
}




std::vector<BlueNoise::node> BlueNoise::SplitPoints(std::vector<BlueNoise::node>& points, Eigen::MatrixXf density){
    std::vector<BlueNoise::node> splittedPoints;
    for(size_t ii=0; ii < points.size(); ++ii){
        BlueNoise::node node = points[ii];

        if(node.weight > 1.0f){
            //node.x *= 2.0f;
            //node.y *= 2.0f;
            //node.weight /= 4.0f;
            float A = 0.3*0.70710678f;
            float Delta[4][2] = {{-A, -A}, {+A, -A}, {-A, +A}, {+A, +A}};
            for(size_t i=0; i<4; i++){
                BlueNoise::node point = node;
                point.x = node.x * 2;
                point.y = node.y * 2;
                point.weight = node.weight / 4;
                point.x += node.scale * Delta[i][0] ;
                point.y += node.scale * Delta[i][1] ;
                float roh = ZeroBorderAccess(density, point.x, point.y);
               // std::cout<<node.x << " "<<node.y<<" "<<point.x<<" "<<point.y<< " "<<density.rows()<<" "<<density.cols()<<std::endl;
              if(roh > 0){
                point.scale = 1.0 / std::sqrt(roh/point.weight);
                splittedPoints.push_back(point);

             }
          //      std::cout<<"size of points after splitting >>"<<splittedPoints.size()<<std::endl;

            }

        }
        else {
            BlueNoise::node point = node;
            point.x = 2.0f * node.x;
            point.y = 2.0f * node.y;
            point.weight = 1.0f;
            float roh = ZeroBorderAccess(density, point.x, point.y);
            if(roh > 0){
            point.scale = 1.0 / std::sqrt(roh/point.weight);
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
    for(int p=level; p >= 0; --p){
        if(p==level){

            float currentEnergy = ComputeEnergy(mipmaps[p],points);

            std::vector<unsigned int> randomScan = RandomScan(mipmaps[p]);
            for(size_t ii=0; ii<randomScan.size(); ++ii){
                unsigned int index = randomScan[ii];
                if (mipmaps[p].data()[index] > 1e-4) {

                    BlueNoise::node y = CreateNode(mipmaps[p],index,p);
                    points.push_back(y);
                    //if it reduces the energy
                    float apxEnergy = ComputeEnergy(mipmaps[p],points);
                    //std::cout<<"apx energy "<<apxEnergy<<" "<<std::endl;
                    if( apxEnergy > currentEnergy ){
                        points.pop_back();

                    } else {
                        //std::cout<< "point added" <<std::endl;
                        currentEnergy = apxEnergy;
                    }
                }
            }
       }
        else {

            //std::cout<<"level" << p << "  splitting: >>"<<points.size()<< " points" << std::endl;
            points = SplitPoints(points,mipmaps[p]);
            Refine(points,mipmaps[p],LANGEVIN_STEPS);
       }


         //   std::cout<<"size of points >>"<<points.size()<<std::endl;

      }


    return points;
}

cv::Mat BlueNoise::PlotBlueNoise(std::vector<BlueNoise::node> points) {

    cv::Mat color = cv::Mat::ones(480,640,CV_8UC3);
    for(size_t ii=0; ii < points.size(); ++ii)
    {

        BlueNoise::node point = points[ii];
        //std::cout<<point.x << " ' " << point.y<<std::endl;
        color.at<cv::Vec3b>(int(point.x)*2.0, int(point.y)*2.0)[0] = static_cast<unsigned char> (255);
        color.at<cv::Vec3b>(int(point.x)*2.0, int(point.y)*2.0)[1] = static_cast<unsigned char> (0);
        color.at<cv::Vec3b>(int(point.x)*2.0, int(point.y)*2.0)[2] = static_cast<unsigned char> (0);
    }
    return color;

}




}




