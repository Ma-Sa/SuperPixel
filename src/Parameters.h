#ifndef PRAMETERS_H
#define PRAMETERS_H

namespace superpixel {

struct parameters {
    parameters();
    unsigned int iteration;     //number of iteration for k-means clustering
    float radius;   //radius of the disc
    unsigned int count;     //number of superpixels
    float search;   //search area for superpixel
};



}






#endif // PRAMETERS_H
