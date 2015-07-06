#ifndef APPROACHCOMPONENTS_H
#define APPROACHCOMPONENTS_H

#include <Eigen/Dense>
#include "frame.h"

namespace ApproachComponents{

static void computePoseNeighbours(vector< std::shared_ptr<Frame> >& frames, int knn){
    //compute closest points
    for(int src_id=0; src_id<frames.size(); src_id++){
        Frame& srcCloud = *frames[src_id];
        srcCloud.computePoseNeighboursKnn(&frames,src_id,knn);
//        srcCloud.computeClosestPointsToNeighbours(&frames,cutoff);
    }
}

static void computeClosestPoints(vector< std::shared_ptr<Frame> >& frames, float cutoff){
    //compute closest points
    for(int src_id=0; src_id<frames.size(); src_id++){
        Frame& srcCloud = *frames[src_id];
//        srcCloud.computePoseNeighboursKnn(&frames,src_id,knn);
        srcCloud.computeClosestPointsToNeighbours(&frames,cutoff);
    }
}

void g2oOptimizer(std::vector< std::shared_ptr<Frame> >& frames, bool pointToPlane);
void ceresOptimizer(std::vector< std::shared_ptr<Frame> >& frames, bool pointToPlane);


}//end ns

#endif // APPROACHCOMPONENTS_H

