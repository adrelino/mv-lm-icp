/** @author Adrian Haarbach
 *
 * Example of multiview LM-ICP.
 * variants:
 * - point to point
 * - point to plane
 * minimizers:
 * - g2o with SO3 vertices and GICP edges
 * - ceres with angle axis
 * - ceres with Eigen Quaternion
 * - ceres with SophusSE3
 * options:
 * - recompute normals using PCA
 * - dmax cutoff distance
 * - knn number of clostest frames
 */


#include "Visualize.h"
#include "common.h"
#include "gflags/gflags.h"
#include "CPUTimer.h"
#include "frame.h"

#include "icp-g2o.h"
#include "icp-ceres.h"

using namespace std;

DEFINE_bool(pointToPlane, true, "use point to plane distance metric");
DEFINE_bool(sophusSE3,true,""); DEFINE_bool(sophusSE3_autodiff,false,"");
DEFINE_bool(angleAxis, false, "");


DEFINE_bool(g2o, false, "use g2o");
DEFINE_double(cutoff,0.05,"dmax/cutoff distance after which we prune correspondences"); //dmax
DEFINE_int32(knn,2,"number of knn nearest neigbhours to build up the graph");

//DEFINE_string(dir,"../samples/dinosaur","dir");
DEFINE_string(dir,"../samples/Bunny_RealData","dir");

DEFINE_double(sigma,0.02,"rotation noise variance");
DEFINE_double(sigmat,0.01,"translation noise variance");

DEFINE_bool(fake,false,"weather to load the first frame repeteadly, useful for testing");
DEFINE_int32(limit,40,"limit");
DEFINE_int32(step,2,"step");

DEFINE_bool(recomputeNormals,true,"weather to recompute normals using PCA of 10 neighbours");

DEFINE_bool(robust,true,"robust loss function. Currently uses the SoftL1Loss with scaling parameter set to 1.5*median of point correspondance distances");

static void loadFrames(vector< std::shared_ptr<Frame> >& frames, std::string dir, bool demean){
    vector<string> clouds = getAllTextFilesFromFolder(dir,"cloud");
    vector<string> poses = getAllTextFilesFromFolder(dir,"pose");
    vector<string> groundtruth = getAllTextFilesFromFolder(dir,"groundtruth");

    if(clouds.size() != poses.size()){// || clouds.size() != groundtruth.size()){
        cout<<"unequal size"<<endl;
    }


   for (int i = 0; i < clouds.size() && i<FLAGS_limit*FLAGS_step; i+=FLAGS_step) {
        shared_ptr<Frame> f(new Frame());
        int j=i;
        if(FLAGS_fake) j=0;
        loadXYZ(clouds[j],f->pts,f->nor);
        if(FLAGS_recomputeNormals){
            f->recomputeNormals();
        }



        if(groundtruth.size()==clouds.size()){
            f->pose=Isometry3d(loadMatrix4d(poses[i]));
            f->poseGroundTruth=Isometry3d(loadMatrix4d(groundtruth[i]));
        }else{
            f->poseGroundTruth=Isometry3d(loadMatrix4d(poses[i]));
//            double sigma = 0.02;
            if(i==0){
                f->pose=f->poseGroundTruth;
            }else{
                f->pose=addNoise(f->poseGroundTruth,FLAGS_sigma, FLAGS_sigmat);
            }

        }

//        if(demean){
//            if(i==0){
//                Matrix3Xd m = vec2mat(f->pts);
//                centroid = m.rowwise().mean();
//                centroid -= f->pose.translation();
//            }
//            Matrix3Xd centered = m.colwise() - centroid;
//            f->pts = mat2vec(centered);
//        }

        frames.push_back(f);
    }
}

namespace ApproachComponents{

static void computePoseNeighbours(vector< std::shared_ptr<Frame> >& frames, int knn){
    //compute closest points
    MatrixXi adjacencyMatrix=MatrixXi::Zero(frames.size(),frames.size());
    for(int src_id=0; src_id<frames.size(); src_id++){
        Frame& srcCloud = *frames[src_id];
        srcCloud.computePoseNeighboursKnn(&frames,src_id,knn);
        for(int j=0; j< srcCloud.neighbours.size(); j++){
            adjacencyMatrix(src_id,srcCloud.neighbours[j].neighbourIdx)=1;
        }
//        srcCloud.computeClosestPointsToNeighbours(&frames,cutoff);
    }
    cout<<"graph adjacency matrix == block structure"<<endl;
    cout<<adjacencyMatrix<<endl;
}

static void computeClosestPoints(vector< std::shared_ptr<Frame> >& frames, float cutoff){
    //compute closest points
    for(int src_id=0; src_id<frames.size(); src_id++){
        Frame& srcCloud = *frames[src_id];
//        srcCloud.computePoseNeighboursKnn(&frames,src_id,knn);
        cout<<"cloud "<<src_id<<endl;
        srcCloud.computeClosestPointsToNeighbours(&frames,cutoff);
    }
}
}//end ns

int main(int argc, char * argv[]){
    google::ParseCommandLineFlags(&argc, &argv, true);

    vector< std::shared_ptr<Frame> > frames;

    CPUTimer timer = CPUTimer();

    loadFrames(frames,FLAGS_dir,true);

    Visualize::setClouds(&frames);

    frames[0]->fixed=true;
    ApproachComponents::computePoseNeighbours(frames,FLAGS_knn);

    cout<<"press q to start optimization"<<endl;
    Visualize::spin();

//    Visualize::getInstance()->selectedFrame=1;
//    Visualize::getInstance()->selectedOutgoingEdgeIdx=0;

        for(int i=0; i<20; i++){

            timer.tic();
            ApproachComponents::computeClosestPoints(frames,FLAGS_cutoff);
            timer.toc(std::string("closest pts ") + std::to_string(i));


            timer.tic();
            if(!FLAGS_g2o){
                if(FLAGS_sophusSE3) ICP_Ceres::ceresOptimizer_sophusSE3(frames,FLAGS_pointToPlane,FLAGS_robust);
                else if(FLAGS_angleAxis) ICP_Ceres::ceresOptimizer_ceresAngleAxis(frames,FLAGS_pointToPlane,FLAGS_robust);
                else ICP_Ceres::ceresOptimizer(frames, FLAGS_pointToPlane,FLAGS_robust);
            }else{
                ICP_G2O::g2oOptimizer(frames, FLAGS_pointToPlane);
            }

            timer.toc(std::string("global ") + std::to_string(i));
            cout<<"round: "<<i<<endl;
            Visualize::spinToggle(2);
        }

    cout<<"Q (capital q) to quit"<<endl;
    Visualize::spinLast();
}
