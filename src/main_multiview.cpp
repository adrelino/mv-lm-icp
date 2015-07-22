#include "Visualize.h"
#include "common.h"
#include "gflags/gflags.h"
#include "CPUTimer.h"
#include "algorithms.h"
#include "frame.h"
#include "approachcomponents.h"

using namespace std;

DEFINE_bool(pointToPlane, false, "use point to plane distance metric");

DEFINE_bool(g2o, false, "use g2o");
DEFINE_double(cutoff,0.1,"cutoff distance for correspondences"); //dmax
DEFINE_int32(knn,2,"knn"); //dmax

//DEFINE_string(dir,"../samples/dinosaur","dir");
DEFINE_string(dir,"../samples/Bunny_RealData","dir");

DEFINE_double(sigma,0.001,"rotation variance");
DEFINE_double(sigmat,0.001,"translation variance");

DEFINE_bool(fake,false,"fake");
DEFINE_int32(limit,5,"limit");
DEFINE_int32(step,1,"step");

//Vector3d centroid;

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

int main(int argc, char * argv[]){

    gflags::ParseCommandLineFlags(&argc, &argv, true);

    vector< std::shared_ptr<Frame> > frames;

    CPUTimer timer = CPUTimer();

    cout<<"STAGE 0: preprocessing"<<endl;
//    timer.tic();
    loadFrames(frames,FLAGS_dir,true);
//    timer.toc("0_preprocessing");


//    frames[1]->pose = frames[0]->pose * Translation3f(0.00,0.00,-0.05);
//    frames[2]->pose = frames[0]->pose * Translation3f(0.00,0.00,0.05);

//    frames[1]->poseGroundTruth = frames[0]->poseGroundTruth;
//    frames[2]->poseGroundTruth = frames[0]->poseGroundTruth;

    Visualize::setClouds(&frames);


//    Visualize::setCentroid(getCentroid(frames[0]->pts));
//    cout<<centroid.transpose();
//    Visualize::setCentroid(centroid);


//        //frames[0]->updateChildrenAbsolutePoses(frames,0);  // update absolute poses of children of this node in dependecy tree

//        Visualize::spin();

//        //ApproachComponents::computeClosestPointsFake(frames,cutoffGlobal,knn);

    ApproachComponents::computePoseNeighbours(frames,FLAGS_knn);

    Visualize::spin();


    Visualize::getInstance()->selectedFrame=0;
    Visualize::getInstance()->selectedOutgoingEdgeIdx=0;


        for(int i=0; i<50; i++){

            timer.tic();
            ApproachComponents::computeClosestPoints(frames,FLAGS_cutoff);
            timer.toc(std::string("closest pts ") + std::to_string(i));
//            Visualize::spin();


            timer.tic();
            if(!FLAGS_g2o){
                ApproachComponents::ceresOptimizer(frames, FLAGS_pointToPlane);
            }else{
                ApproachComponents::g2oOptimizer(frames, FLAGS_pointToPlane);
            }

//            //ApproachComponents::g2oOptimizer(frames);
            timer.toc(std::string("global ") + std::to_string(i));
            cout<<"round: "<<i<<endl;
            Visualize::spinToggle(2);
        }




    cout<<"Q (capital q) to quit"<<endl;
    Visualize::spinLast();

}
