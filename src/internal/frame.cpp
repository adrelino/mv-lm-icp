#include "frame.h"
#include <algorithm>

Frame::Frame() : uploaded(false), fixed(false), indexComputed(false) {
//    recomputeNormals();
}

Frame::~Frame()
{
//    delete[] pts;
    cout<<"delete frame: "<<triangleVBO<<endl;
    glDeleteBuffers(1, &triangleVBO);

}

void Frame::upload(){
    //http://www.songho.ca/opengl/gl_vbo.html
    //Vertices of a triangle (counter-clockwise winding)
    //float data[] = {1.0, 0.0, 1.0, 0.0, 0.0, -1.0, -1.0, 0.0, 1.0};
    //try float data[] = {0.0, 1.0, 0.0, -1.0, -1.0, 0.0, 1.0, -1.0, 0.0}; if the above doesn't work.

    //Create a new VBO and use the variable id to store the VBO id
    glGenBuffers(1, &triangleVBO);

    cout<<"uploaded frame: "<<triangleVBO<<endl;

    //Make the new VBO active
    glBindBuffer(GL_ARRAY_BUFFER, triangleVBO);

    //Upload vertex data to the video device
    glBufferData(GL_ARRAY_BUFFER, sizeof(Vector3d)*pts.size(), pts.data(), GL_STATIC_DRAW);

    uploaded=true;
}

void Frame::draw(){
    if(!uploaded) upload();

    glPushMatrix();
    glPointSize(5.0f);
    glMultMatrixd(pose.matrix().data());
//    glColor3d(1,0,0);

    //Make the new VBO active. Repeat here incase changed since initialisation
    glBindBuffer(GL_ARRAY_BUFFER, triangleVBO);

    //Draw Triangle from VBO - do each time window, view point or data changes
    //Establish its 3 coordinates per vertex with zero stride in this array; necessary here
    glVertexPointer(3, GL_DOUBLE, 0, NULL);

    //Establish array contains vertices (not normals, colours, texture coords etc)
    glEnableClientState(GL_VERTEX_ARRAY);

    //Actually draw the triangle, giving the number of vertices provided
    glDrawArrays(GL_POINTS, 0, pts.size());

    glDisableClientState(GL_VERTEX_ARRAY);

    glPopMatrix();
}

#include <iomanip>

bool myfunction1 (OutgoingEdge a, OutgoingEdge b) { return (a.weight<b.weight); }

void Frame::computePoseNeighboursKnn(vector< shared_ptr<Frame> >* frames, int i, int k){
    neighbours.clear();

    Isometry3d& P1 = pose;
    for (int j = 0; j < (*frames).size(); ++j){
        if(i==j) continue;
        Isometry3d& P2 = (*frames)[j]->pose;

        //Translation
        float diff_tra=(P1.translation()-P2.translation()).norm();

//        if (diff_tra<cutoff){
            neighbours.push_back({j,diff_tra});
//        }
    }

    if(neighbours.size()<k){
        std::sort(neighbours.begin(),neighbours.end(),myfunction1);
    }else{
        std::partial_sort (neighbours.begin(), neighbours.begin()+k, neighbours.end(),myfunction1);
        neighbours.resize(k);
    }
}

void Frame::computeClosestPointsToNeighbours(vector< shared_ptr<Frame> >* frames, float thresh){

    if(this->fixed) return;
    Frame& srcCloud = *this;

//    Vector3d Ts=Vector3d(srcCloud.pose.translation());
//    Matrix3d Rs=srcCloud.pose.linear();
//    Matrix3d Rsi=Rs.transpose();



//    Isometry3d srcPoseI=srcCloud.pose.inverse();
//    cout<<"srcPose"<<endl;
//    cout<<srcCloud.pose.matrix()<<endl;
//    cout<<srcPoseI.matrix()<<endl;

    for (int j = 0; j < srcCloud.neighbours.size(); ++j) {

        int dst_id=srcCloud.neighbours[j].neighbourIdx;
        srcCloud.neighbours[j].correspondances.clear();
//        cout<<"neighbours: "<<dst_id<<endl;

        Frame& dstCloud = *frames->at(dst_id);

//        Isometry3d dstPoseI=dstCloud.pose.inverse();

        Vector3d preTra = Vector3d(dstCloud.pose.translation());
        auto preInvRot = dstCloud.pose.linear().inverse();
//        Vector3d Td=dstCloud.pose.translation();
//        Matrix3d Rd=dstCloud.pose.linear();

//        Matrix3d RdRsi=Rd*Rsi;


//        float error=0;

        std::vector<double> dists;

        for (int k = 0; k < srcCloud.pts.size(); ++k) {
            Vector3d& srcPtOrig = srcCloud.pts[k];
            Vector3d srcPtInGlobalFrame = srcCloud.pose*srcPtOrig;

            size_t idxMin;
            double pointDistSquared;

            Vector3d srcPtinDstFrame = preInvRot * (srcPtInGlobalFrame-preTra);
//            Vector3d srcPtinDstFrame = RdRsi*srcPtOrig-RdRsi*Ts+Td;
            pointDistSquared = dstCloud.getClosestPoint(srcPtinDstFrame,idxMin);

//            pointDistSquared = 1e10;
            double pointDist = 1e10;
            pointDist = sqrt(pointDistSquared);
//            for(int i=0; i<dstCloud.pts.size(); i++){
//                Vector3d dstPtInGlobalFrame = dstCloud.pose*dstCloud.pts[i];
//                Vector3d diffVec=(srcPtInGlobalFrame-dstPtInGlobalFrame);
//                double diff = diffVec.norm();
//                if(diff<pointDist){
//                    pointDist=diff;
//                    idxMin=i;
//                }
//            }

            //Vector3f srcPtinDstFrame = preInv*srcPtInGlobalFrame;
            //pointDistSquared = dstCloud.getClosestPointInGlobalFrameLinear(srcPtInGlobalFrame,idxMin);

            if(pointDist<thresh){
//                error += pointDistSquared;
                srcCloud.neighbours[j].correspondances.push_back({k,(int) idxMin,pointDist});
                dists.push_back(pointDist);
            }
        }



////        // this sets iterator middle to the median element
        std::vector<double>::iterator middle = dists.begin() + (dists.size() / 2);
         std::nth_element(dists.begin(), middle, dists.end());
         double nthValue = *middle;

         cout<<"median dist to: "<<dst_id<<" is: "<<nthValue<<endl;

//         summary(dists);

//        int sizeBefore=srcCloud.neighbours[j].correspondances.size();

        srcCloud.neighbours[j].weight=nthValue*1.5;

//        srcCloud.neighbours[j].correspondances.erase(std::remove_if(srcCloud.neighbours[j].correspondances.begin(),
//                                                                    srcCloud.neighbours[j].correspondances.end(),
//                                                                    [nthValue](Correspondance c){return c.dist>(nthValue*1.5);}),
//                                                    srcCloud.neighbours[j].correspondances.end());

//        cout<<"->"<<dst_id<<" size before:"<<sizeBefore<<" size after"<<srcCloud.neighbours[j].correspondances.size()<<endl;
    }
}

double Frame::getClosestPoint(const Vector3d& query_pt, size_t& ret_index){ //query_pt must be in dstFrame
    if(!indexComputed){
        indexPtr = new my_kd_tree_t(3 /*dim*/, *this, nanoflann::KDTreeSingleIndexAdaptorParams(1 /* max leaf */));
        indexPtr->buildIndex();
        cout<<"flann: build index"<<endl;
        indexComputed=true;
    }
        // do a knn search
        const size_t num_results = 1;
        //size_t ret_index;
        double out_dist_sqr;
        nanoflann::KNNResultSet<double> resultSet(num_results);
        resultSet.init(&ret_index, &out_dist_sqr );
        indexPtr->findNeighbors(resultSet, &query_pt[0], nanoflann::SearchParams(32,0,false));

        //std::cout << "knnSearch(nn="<<num_results<<"): \n";
        //std::cout << "ret_index=" << ret_index << " out_dist_sqr=" << out_dist_sqr << endl;

        return out_dist_sqr;
}

vector<Vector3d> Frame::getNeighbours(int queryIdx, size_t num_results){ //query_pt must be in dstFrame
    if(!indexComputed){
        indexPtr = new my_kd_tree_t(3 /*dim*/, *this, nanoflann::KDTreeSingleIndexAdaptorParams(1 /* max leaf */));
        indexPtr->buildIndex();
        cout<<"flann: build index"<<endl;
        indexComputed=true;
    }
        // do a knn search

//    const size_t num_results = 5;
    std::vector<size_t> ret_index(num_results);
    std::vector<double> out_dist_sqr(num_results);
    indexPtr->knnSearch(pts[queryIdx].data(), num_results, &ret_index[0], &out_dist_sqr[0]);

    vector<Vector3d> neighbours;

    for (int i = 0; i < ret_index.size(); ++i) {
        neighbours.push_back(pts[ret_index[i]]);
    }

    return neighbours;


//        const size_t num_results = 1;
//        //size_t ret_index;
//        double out_dist_sqr;
//        nanoflann::KNNResultSet<double> resultSet(num_results);
//        resultSet.init(&ret_index, &out_dist_sqr );
//        indexPtr->findNeighbors(resultSet, &query_pt[0], nanoflann::SearchParams(32,0,false));

        //std::cout << "knnSearch(nn="<<num_results<<"): \n";
        //std::cout << "ret_index=" << ret_index << " out_dist_sqr=" << out_dist_sqr << endl;

//        return out_dist_sqr;
}

void Frame::recomputeNormals(){
    int n = pts.size();
    for (int i = 0; i < n; ++i) {
        Vector3d centroid;
        double curvature;
        vector<Vector3d> neighbours = getNeighbours(i,10);

        pointSetPCA(neighbours,centroid,nor[i],curvature);
    }
    cout<<"recomputeNormals for "<<n<<" pts"<<endl;

}
