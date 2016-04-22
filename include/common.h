#ifndef COMMON_H
#define COMMON_H

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <vector>
#include <iostream>
#include <fstream>

#include <string>
#include <iomanip>
#include <cmath>
#include <numeric>

using namespace std;
using namespace Eigen;

#define rad2deg2(r) (180*(r)/M_PI)
//#define deg2rad(d) (M_PI*(d)/180)

#include <dirent.h>
#include <vector>
#include <algorithm>    // std::any_of
//#include <array>        // std::array


#include <sophus/so3.hpp>
#include <random>
#include <memory>

static std::mt19937 generator;

static Isometry3d addNoise(const Isometry3d& pose, double sigma, double sigmat){
    double mean = 0.0;
    double std  = 1.0;
    std::normal_distribution<double> normal(mean, std);

    Vector3d w(normal(generator), normal(generator), normal(generator));
    w*=sigma;
//    cout<<"w: "<<w.transpose()<<endl;
    auto foo = Sophus::SO3d::exp(w);

//    cout<<"mat: "<<endl<<foo.matrix()<<endl;

    Isometry3d noisyPose(pose*foo.unit_quaternion());

    Vector3d t(normal(generator), normal(generator), normal(generator));
    t*=sigmat;

//    cout<<"t: "<<t.transpose()<<endl;

    noisyPose.translation() +=t;

//    noisyPose.linear() = foo.matrix() * noisyPose.linear();

    return noisyPose;

//    Sophus::SO3::exp();

//    w=sigmaC*randn(3,1);

}

static bool isPrefixAndSuffix(const char* file, uint16_t filename_length, string prefix, string suffix){


   const char* startSuffix=strstr(file,suffix.c_str());
   bool isSuffix = (startSuffix-file) == filename_length - suffix.length();
   if(!isSuffix) return false;


   if(prefix[0]=='*'){
        string contains = prefix.substr (1);
        const char* startContains=strstr(file,contains.c_str());
        bool isContains = (startContains-file >= 0); //contains
        return isContains;
   }

    const char* startPrefix=strstr(file,prefix.c_str());
    bool isPrefix = (startPrefix-file == 0);

    //cout<<"start: "<<isPrefix<<" end:"<<isSuffix<<endl;

    return isPrefix;
}

static bool hasPrefixAndSuffixes(const char* file, uint16_t filename_length, string prefix,std::vector<string> suffixes){
  return std::any_of(suffixes.begin(), suffixes.end(),
         [&](string suffix){return isPrefixAndSuffix(file,filename_length,prefix,suffix);
  });
}

static string getOSSeparator() {
#ifdef _WIN32
  return "\\";
#else
  return "/";
#endif
}

//http://stackoverflow.com/questions/9277906/stdvector-to-string-with-custom-delimiter
static string join(vector<string>& v, const string& delim) {
    ostringstream s;
    for (const auto& i : v) {
        if (&i != &v[0]) {
            s << delim;
        }
        s << i;
    }
    return s.str();
}


static vector<string> getAllFilesFromFolder(string dirStr, string prefix, vector<string> suffixes) {
  DIR *dir = NULL;
  struct dirent *entry;
  vector<string> allImages;

  dir = opendir(dirStr.c_str());

  if (!dir) {
    cerr << "Could not open directory " << dirStr <<endl;//<< ". Exiting..." << endl;
    return allImages;
    //exit(1);
  }

  const string sep = getOSSeparator();

  while((entry = (readdir(dir)))) {
      string fileName(entry->d_name);
    //#ifdef __MACH__
      uint16_t len = fileName.size(); //entry->d_namlen;
    //#else
      //igned short len = entry->d_reclen;
      cout<<entry->d_name<<"\t"<<len<<endl;
    //#endif
    if (hasPrefixAndSuffixes(entry->d_name,len,prefix,suffixes)){
      string fullPath = dirStr + sep + fileName;
      allImages.push_back(fullPath);
    }
  }
  closedir(dir);

  std::sort(allImages.begin(), allImages.end(), [](const std::string &left, const std::string &right) {
      int lengthdiff=left.size()-right.size();
      if(lengthdiff==0){
          return left < right;
      }
      return lengthdiff<0;
  });

  string joined = join(suffixes,"|");

  cout<<"Found "<<allImages.size()<<" files in "<<dirStr << " that match the pattern: "<<prefix<<"*"<<joined<<" .";
  if(allImages.size()>0){
      cout<<" first is: "<<allImages[0];
  }
  cout<<endl;
  return allImages;
}

static const vector<string> SUFFIX_TEXT  = {".txt",".xyz"};
static vector<string> getAllTextFilesFromFolder(string dirStr, string prefix){
    return getAllFilesFromFolder(dirStr,prefix,SUFFIX_TEXT);
}

static Matrix4d loadMatrix4d(std::string filename){
    std::ifstream file(filename,std::ifstream::in);
    if( file.fail() == true )
    {
        cerr << filename << " could not be opened" << endl;
        return Matrix4d::Zero();

    }
    double array[16]={0};
    array[15]=1;
    int i=0;
    while(file >> array[i++]){}

    //cout<<"Loaded Matrix4f from "<<filename<<endl;
    return Map< Matrix<double,4,4,RowMajor> > (array);
}

static vector<Eigen::Vector3d> loadPLY(const std::string filename)
{
  vector<Eigen::Vector3d> pts;
  int numVertices=0;

  std::ifstream ifs(filename.c_str());

  if (!ifs.is_open())
  {
    printf("Cannot open file...\n");
    return pts;
  }

  std::string str;
  while (str.substr(0, 10) !="end_header")
  {
    std::string entry = str.substr(0, 14);
    if (entry == "element vertex")
    {
      numVertices = atoi(str.substr(15, str.size()-15).c_str());
    }
    std::getline(ifs, str);
  }

  pts.resize(numVertices);

  for (int i = 0; i < numVertices; i++)
  {
    double* data1 = (double*)(&pts[i]);
    ifs >> data1[0] >> data1[1] >> data1[2];
  }

  return pts;
}

static void loadXYZ(const std::string filename, vector<Vector3d>& pts, vector<Vector3d>& nor)
{
    std::ifstream file(filename.c_str(),std::ifstream::in);
    if( file.fail() == true )
    {
        cerr << filename << " could not be opened" << endl;
    }

    int i=0;
    while(file){
        Vector3d pt,no;
        file >> pt.x() >> pt.y() >> pt.z() >> no.x() >> no.y() >> no.z();
        pts.push_back(pt);
        nor.push_back(no);
    }
}

//https://forum.kde.org/viewtopic.php?f=74&t=94839
static Matrix3Xd vec2mat(vector<Vector3d>& vec){
    Map<Matrix<double,3,Dynamic,ColMajor> > mat(&vec[0].x(), 3, vec.size());
    return mat;
}

static vector<Vector3d> mat2vec(const Matrix3Xd& mat){
    vector<Vector3d> vec(mat.cols());
    for(int i=0; i<mat.cols(); i++){
        vec[i]=mat.col(i);
    }

    //http://stackoverflow.com/questions/26094379/typecasting-eigenvectorxd-to-stdvector
//    vector<Vector3d> vec(mat.data(),mat.data() + mat.rows() * mat.cols());
    return vec;
}


static std::string poseDiff(Isometry3d P1, Isometry3d P2){
    //cout<<"isPoseSimilar tra: "<<thresh_tra_l<<" rot: "<<thresh_rot_l<<endl;
    Vector3d    tra1 = P1.translation();
    Quaterniond rot1(P1.linear());

    Vector3d    tra2 = P2.translation();
    Quaterniond rot2(P2.linear());


    //Translation
    double diff_tra=(tra1-tra2).norm();
    //Rotation
    double d = rot1.dot(rot2);
    double val = 2*d*d - 1;
    if(val< -1) val = -1;
    if(val> 1) val = 1;
    double diff_rot_degrees = rad2deg2(acos(val));


    stringstream ss;
    ss<<"\t diff_tra:"<<diff_tra<<"\t diff_rot_degrees:"<<diff_rot_degrees<<endl;

    return ss.str();
}

static Vector3d getCentroid(vector<Vector3d>& pts){
    Matrix3Xd m = vec2mat(pts);
    Vector3d mean1=m.rowwise().mean();
    return mean1;
}

static void summary(std::vector<double> v){  //Like R's summary
    double sum = std::accumulate(v.begin(), v.end(), 0.0);
    double mean = sum / v.size();

    double sq_sum = std::inner_product(v.begin(), v.end(), v.begin(), 0.0);
    double std = std::sqrt(sq_sum / v.size() - mean * mean);

    std::sort(v.begin(),v.end());
    double min=v[0];
    double firstQuantile=v[v.size()*0.25];
    double median=v[v.size()*0.5];
    double thirdQuantile=v[v.size()*0.75];
    double max=v[v.size()-1];

    cout<<"Summary of "<<v.size()<<" bucket sizes:"<<endl;
    cout<<"Min\t.25\tMed\tMean\t.75\tMax \tStd"<<endl;
    cout<<min<<" \t"<<firstQuantile<<" \t"<<median<<" \t"<<round(mean*100)*0.01<<" \t"<<thirdQuantile<<" \t"<<max<<" \t"<<round(std*100)*0.01<<endl;





    /*std::vector<double> diff(v.size());
    std::transform(v.begin(), v.end(), diff.begin(),
                   std::bind2nd(std::minus<double>(), mean));
    double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
    double stdev = std::sqrt(sq_sum / v.size());


    double sum = std::accumulate(std::begin(v), std::end(v), 0.0);
    double m =  sum / v.size();

    double accum = 0.0;
    std::for_each (std::begin(v), std::end(v), [&](const double d) {
        accum += (d - m) * (d - m);
    });

    double stdev = sqrt(accum / (v.size()-1));*/

}

static void pointSetPCA(const vector<Vector3d>& pts, Vector3d& centroid, Vector3d& normal, double& curvature){

    assert(pts.size()>=3); //otherwise normals are undetermined
    Map<const Matrix3Xd> P(&pts[0].x(),3,pts.size());

    centroid = P.rowwise().mean();
    MatrixXd centered = P.colwise() - centroid;
    Matrix3d cov = centered * centered.transpose();

    //eigvecs sorted in increasing order of eigvals
    SelfAdjointEigenSolver<Matrix3d> eig(cov);
    normal = eig.eigenvectors().col(0); //is already normalized
    if (normal(2) > 0) normal = -normal; //flip towards camera
    Vector3d eigVals = eig.eigenvalues();
    curvature = eigVals(0) / eigVals.sum();
}

#endif // COMMON_H

