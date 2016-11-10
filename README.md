# mv-lm-icp
Multi-view Levenberg-Marquardt Iterative Closest Point Algorithm. Point-to-point and point-to-plane variants. 

A C++ implementation of Section 3.3.2 in:
* Fantoni, Simone, Umberto Castellani, and Andrea Fusiello. ["Accurate and Automatic Alignment of Range Surfaces."](http://www.diegm.uniud.it/fusiello/papers/3dimpvt12-a.pdf) 3DIMPVT (2012): 73-80.

Using nanoflann for the kd-tree and ceres-solver for the actual minimization.

### Documentation
https://github.com/adrelino/mv-lm-icp/blob/master/documentation/mv-lm-icp.pdf


### Background: ICP Variants
point-to-point:
* Besl, Paul J., and Neil D. McKay. ["Method for registration of 3-D shapes."](http://eecs.vanderbilt.edu/courses/CS359/other_links/papers/1992_besl_mckay_ICP.pdf) Robotics-DL tentative (1992): 586-606.

point-to-plane:
* Chen, Yang, and Gérard Medioni. ["Object modelling by registration of multiple range images."](http://graphics.stanford.edu/~smr/ICP/comparison/chen-medioni-align-rob91.pdf) Image and vision computing 10.3 (1992): 145-155.

### Previous work
* Fitzgibbon, Andrew W. ["Robust registration of 2D and 3D point sets."](http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.83.3846&rep=rep1&type=pdf) Image and Vision Computing 21.13 (2003): 1145-1153.
* Brown, Benedict J., and Szymon Rusinkiewicz. ["Global non-rigid alignment of 3-D scans."](http://gfx.cs.princeton.edu/pubs/Brown_2007_GNA/global_tps.pdf) ACM Transactions on Graphics (TOG). Vol. 26. No. 3. ACM, 2007.
* Pulli, Kari. ["Multiview registration for large data sets."](https://graphics.stanford.edu/papers/pulli-3dim99/3dim99.pdf) 3-D Digital Imaging and Modeling, 1999. Proceedings. Second International Conference on. IEEE, 1999.

 


### Dependencies
cmake, Eigen3, Ceres, (g2o)

#### OSX El Capitan
```sh
brew install cmake eigen ceres-solver g2o
```

### Building

```sh
git checkout https://github.com/adrelino/mv-lm-icp.git
cd mv-lm-icp
git submodule init && git submodule update

mkdir build && cd build
cmake .. && make
```
