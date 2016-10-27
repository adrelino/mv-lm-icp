# mv-lm-icp
Multi-view Levenberg-Marquardt ICP

### Documentation
https://github.com/adrelino/mv-lm-icp/blob/master/documentation/mv-lm-icp.pdf

### Previous work
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
