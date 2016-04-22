# mv-lm-icp
Multi-view Levenberg-Marquardt ICP

https://graphics.stanford.edu/papers/pulli-3dim99/3dim99.pdf
http://gfx.cs.princeton.edu/pubs/Brown_2007_GNA/index.php / http://gfx.cs.princeton.edu/pubs/Brown_2007_GNA/global_tps.pdf


### Dependencies
cmake, Eigen3, Ceres, (g2o)

#### OSX El Capitan
```sh
brew install cmake eigen ceres-solver g2o
```

### building

```sh
git checkout https://github.com/adrelino/mv-lm-icp.git
cd mv-lm-icp
git submodule init && git submodule update

mkdir build && cd build
cmake .. && make
```
