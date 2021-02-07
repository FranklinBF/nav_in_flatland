
#include <arena_mapping/edt_environment.h>


void EDTEnvironment::init() {
}

void EDTEnvironment::setMap(SDFMap::Ptr  map) {
  this->sdf_map_ = map;
  resolution_inv_ = 1 / sdf_map_->getResolution();
}

int EDTEnvironment::getOccupancy(Eigen::Vector2d pos){
  return sdf_map_->getFusedInflateOccupancy(pos);
}

/* Get distance*/
double EDTEnvironment::evaluateCoarseEDT(Eigen::Vector2d& pos) {
  double d1 = sdf_map_->getDistance(pos);
  return d1;
}


/* Get distance gradient*/
void EDTEnvironment::getSurroundDistance(Eigen::Vector2d pts[2][2], double dists[2][2]) {
  for (int x = 0; x < 2; x++) {
    for (int y = 0; y < 2; y++) {
        dists[x][y] = sdf_map_->getDistance(pts[x][y]);
    }
  }
}


std::pair<double, Eigen::Vector2d> EDTEnvironment::interpolateBilinear(double values[2][2],
                                                                   const Eigen::Vector2d& diff,
                                                                   double& value,
                                                                   Eigen::Vector2d& grad) {
  
  // bilinear interpolation
  double v00=values[0][0];  //f(x0,y0)
  double v10=values[1][0];  //f(x1,y0)
  double v01=values[0][1];  //f(x0,y1)
  double v11=values[1][1];  //f(x1,y1)

  double v0=(1-diff(0))*v00+diff(0)*v10;  
  double v1=(1-diff(0))*v01+diff(0)*v11;
  value = (1-diff(1))*v0 + diff(1)*v1;
  
  // calculate gradient
  grad[1]= (v1-v0) * resolution_inv_;
  grad[0]= ((1 - diff[1]) * (v10 - v00) + diff[1] * (v11 - v01)) * resolution_inv_;

}




std::pair<double, Eigen::Vector3d> EDTEnvironment::evaluateEDTWithGrad(  const Eigen::Vector2d& pos,
                                                                    double& dist,
                                                                    Eigen::Vector2d& grad) {
    // get diff & surround pts
    Eigen::Vector2d diff;
    Eigen::Vector2d sur_pts[2][2];
    sdf_map_->getSurroundPts(pos, sur_pts, diff);

    // get distances of the surround pts
    double dists[2][2];
    getSurroundDistance(sur_pts, dists);

    // do interpolate to get distance gradient
    interpolateBilinear(dists, diff, dist, grad);
    
}





