#include "cam/CamRadtan.h"
#include <iostream>
#include <stdexcept>

// Independent OpenCV reference in double precision, including measured k3.
Eigen::Vector2d reference(const Eigen::Vector2d &xy, const Eigen::VectorXd &p, double k3) {
  cv::Matx33d k(p(0),0,p(2),0,p(1),p(3),0,0,1);
  cv::Vec<double,5> d(p(4),p(5),p(6),p(7),k3);
  std::vector<cv::Point3d> points{{xy.x(),xy.y(),1}};
  std::vector<cv::Point2d> result;
  cv::projectPoints(points,cv::Vec3d(0,0,0),cv::Vec3d(0,0,0),k,d,result);
  return {result[0].x,result[0].y};
}
void require(bool pass,const char* message) { if(!pass) throw std::runtime_error(message); }
int main() {
  Eigen::VectorXd p(8);
  p << 332.6789685600847,333.44863304719337,329.8565463602694,180.64067423778448,
       -.31654277244569684,.09154742632509782,-.0010595481580590413,-.0005932283653060463;
  for(double k3 : {0.0,-.01115685698766181,.02}) {
    ov_core::CamRadtan camera(640,400,k3); camera.set_value(p);
    for(int ix=-9;ix<=9;++ix) for(int iy=-6;iy<=6;++iy) {
      Eigen::Vector2d xy(ix*.1,iy*.1);
      auto expected=reference(xy,p,k3);
      auto actual=camera.distort_f(xy.cast<float>());
      require((actual.cast<double>()-expected).norm()<1e-4,"projection lost calibration");
      require((camera.undistort_d(expected)-xy).norm()<2e-5,"inverse distortion mismatch");
      Eigen::MatrixXd h,j; camera.compute_distort_jacobian(xy,h,j);
      const double epsilon=1e-6;
      for(int axis=0;axis<2;++axis) {
        auto plus=xy,minus=xy; plus(axis)+=epsilon;minus(axis)-=epsilon;
        Eigen::Vector2d numeric=(reference(plus,p,k3)-reference(minus,p,k3))/(2*epsilon);
        require((numeric-h.col(axis)).norm()<1e-5,"normalized Jacobian mismatch");
      }
      for(int axis=0;axis<8;++axis) {
        auto plus=p,minus=p;plus(axis)+=epsilon;minus(axis)-=epsilon;
        Eigen::Vector2d numeric=(reference(xy,plus,k3)-reference(xy,minus,k3))/(2*epsilon);
        require((numeric-j.col(axis)).norm()<1e-5,"intrinsics Jacobian mismatch");
      }
    }
  }
  std::cout << "741 projection/inverse and 7410 finite-difference Jacobian comparisons passed\n";
}
