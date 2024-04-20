/*
 * img_align.h
 *
 *  Created on: Aug 22, 2012
 *      Author: cforster
 */

#ifndef IMG_ALIGN_H_
#define IMG_ALIGN_H_

#include <vector>
#include <stdint.h>
#include <math.h>
#include <Eigen/Core>
#include <Eigen/StdVector>
#include <opencv2/opencv.hpp>
#include <vikit/math_utils.h>
#include <vikit/pinhole_camera.h>
#include <vikit/nlls_solver.h>
#include <vikit/performance_monitor.h>
#include <sophus/se3.hpp>

namespace vk {

using namespace std;
using namespace Eigen;
using namespace vk;
using namespace Sophus;

//! Forward Compositional Image Alignment
class ForwardCompositionalSE3 : public NLLSSolver<6, SE3d> {

protected:
  std::vector<vk::PinholeCamera>& cam_pyr_;
  std::vector<cv::Mat>&      depth_pyr_;
  std::vector<cv::Mat>&      img_pyr_;
  std::vector<cv::Mat>&      tpl_pyr_;
  std::vector<cv::Mat>&      img_pyr_dx_;
  std::vector<cv::Mat>&      img_pyr_dy_;
  int                   level_;
  int                   n_levels_;
  PerformanceMonitor    permon_;
  bool                  display_;
  bool                  log_;
  double                res_thresh_;

  virtual double
  computeResiduals (const SE3d& model, bool linearize_system, bool compute_weight_scale = false);

  virtual int
  solve();

  virtual void
  update(const ModelType& old_model,  ModelType& new_model);

  virtual void
  startIteration();

  virtual void
  finishIteration();

public:
  cv::Mat               resimg_;

  ForwardCompositionalSE3( std::vector<PinholeCamera>& cam_pyr,
                           std::vector<cv::Mat>& depth_pyr,
                           std::vector<cv::Mat>& img_pyr,
                           std::vector<cv::Mat>& tpl_pyr,
                           std::vector<cv::Mat>& img_pyr_dx,
                           std::vector<cv::Mat>& img_pyr_dy,
                           SE3d& init_model,
                           int n_levels,
                           int n_iter = 50,
                           float res_thresh = 0.2,
                           bool display = true,
                           Method method = LevenbergMarquardt,
                           int test_id = 0);

  ForwardCompositionalSE3( std::vector<PinholeCamera>& cam_pyr,
                           std::vector<cv::Mat>& depth_pyr,
                           std::vector<cv::Mat>& img_pyr,
                           std::vector<cv::Mat>& tpl_pyr,
                           std::vector<cv::Mat>& img_pyr_dx,
                           std::vector<cv::Mat>& img_pyr_dy,
                           int n_levels,
                           int n_iter = 50,
                           float res_thresh = 0.2,
                           bool display = true,
                           Method method = LevenbergMarquardt,
                           int test_id = 0);

  void
  runOptimization(Sophus::SE3d& model, int levelBegin = -1, int levelEnd = -1);

};


//! Efficient Second Order Minimization (ESM)
class SecondOrderMinimisationSE3 : public NLLSSolver<6, SE3d> {

protected:
  std::vector<vk::PinholeCamera>& cam_pyr_;
  std::vector<cv::Mat>&      depth_pyr_;
  std::vector<cv::Mat>&      img_pyr_;
  std::vector<cv::Mat>&      tpl_pyr_;
  std::vector<cv::Mat>&      img_pyr_dx_;
  std::vector<cv::Mat>&      img_pyr_dy_;
  std::vector<cv::Mat>&      tpl_pyr_dx_;
  std::vector<cv::Mat>&      tpl_pyr_dy_;
  int                   level_;
  PerformanceMonitor    permon_;
  bool                  display_;
  bool                  log_;
  float                 res_thresh_;

  virtual double
  computeResiduals (const SE3d& model, bool linearize_system, bool compute_weight_scale = false);

  virtual int
  solve();

  virtual void
  update(const ModelType& old_model,  ModelType& new_model);

  virtual void
  startIteration();

  virtual void
  finishIteration();

public:
  cv::Mat               resimg_;

  SecondOrderMinimisationSE3( std::vector<PinholeCamera>& cam_pyr,
                              std::vector<cv::Mat>& depth_pyr,
                              std::vector<cv::Mat>& img_pyr,
                              std::vector<cv::Mat>& tpl_pyr,
                              std::vector<cv::Mat>& img_pyr_dx,
                              std::vector<cv::Mat>& img_pyr_dy,
                              std::vector<cv::Mat>& tpl_pyr_dx,
                              std::vector<cv::Mat>& tpl_pyr_dy,
                              SE3d& init_model,
                              int n_levels,
                              int n_iter = 50,
                              float res_thresh = 0.2,
                              bool display = true,
                              Method method = LevenbergMarquardt,
                              int test_id = -1);
};

} // end namespace ImgAlign

#endif /* IMG_ALIGN_H_ */
