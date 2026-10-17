#include <gtest/gtest.h>
#include <random>
#include "../vins_estimator/src/factor/pose_local_parameterization.h"
#include "../vins_estimator/src/factor/line_parameterization.h"
#include "../vins_estimator/src/factor/projection_factor.h"
#include "../vins_estimator/src/factor/line_projection_factor.h"
#include "../vins_estimator/src/factor/marginalization_factor.h"
#include "../vins_estimator/src/factor/imu_factor.h"

namespace {
void checkManifold(const ceres::Manifold& manifold, const std::vector<double>& x) {
  const int a=manifold.AmbientSize(),t=manifold.TangentSize();
  std::vector<double> delta(t,.002),y(a),back(t);
  ASSERT_TRUE(manifold.Plus(x.data(),delta.data(),y.data()));
  ASSERT_TRUE(manifold.Minus(y.data(),x.data(),back.data()));
  for (int i=0;i<t;++i) EXPECT_NEAR(back[i],delta[i],1e-9);
  Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> J(a,t),M(t,a);
  ASSERT_TRUE(manifold.PlusJacobian(x.data(),J.data()));
  ASSERT_TRUE(manifold.MinusJacobian(x.data(),M.data()));
  EXPECT_LT((M*J-Eigen::MatrixXd::Identity(t,t)).norm(),1e-10);
  constexpr double eps=1e-7;
  for (int k=0;k<t;++k) {
    std::fill(delta.begin(),delta.end(),0.); delta[k]=eps;
    std::vector<double> yp(a),ym(a);
    manifold.Plus(x.data(),delta.data(),yp.data()); delta[k]=-eps;
    manifold.Plus(x.data(),delta.data(),ym.data());
    for (int r=0;r<a;++r) EXPECT_NEAR(J(r,k),(yp[r]-ym[r])/(2*eps),2e-8);
  }
}
void checkFactor(ceres::CostFunction& factor,std::vector<std::vector<double>> blocks,
                 const std::vector<const ceres::Manifold*>& manifolds,double tolerance=2e-5) {
  const int rows=factor.num_residuals();
  std::vector<const double*> parameters; std::vector<double*> jacobians;
  std::vector<Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>> matrices;
  matrices.reserve(blocks.size());
  for (auto& block:blocks) { parameters.push_back(block.data()); matrices.emplace_back(rows,block.size()); }
  for (auto& matrix:matrices) jacobians.push_back(matrix.data());
  std::vector<double> residual(rows);
  ASSERT_TRUE(factor.Evaluate(parameters.data(),residual.data(),jacobians.data()));
  for (size_t i=0;i<blocks.size();++i) {
    const auto* manifold=manifolds[i]; const int n=blocks[i].size(), t=manifold?manifold->TangentSize():n;
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> plus(n,t);
    if (manifold) ASSERT_TRUE(manifold->PlusJacobian(blocks[i].data(),plus.data())); else plus.setIdentity();
    const Eigen::MatrixXd local=matrices[i]*plus;
    for (int k=0;k<t;++k) {
      const double eps=1e-6; std::vector<double> delta(t,0.),yp(n),ym(n),rp(rows),rm(rows);
      delta[k]=eps;
      if (manifold) manifold->Plus(blocks[i].data(),delta.data(),yp.data());
      else { yp=blocks[i]; yp[k]+=eps; }
      delta[k]=-eps;
      if (manifold) manifold->Plus(blocks[i].data(),delta.data(),ym.data());
      else { ym=blocks[i]; ym[k]-=eps; }
      parameters[i]=yp.data(); ASSERT_TRUE(factor.Evaluate(parameters.data(),rp.data(),nullptr));
      parameters[i]=ym.data(); ASSERT_TRUE(factor.Evaluate(parameters.data(),rm.data(),nullptr));
      parameters[i]=blocks[i].data();
      for (int r=0;r<rows;++r)
        EXPECT_NEAR(local(r,k),(rp[r]-rm[r])/(2*eps),tolerance) << "block "<<i<<" row "<<r<<" col "<<k;
    }
  }
}
std::vector<double> pose(double px,double angle) {
  Eigen::Quaterniond q(Eigen::AngleAxisd(angle,Eigen::Vector3d(.3,.4,.5).normalized()));
  return {px,.2,-.1,q.x(),q.y(),q.z(),q.w()};
}
}
TEST(Manifolds,PoseRetractionAndJacobian) {
  for (double angle:{0.,.4,1.6,2.9}) checkManifold(PoseLocalParameterization(),pose(.3,angle));
}
TEST(Manifolds,LineRetractionAndJacobian) {
  for (double angle:{-.7,0.,.8}) checkManifold(LineOrthParameterization(),{.3,angle,-.4,.7});
}
TEST(Factors,PointProjection) {
  ProjectionFactor::sqrt_info.setIdentity();
  ProjectionFactor factor(Eigen::Vector3d(.2,.3,1.),Eigen::Vector3d(.1,.4,1.));
  PoseLocalParameterization pose_manifold;
  checkFactor(factor,{pose(.2,.4),pose(.3,-.2),pose(.1,.15),{.3}}, {&pose_manifold,&pose_manifold,&pose_manifold,nullptr});
}
TEST(Factors,LineProjection) {
  lineProjectionFactor::sqrt_info.setIdentity();
  lineProjectionFactor_incamera::sqrt_info.setIdentity();
  lineProjectionFactor_instartframe::sqrt_info.setIdentity();
  PoseLocalParameterization p; LineOrthParameterization l;
  Eigen::Vector4d observation(.1,.2,.4,.6);
  lineProjectionFactor world(observation);
  checkFactor(world,{pose(.2,.4),pose(.1,-.3),{.2,.3,.4,.7}},{&p,&p,&l});
  lineProjectionFactor_incamera camera(observation);
  checkFactor(camera,{pose(.2,.4),pose(.1,-.3),pose(.12,.2),{.2,.3,.4,.7}},{&p,&p,&p,&l});
  lineProjectionFactor_instartframe start(observation);
  checkFactor(start,{{.2,.3,.4,.7}},{&l});
}
TEST(Factors,ImuPreintegration) {
  ACC_N=.08; GYR_N=.004; ACC_W=.00004; GYR_W=.000002;
  IntegrationBase integration(Eigen::Vector3d(.1,.2,9.8),Eigen::Vector3d(.01,.02,.03),Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero());
  for (int i=0;i<30;++i) integration.push_back(.005,Eigen::Vector3d(.1,.2,9.8),Eigen::Vector3d(.01,.02,.03));
  IMUFactor factor(&integration); PoseLocalParameterization p;
  checkFactor(factor,{pose(.2,.1),std::vector<double>(9,0.),pose(.25,.11),std::vector<double>(9,0.)},{&p,nullptr,&p,nullptr},.1);
}
TEST(Factors,MarginalizationPrior) {
  auto original=pose(.2,.4); auto current=pose(.23,.45);
  MarginalizationInfo info;
  info.n=6; info.m=0; info.keep_block_size={7}; info.keep_block_idx={0}; info.keep_block_data={original.data()};
  info.linearized_jacobians=Eigen::Matrix<double,6,6>::Identity();
  info.linearized_residuals=Eigen::VectorXd::Zero(6);
  MarginalizationFactor factor(&info); PoseLocalParameterization p;
  checkFactor(factor,{current},{&p});
  for (int i=3;i<7;++i) current[i]=-current[i];
  checkFactor(factor,{current},{&p});
}

#include "../vins_estimator/src/initial/solve_5pts.h"
TEST(Initialization,DegenerateCorrespondencesAreRejected) {
  MotionEstimator estimator; Eigen::Matrix3d rotation; Eigen::Vector3d translation;
  std::vector<std::pair<Eigen::Vector3d,Eigen::Vector3d>> correspondences(20,{Eigen::Vector3d(0.,0.,1.),Eigen::Vector3d(0.,0.,1.)});
  EXPECT_FALSE(estimator.solveRelativeRT(correspondences,rotation,translation));
}
TEST(Initialization,RelativePoseUsesUpstreamOpenCv) {
  MotionEstimator estimator;
  const Eigen::Matrix3d R=Eigen::AngleAxisd(.07,Eigen::Vector3d::UnitY()).toRotationMatrix();
  const Eigen::Vector3d t(.2,.03,.01);
  std::mt19937 random(42); std::uniform_real_distribution<double> xy(-1.,1.), depth(3.,6.);
  std::vector<std::pair<Eigen::Vector3d,Eigen::Vector3d>> correspondences;
  for (int i=0;i<80;++i) {
    const Eigen::Vector3d first(xy(random),xy(random),depth(random));
    const Eigen::Vector3d second=R*first+t;
    correspondences.emplace_back(first/first.z(),second/second.z());
  }
  Eigen::Matrix3d result; Eigen::Vector3d translation;
  ASSERT_TRUE(estimator.solveRelativeRT(correspondences,result,translation));
  EXPECT_LT((result-R.transpose()).norm(),1e-3);
  EXPECT_GT(translation.normalized().dot((-R.transpose()*t).normalized()),.99);
}
