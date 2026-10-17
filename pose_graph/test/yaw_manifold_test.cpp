#include <gtest/gtest.h>
#include "../src/pose_graph.h"

TEST(YawManifold,WrapsInDegreesAndRecoversLocalIncrement) {
  AngleManifold manifold;
  for (const double x: {-179.,0.,179.}) {
    const double delta=4.;double y=0.,recovered=0.;
    ASSERT_TRUE(manifold.Plus(&x,&delta,&y));
    ASSERT_TRUE(manifold.Minus(&y,&x,&recovered));
    EXPECT_NEAR(recovered,delta,1e-12);
    EXPECT_GE(y,-180.);EXPECT_LE(y,180.);
  }
}
TEST(YawManifold,DerivativesMatchFiniteDifferences) {
  AngleManifold manifold;const double x=37.,eps=1e-7,negative=-eps;
  double plus=0.,minus=0.,jacobian=0.,inverse=0.;
  ASSERT_TRUE(manifold.Plus(&x,&eps,&plus));ASSERT_TRUE(manifold.Plus(&x,&negative,&minus));
  ASSERT_TRUE(manifold.PlusJacobian(&x,&jacobian));ASSERT_TRUE(manifold.MinusJacobian(&x,&inverse));
  EXPECT_NEAR(jacobian,(plus-minus)/(2*eps),1e-7);
  EXPECT_NEAR(jacobian*inverse,1.,1e-12);
}
