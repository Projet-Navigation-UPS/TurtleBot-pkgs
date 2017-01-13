#include <gtest/gtest.h>

#include <vector>

#include <base_local_planner/simple_trajectory_generator.h>

namespace base_local_planner {

class TrajectoryGeneratorTest : public testing::Test {
public:
	SimpleTrajectoryGenerator tg;

  TrajectoryGeneratorTest() {
  }

  virtual void TestBody(){}
};
  
}
