#include <gtest/gtest.h>
#include "my_dummy_lib_funct1.hpp"
#include "my_dummy_lib_funct2.hpp"

TEST(dummy_test, this_should_pass) {
  EXPECT_EQ(1, 1);
}

TEST(dummy_test, this_should_pass_too) {
  EXPECT_EQ(function1(3), 103);
}

TEST(dummy_test, this_will_fail) {
  EXPECT_EQ(function2(32), function1(32));
}
