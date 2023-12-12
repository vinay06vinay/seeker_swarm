#include <gtest/gtest.h>

void function_not_used() {
  int notUsed = 3;
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
