#include <gtest/gtest.h>

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  (void)RUN_ALL_TESTS();
  // Always return zero-code and allow PlatformIO to parse results
  return 0;
}