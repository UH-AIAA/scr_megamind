#include <gtest/gtest.h>
#include "example.h"

TEST(MathTest, AddBasic) {
    EXPECT_EQ(add(2, 3), 5);
}