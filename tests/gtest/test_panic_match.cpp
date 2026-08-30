#include <gtest/gtest.h>

extern "C" {
#include "panic_match.h"
}

TEST(PanicMessageMatch, MatchesWithoutCaseSensitivity)
{
    constexpr char message[] = "SEDSNet ALLOCator exhausted";
    EXPECT_TRUE(fc_panic_message_contains(message, sizeof(message) - 1u, "alloc"));
    EXPECT_TRUE(fc_panic_message_contains(message, sizeof(message) - 1u, "ALLOC"));
    EXPECT_FALSE(fc_panic_message_contains(message, sizeof(message) - 1u, "mutex"));
}

TEST(PanicMessageMatch, RespectsExplicitMessageLength)
{
    constexpr char message[] = "ok memory";
    EXPECT_FALSE(fc_panic_message_contains(message, 2u, "memory"));
    EXPECT_TRUE(fc_panic_message_contains(message, sizeof(message) - 1u, "memory"));
}

TEST(PanicMessageMatch, RejectsInvalidInputs)
{
    EXPECT_FALSE(fc_panic_message_contains(nullptr, 4u, "oom"));
    EXPECT_FALSE(fc_panic_message_contains("oom", 0u, "oom"));
    EXPECT_FALSE(fc_panic_message_contains("oom", 3u, nullptr));
    EXPECT_FALSE(fc_panic_message_contains("oom", 3u, ""));
}
