#ifndef TEST_FRAMEWORK_H
#define TEST_FRAMEWORK_H

#include <stdio.h>
#include <math.h>

extern int g_test_count;
extern int g_fail_count;

/* 布尔断言 */
#define CHECK(cond, ...) do {                                        \
    g_test_count++;                                                  \
    if (!(cond)) {                                                   \
        g_fail_count++;                                              \
        printf("  \033[31mFAIL\033[0m %s:%d  ", __FILE__, __LINE__); \
        printf(__VA_ARGS__);                                         \
        printf("\n");                                                \
    }                                                                \
} while (0)

/* 浮点近似相等断言 */
#define CHECK_NEAR(actual, expect, tol, ...) do {                    \
    double _a = (double)(actual), _e = (double)(expect);             \
    g_test_count++;                                                  \
    if (fabs(_a - _e) > (double)(tol)) {                             \
        g_fail_count++;                                              \
        printf("  \033[31mFAIL\033[0m %s:%d  ", __FILE__, __LINE__); \
        printf(__VA_ARGS__);                                         \
        printf("  期望 %.6f  实际 %.6f  差 %.2e\n",                  \
                _e, _a, fabs(_a - _e));                               \
    }                                                                \
} while (0)

#define RUN_TEST(fn) do {      \
    printf("[ %s ]\n", #fn);   \
    fn();                      \
} while (0)

#endif /* TEST_FRAMEWORK_H */