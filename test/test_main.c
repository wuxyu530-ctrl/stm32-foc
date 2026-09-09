#include <stdio.h>
#include "test_framework.h"

/* 全局计数器的实际定义（头文件里只是 extern 声明） */
int g_test_count = 0;
int g_fail_count = 0;

/* 各测试组的入口（定义在对应的 test_*.c 里） */
void test_transform_all(void);
void test_svpwm_all(void);

static void test_sanity(void)
{
    CHECK(1 + 1 == 2, "基础算术都错了？");
    CHECK_NEAR(0.1 + 0.2, 0.3, 1e-9, "浮点近似比较");
}

int main(void)
{
    RUN_TEST(test_sanity);
    test_transform_all();
    test_svpwm_all();

    printf("\n===== %d 项检查，%d 项失败 =====\n",
            g_test_count, g_fail_count);

    return (g_fail_count == 0) ? 0 : 1;
}