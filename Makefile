# ============ 工具与编译选项 ============
CC      := gcc
CFLAGS  := -std=c11 -O2 -Wall -Wextra -Icore -Itest
LDLIBS  := -lm

# ============ 源文件（自动收集） ============
CORE_SRC := $(wildcard core/*.c)
TEST_SRC := $(wildcard test/*.c)
BUILD    := build

# ============ 目标 ============
.PHONY: all test lib clean
all: test

test: $(BUILD)/run_tests
	@echo "--- 运行单元测试 ---"
	@./$(BUILD)/run_tests

$(BUILD)/run_tests: $(CORE_SRC) $(TEST_SRC) | $(BUILD)
	$(CC) $(CFLAGS) $^ -o $@ $(LDLIBS)

lib: $(BUILD)/libfoc.so
$(BUILD)/libfoc.so: $(CORE_SRC) | $(BUILD)
	$(CC) $(CFLAGS) -fPIC -shared $^ -o $@ $(LDLIBS)

$(BUILD):
	@mkdir -p $(BUILD)

clean:
	@rm -rf $(BUILD)
	@echo "已清理"
