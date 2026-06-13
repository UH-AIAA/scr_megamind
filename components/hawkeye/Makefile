HAWKEYE_INC_DIR = ./include
HAWKEYE_SRC_DIR = ./src
HAWKEYE_BUILD_DIR = ./build
CMOCKA_DIR = ./cmocka
CMOCKA_BUILD_DIR = $(HAWKEYE_BUILD_DIR)/build-cmocka
TEST_DIR = ./test
LIB = libscr_hawkeye.a

SOURCE_FILES = $(wildcard $(HAWKEYE_SRC_DIR)/*.c)
OBJECT_FILES = $(patsubst $(HAWKEYE_SRC_DIR)/%.c,$(HAWKEYE_BUILD_DIR)/%.o,$(SOURCE_FILES))

# Top-level target: builds library and test
all: lib test_exec

# Compile static library
lib: $(LIB)

$(LIB): $(OBJECT_FILES)
	ar rcs $@ $^

# Compile .c -> .o
$(HAWKEYE_BUILD_DIR)/%.o: $(HAWKEYE_SRC_DIR)/%.c | $(HAWKEYE_BUILD_DIR)
	@mkdir -p $(dir $@)
	gcc -Wall -Werror -I$(HAWKEYE_INC_DIR) -c $< -o $@

# Ensure build directory exists
$(HAWKEYE_BUILD_DIR):
	mkdir -p $(HAWKEYE_BUILD_DIR)

$(CMOCKA_BUILD_DIR):
	mkdir -p $(HAWKEYE_BUILD_DIR)/(CMOCKA_BUILD_DIR)

# Compile and link test executable
test_exec: $(OBJECT_FILES)
	cmake -S $(CMOCKA_DIR) -B $(CMOCKA_BUILD_DIR) -DBUILD_SHARED_LIBS=OFF
	cmake --build $(CMOCKA_BUILD_DIR) --parallel
	gcc -Wall -Werror -I$(HAWKEYE_INC_DIR) -I$(CMOCKA_DIR)/include $(TEST_DIR)/*.c -L$(CMOCKA_BUILD_DIR)/src -L. -lscr_hawkeye -lcmocka -lm -o hawkeye_test

.PHONY: clean
clean:
	rm -rf $(HAWKEYE_BUILD_DIR) $(LIB) hawkeye_test
