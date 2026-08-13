# ==============================================================================
# Surface grinding controller build
# ==============================================================================
# main.cpp coordinates one session. The files in src/ provide configuration,
# control, runtime interaction, and reporting. The calibration utilities are
# built separately when required.

CXX := g++
TARGET := surface_grinding_controller
BUILD := build

FRANKA_DIR := /home/hm-panda/libfranka
EIGEN_INC := /usr/include/eigen3

CXXFLAGS := -std=c++14 -O2 -Wall -Wextra \
            -Iinclude \
            -I$(FRANKA_DIR)/include \
            -I$(FRANKA_DIR)/examples \
            -I$(EIGEN_INC)

LDFLAGS := -L$(FRANKA_DIR)/build \
           -Wl,-rpath,$(FRANKA_DIR)/build

LDLIBS := -lfranka -lpthread

# ------------------------------------------------------------------------------
# Controller sources
# ------------------------------------------------------------------------------

MAIN_SRC := main.cpp

CORE_SRCS := src/config/parameter_io.cpp \
             src/control/math_utils.cpp \
             src/control/cartesian_impedance.cpp \
             src/control/robot_support.cpp \
             src/interface/terminal_output.cpp \
             src/interface/operator_interface.cpp \
             src/control/tool_axis_estimation.cpp \
             src/report/csv_logging.cpp \
             src/report/setup_evaluation.cpp

RUN_SRCS := src/control/impedance_gains.cpp \
            src/runtime/control_loop.cpp \
            src/report/run_logging.cpp

HEADERS := $(wildcard include/*.h)

MAIN_OBJ := $(BUILD)/main.o
CORE_OBJS := $(patsubst %.cpp,$(BUILD)/%.o,$(CORE_SRCS))
RUN_OBJS := $(patsubst %.cpp,$(BUILD)/%.o,$(RUN_SRCS))
EXAMPLES_COMMON_OBJ := $(BUILD)/examples_common.o

# ------------------------------------------------------------------------------
# Calibration utilities
# ------------------------------------------------------------------------------

TOOLS_DIR := tools
MEASURE_PLANE_TARGET := measure_plane
MEASURE_PLANE_SRC := $(TOOLS_DIR)/measure_plane.cpp
MEASURE_PLANE_BIN := $(TOOLS_DIR)/$(MEASURE_PLANE_TARGET)
MEASURE_TOOL_AXIS_TARGET := measure_tool_axis
MEASURE_TOOL_AXIS_SRC := $(TOOLS_DIR)/measure_tool_axis.cpp
MEASURE_TOOL_AXIS_BIN := $(TOOLS_DIR)/$(MEASURE_TOOL_AXIS_TARGET)
MEASURE_TOOL_AXIS_AUTO_TARGET := measure_tool_axis_auto
MEASURE_TOOL_AXIS_AUTO_SRC := $(TOOLS_DIR)/measure_tool_axis_auto.cpp
MEASURE_TOOL_AXIS_AUTO_BIN := $(TOOLS_DIR)/$(MEASURE_TOOL_AXIS_AUTO_TARGET)
TOOL_OBJS := $(CORE_OBJS) $(EXAMPLES_COMMON_OBJ)

.PHONY: all run run_measure_plane run_measure_tool_axis \
        run_measure_tool_axis_auto check clean \
        $(MEASURE_PLANE_TARGET) $(MEASURE_TOOL_AXIS_TARGET) \
        $(MEASURE_TOOL_AXIS_AUTO_TARGET)

all: $(TARGET)

# ------------------------------------------------------------------------------
# Compilation and linking
# ------------------------------------------------------------------------------

$(BUILD)/%.o: %.cpp $(HEADERS)
	mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(EXAMPLES_COMMON_OBJ): $(FRANKA_DIR)/examples/examples_common.cpp
	mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(TARGET): $(MAIN_OBJ) $(CORE_OBJS) $(RUN_OBJS) $(EXAMPLES_COMMON_OBJ)
	$(CXX) $^ -o $@ $(LDFLAGS) $(LDLIBS)

$(MEASURE_PLANE_TARGET): $(MEASURE_PLANE_BIN)
$(MEASURE_PLANE_BIN): $(MEASURE_PLANE_SRC) $(TOOL_OBJS) $(HEADERS)
	$(CXX) $(CXXFLAGS) $(MEASURE_PLANE_SRC) $(TOOL_OBJS) -o $@ $(LDFLAGS) $(LDLIBS)

$(MEASURE_TOOL_AXIS_TARGET): $(MEASURE_TOOL_AXIS_BIN)
$(MEASURE_TOOL_AXIS_BIN): $(MEASURE_TOOL_AXIS_SRC) $(TOOL_OBJS) $(HEADERS)
	$(CXX) $(CXXFLAGS) $(MEASURE_TOOL_AXIS_SRC) $(TOOL_OBJS) -o $@ $(LDFLAGS) $(LDLIBS)

$(MEASURE_TOOL_AXIS_AUTO_TARGET): $(MEASURE_TOOL_AXIS_AUTO_BIN)
$(MEASURE_TOOL_AXIS_AUTO_BIN): $(MEASURE_TOOL_AXIS_AUTO_SRC) $(TOOL_OBJS) $(RUN_OBJS) $(HEADERS)
	$(CXX) $(CXXFLAGS) $(MEASURE_TOOL_AXIS_AUTO_SRC) $(TOOL_OBJS) $(RUN_OBJS) -o $@ $(LDFLAGS) $(LDLIBS)

# ------------------------------------------------------------------------------
# Common commands
# ------------------------------------------------------------------------------

run: $(TARGET)
	./$(TARGET)

run_measure_plane: $(MEASURE_PLANE_BIN)
	./$(MEASURE_PLANE_BIN)

run_measure_tool_axis: $(MEASURE_TOOL_AXIS_BIN)
	./$(MEASURE_TOOL_AXIS_BIN)

run_measure_tool_axis_auto: $(MEASURE_TOOL_AXIS_AUTO_BIN)
	./$(MEASURE_TOOL_AXIS_AUTO_BIN)

check: $(TARGET)
	ldd ./$(TARGET) | grep franka

clean:
	rm -rf $(BUILD)
	rm -f $(TARGET) $(MEASURE_PLANE_BIN) $(MEASURE_TOOL_AXIS_BIN) \
	      $(MEASURE_TOOL_AXIS_AUTO_BIN)
