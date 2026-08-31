#
# Cross Platform Makefile
# Compatible with MSYS2/MINGW, Ubuntu 14.04.1 and Mac OS X
#
# You will need GLFW (http://www.glfw.org):
# Linux:
#   apt-get install libglfw-dev
# Mac OS X:
#   brew install glfw
# MSYS2:
#   pacman -S --noconfirm --needed mingw-w64-x86_64-toolchain mingw-w64-x86_64-glfw
#

EXE = bin/NICEcontrol
IMGUI_DIR = lib/imgui
IMPLOT_DIR = lib/implot
IMPLOT_PATCH = implot.patch
FONT_DIR = lib/fonts
SRC_DIR = src
BUILD_DIR = build

##---------------------------------------------------------------------
## .cpp files
##---------------------------------------------------------------------

# my source files. Only the device drivers have a .cpp; the rest of the tree is header only.
SOURCES = $(wildcard $(SRC_DIR)/*.cpp) $(wildcard $(SRC_DIR)/devices/*.cpp)

# add general imgui sources
SOURCES += $(IMGUI_DIR)/imgui.cpp $(IMGUI_DIR)/imgui_demo.cpp $(IMGUI_DIR)/imgui_draw.cpp $(IMGUI_DIR)/imgui_tables.cpp $(IMGUI_DIR)/imgui_widgets.cpp

# add imgui sources specific to GLFW+OpenGL3
SOURCES += $(IMGUI_DIR)/backends/imgui_impl_glfw.cpp $(IMGUI_DIR)/backends/imgui_impl_opengl3.cpp

# add implot sources
SOURCES += $(IMPLOT_DIR)/implot.cpp $(IMPLOT_DIR)/implot_demo.cpp $(IMPLOT_DIR)/implot_items.cpp

##---------------------------------------------------------------------
## .o files
##---------------------------------------------------------------------

# For each .cpp file in SOURCES, create a corresponding .o file in BUILD_DIR
OBJS = $(patsubst %.cpp,$(BUILD_DIR)/%.o,$(notdir $(SOURCES)))
OBJS += $(BUILD_DIR)/imgui_impl_glfw.o $(BUILD_DIR)/imgui_impl_opengl3.o

##---------------------------------------------------------------------
## Libraries
##---------------------------------------------------------------------

LIBS = -lfftw3 -lm -liir -lpi_pi_gcs2 -lnF_interface_x64

# The esd subdevice stack. It reads the EtherCAT distributed clock from the ECS-PCIe/FPGA card.
LIBS += -L$(ESD_DIR)/lib -less

LIB_TANGO_DIR = -L /usr/local/tango/lib
LIBS += $(LIB_TANGO_DIR) -ltango \
					-lomniORB4 \
					-lomniDynamic4 \
					-lCOS4 \
					-lomnithread \
					-lzmq \
					-lpthread \
					-lAdsLib # Beckhoff ADS to communicate with PLC

##---------------------------------------------------------------------
## Dependency files
##---------------------------------------------------------------------

DEPENDS = $(patsubst %.cpp,$(BUILD_DIR)/%.d,$(notdir $(SOURCES)))

##---------------------------------------------------------------------
## Compiler and flags
##---------------------------------------------------------------------

UNAME_S := $(shell uname -s)
LINUX_GL_LIBS = -lGL

# compiler flags
# -I. resolves the vendor headers under lib/, and -I$(SRC_DIR) resolves the project headers, so no include needs a
# relative path.
# The esd subdevice stack is a binary library, thus ESS_ESD_LIBRARY must select the settings of that build.
ESD_DIR = lib/esd

CXXFLAGS = -std=c++20 -I. -I$(SRC_DIR) -I$(IMGUI_DIR) -I$(IMGUI_DIR)/backends -I/usr/local/tango/include/tango
CXXFLAGS += -I$(ESD_DIR)/include -DESS_ESD_LIBRARY
CXXFLAGS += -Ofast -Wall -Wformat -Wextra #-g

##---------------------------------------------------------------------
## OPENGL ES
##---------------------------------------------------------------------

## This assumes a GL ES library available in the system, e.g. libGLESv2.so
# CXXFLAGS += -DIMGUI_IMPL_OPENGL_ES2
# LINUX_GL_LIBS = -lGLESv2

##---------------------------------------------------------------------
## Build flags per platform
##---------------------------------------------------------------------

ifeq ($(UNAME_S), Linux) #LINUX
	ECHO_MESSAGE = "Linux"
	# Build GLFW 3.4 from source to fix BadRRCrtc NULL-deref crash on monitor hotplug.
	# The source is cloned from the upstream 3.4 tag and patched via glfw34_x11monitor.patch.
	GLFW34_DIR = lib/glfw34
	GLFW34_LIB = $(GLFW34_DIR)/build/src/libglfw3.a
	GLFW34_PATCH = glfw34_x11monitor.patch
	CXXFLAGS += -I$(GLFW34_DIR)/include
	LIBS += $(LINUX_GL_LIBS) $(GLFW34_LIB) \
	        -lrt -lm -ldl -lX11 -lXrandr -lXi -lXcursor -lXinerama

	CFLAGS = $(CXXFLAGS)
endif

ifeq ($(UNAME_S), Darwin) #APPLE
	ECHO_MESSAGE = "Mac OS X"
	LIBS += -framework OpenGL -framework Cocoa -framework IOKit -framework CoreVideo
	LIBS += -L/usr/local/lib -L/opt/local/lib -L/opt/homebrew/lib
	#LIBS += -lglfw3
	LIBS += -lglfw

	CXXFLAGS += -I/usr/local/include -I/opt/local/include -I/opt/homebrew/include
	CFLAGS = $(CXXFLAGS)
endif

ifeq ($(OS), Windows_NT)
	ECHO_MESSAGE = "MinGW"
	LIBS += -lglfw3 -lgdi32 -lopengl32 -limm32

	CXXFLAGS += `pkg-config --cflags glfw3`
	CFLAGS = $(CXXFLAGS)
endif

##---------------------------------------------------------------------
## Build rules
##---------------------------------------------------------------------

# Order-only dep on $(GLFW34_LIB) ensures GLFW is cloned before headers are needed.
# -MMD writes header dependencies to .d files; -MP adds a phony target for each
# header so deleting or renaming a header doesn't break the build with a stale
# "No rule to make target" error.
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp | $(GLFW34_LIB)
	$(CXX) $(CXXFLAGS) -c -MMD -MP -o $@ $<

$(BUILD_DIR)/%.o: $(SRC_DIR)/devices/%.cpp | $(GLFW34_LIB)
	$(CXX) $(CXXFLAGS) -c -MMD -MP -o $@ $<

$(BUILD_DIR)/%.o: $(IMGUI_DIR)/%.cpp | $(GLFW34_LIB)
	$(CXX) $(CXXFLAGS) -c -o $@ $<

$(BUILD_DIR)/%.o: $(IMPLOT_DIR)/%.cpp | $(GLFW34_LIB) $(BUILD_DIR)/.implot_patched
	$(CXX) $(CXXFLAGS) -c -o $@ $<

$(BUILD_DIR)/%.o: $(IMGUI_DIR)/backends/%.cpp | $(GLFW34_LIB)
	$(CXX) $(CXXFLAGS) -c -o $@ $<

all: $(EXE)
	@echo Build complete for $(ECHO_MESSAGE)

-include $(DEPENDS)

# $^ deduplicates $(OBJS) (the Makefile lists imgui backends twice).
# $(GLFW34_LIB) is included via $(LIBS); the order-only dep above ensures it's built first.
$(EXE): $(OBJS)
	$(CXX) -o $@ $^ $(CXXFLAGS) $(LIBS)

# Rule so that stale dependency files referencing GLFW headers don't block
# a clean build: rebuild GLFW (which clones the source) to satisfy the header.
$(GLFW34_DIR)/include/GLFW/glfw3.h: $(GLFW34_LIB)
	@:

# Build GLFW 3.4 from source, applying our monitor-hotplug null-check patch.
$(GLFW34_LIB): $(GLFW34_PATCH)
	@if [ ! -f "$(GLFW34_DIR)/CMakeLists.txt" ]; then \
		echo "Cloning GLFW 3.4..."; \
		git clone --depth=1 --branch 3.4 https://github.com/glfw/glfw.git $(GLFW34_DIR); \
	fi
	@git -C $(GLFW34_DIR) apply --check $(CURDIR)/$(GLFW34_PATCH) 2>/dev/null \
		&& git -C $(GLFW34_DIR) apply $(CURDIR)/$(GLFW34_PATCH) \
		|| true
	@cmake -S $(GLFW34_DIR) -B $(GLFW34_DIR)/build \
		-DGLFW_BUILD_EXAMPLES=OFF -DGLFW_BUILD_TESTS=OFF \
		-DGLFW_BUILD_DOCS=OFF -DBUILD_SHARED_LIBS=OFF \
		-DCMAKE_BUILD_TYPE=Release -Wno-dev --log-level=ERROR
	@$(MAKE) -C $(GLFW34_DIR)/build -j$$(nproc) --no-print-directory

# Patch implot: double the drag-handle grab radius and make DragRect fill transparent.
$(BUILD_DIR)/.implot_patched: $(IMPLOT_PATCH)
	@mkdir -p $(BUILD_DIR)
	@git -C $(IMPLOT_DIR) apply --check $(CURDIR)/$(IMPLOT_PATCH) 2>/dev/null \
		&& git -C $(IMPLOT_DIR) apply $(CURDIR)/$(IMPLOT_PATCH) \
		|| true
	@touch $@

# Tests that need no hardware and no Tango connection.
TESTS = test_photometry_regions test_extremum_seeker test_clocks
TEST_EXES = $(addprefix $(BUILD_DIR)/,$(TESTS))

test: $(TEST_EXES)
	@for t in $(TEST_EXES); do echo "--- $$t"; $$t || exit 1; done

$(BUILD_DIR)/test_%: test/test_%.cpp $(wildcard $(SRC_DIR)/*/*.hpp) client/nice_clock.h
	@mkdir -p $(BUILD_DIR)
	$(CXX) -std=c++20 -I. -I$(SRC_DIR) -I$(IMGUI_DIR) -Wall -Wextra -o $@ $<

# The reader of the two clocks, for a check of the record from a shell.
nice-clock-read: $(BUILD_DIR)/nice_clock_read

$(BUILD_DIR)/nice_clock_read: client/nice_clock_read.c client/nice_clock.h
	@mkdir -p $(BUILD_DIR)
	$(CC) -std=gnu17 -Wall -Wextra -o $@ $<

# Checks against a live camera. They need TANGO_HOST set and the FLIR_IR_Camera server running:
#   export TANGO_HOST=localhost:10000
#   make test-camera
# camera_client  the client calls that the FLIR panel makes
# camera_drag    a region write in each frame, as a drag does
# camera_resize  the regions after a change of Width or Height
CAMERA_TESTS = camera_client camera_drag camera_resize
CAMERA_EXES = $(addprefix $(BUILD_DIR)/,$(CAMERA_TESTS))
TANGO_LIBS = -L/usr/local/tango/lib -ltango -lomniDynamic4 -lCOS4 -lomniORB4 -lomnithread -lzmq -lpthread

test-camera: $(CAMERA_EXES)
	@for t in $(CAMERA_EXES); do echo "--- $$t"; $$t || exit 1; done

$(BUILD_DIR)/camera_%: test/camera_%.cpp $(SRC_DIR)/devices/TangoFlirCamInterface.hpp $(SRC_DIR)/data/PhotometryRegions.hpp
	@mkdir -p $(BUILD_DIR)
	$(CXX) -std=c++20 -I. -I$(SRC_DIR) -I$(IMGUI_DIR) -I/usr/local/tango/include/tango -Wall -Wextra -o $@ $< $(TANGO_LIBS)

# The dependency files record the path a source had when it was compiled, so a move leaves them stale.
clean:
	rm -f $(EXE) $(OBJS) $(DEPENDS) $(TEST_EXES) $(CAMERA_EXES) $(BUILD_DIR)/nice_clock_read

clean-glfw:
	rm -rf $(GLFW34_DIR)
	rm -f $(DEPENDS)
