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

# my source files
SOURCES = $(wildcard $(SRC_DIR)/*.cpp)

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
CXXFLAGS = -std=c++20 -I$(IMGUI_DIR) -I$(IMGUI_DIR)/backends -I/usr/local/tango/include/tango
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
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp | $(GLFW34_LIB)
	$(CXX) $(CXXFLAGS) -c -MMD -o $@ $<

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

clean:
	rm -f $(EXE) $(OBJS)

clean-glfw:
	rm -rf $(GLFW34_DIR)
	rm -f $(DEPENDS)
