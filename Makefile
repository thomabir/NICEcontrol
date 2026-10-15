# Linux only: Tango, the esd EtherCAT stack and AdsLib are Linux libraries.

EXE = bin/NICEcontrol
IMGUI_DIR = lib/imgui
IMPLOT_DIR = lib/implot
IMPLOT_PATCH = implot.patch
SRC_DIR = src
BUILD_DIR = build

# The esd subdevice stack. It reads the EtherCAT distributed clock from the ECS-PCIe/FPGA card.
ESD_DIR = lib/esd

# GLFW 3.4 from source, with a patch against the BadRRCrtc NULL dereference on a monitor hotplug.
# The source is cloned from the upstream 3.4 tag and patched with glfw34_x11monitor.patch.
GLFW34_DIR = lib/glfw34
GLFW34_LIB = $(GLFW34_DIR)/build/src/libglfw3.a
GLFW34_PATCH = glfw34_x11monitor.patch

TANGO_INCLUDE = -isystem /usr/local/tango/include/tango
TANGO_LIBS = -L/usr/local/tango/lib -ltango -lomniORB4 -lomniDynamic4 -lCOS4 -lomnithread -lzmq -lpthread

# The repository of the Tango device servers. The GUI runs its restart_servers.sh.
TANGO_SERVERS_DIR ?= $(abspath ../TangoServers)

# The project sources. Only the device drivers have a .cpp, and the rest of the tree is header only.
SOURCES = $(wildcard $(SRC_DIR)/*.cpp) $(wildcard $(SRC_DIR)/devices/*.cpp)
SOURCES += $(IMGUI_DIR)/imgui.cpp $(IMGUI_DIR)/imgui_demo.cpp $(IMGUI_DIR)/imgui_draw.cpp $(IMGUI_DIR)/imgui_tables.cpp $(IMGUI_DIR)/imgui_widgets.cpp
SOURCES += $(IMGUI_DIR)/backends/imgui_impl_glfw.cpp $(IMGUI_DIR)/backends/imgui_impl_opengl3.cpp
SOURCES += $(IMPLOT_DIR)/implot.cpp $(IMPLOT_DIR)/implot_demo.cpp $(IMPLOT_DIR)/implot_items.cpp

OBJS = $(patsubst %.cpp,$(BUILD_DIR)/%.o,$(notdir $(SOURCES)))
DEPENDS = $(patsubst %.cpp,$(BUILD_DIR)/%.d,$(notdir $(SOURCES)))

# -I. resolves the vendor headers under lib/, and -I$(SRC_DIR) resolves the project headers, so no include needs a
# relative path.
# The vendor folders are -isystem, thus the compiler gives no warnings for their headers.
# The esd subdevice stack is a binary library, thus ESS_ESD_LIBRARY must select the settings of that build.
CXXFLAGS = -std=c++20 -I. -I$(SRC_DIR) -isystem $(IMGUI_DIR) -isystem $(IMGUI_DIR)/backends $(TANGO_INCLUDE)
CXXFLAGS += -isystem $(GLFW34_DIR)/include -isystem $(ESD_DIR)/include -DESS_ESD_LIBRARY
CXXFLAGS += -Ofast -Wall -Wformat -Wextra
CXXFLAGS += -DNICE_TANGO_SERVERS_DIR=\"$(TANGO_SERVERS_DIR)\"

# Warnings for the project sources only. The vendor sources compile without them.
PROJECT_WARNINGS = -Wshadow -Wconversion -Wnon-virtual-dtor

LIBS = -lfftw3 -lm -liir -lpi_pi_gcs2 -lnF_interface_x64
LIBS += -L$(ESD_DIR)/lib -less
LIBS += $(TANGO_LIBS)
LIBS += -lAdsLib
LIBS += -lGL $(GLFW34_LIB) -lrt -ldl -lX11 -lXrandr -lXi -lXcursor -lXinerama

# The order-only prerequisite $(GLFW34_LIB) clones GLFW before the compiler needs its headers.
# -MMD writes the header dependencies to .d files. -MP adds a phony target for each header, thus a deleted or renamed
# header does not stop the build with "No rule to make target".
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp | $(GLFW34_LIB)
	$(CXX) $(CXXFLAGS) $(PROJECT_WARNINGS) -c -MMD -MP -o $@ $<

$(BUILD_DIR)/%.o: $(SRC_DIR)/devices/%.cpp | $(GLFW34_LIB)
	$(CXX) $(CXXFLAGS) $(PROJECT_WARNINGS) -c -MMD -MP -o $@ $<

$(BUILD_DIR)/%.o: $(IMGUI_DIR)/%.cpp | $(GLFW34_LIB)
	$(CXX) $(CXXFLAGS) -c -o $@ $<

$(BUILD_DIR)/%.o: $(IMPLOT_DIR)/%.cpp | $(GLFW34_LIB) $(BUILD_DIR)/.implot_patched
	$(CXX) $(CXXFLAGS) -c -o $@ $<

$(BUILD_DIR)/%.o: $(IMGUI_DIR)/backends/%.cpp | $(GLFW34_LIB)
	$(CXX) $(CXXFLAGS) -c -o $@ $<

all: $(EXE)
	@echo Build complete

-include $(DEPENDS)

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

# Static analysis of the project sources and of the project headers that they include. .clang-tidy holds the checks.
# clang-tidy 14 cannot parse the C++20 headers of libstdc++ 14, thus TIDY_STDLIB selects an older libstdc++.
TIDY_STDLIB ?= -nostdinc++ -isystem /usr/include/c++/11 -isystem /usr/include/x86_64-linux-gnu/c++/11

tidy: | $(GLFW34_LIB) $(BUILD_DIR)/.implot_patched
	clang-tidy $(filter $(SRC_DIR)/%,$(SOURCES)) -- $(TIDY_STDLIB) $(CXXFLAGS)

# Tests that need no hardware and no Tango connection.
TESTS = test_photometry_regions test_extremum_seeker test_clocks test_script_runner test_tango_widgets
TEST_EXES = $(addprefix $(BUILD_DIR)/,$(TESTS))

test: $(TEST_EXES)
	@for t in $(TEST_EXES); do echo "--- $$t"; $$t || exit 1; done

$(BUILD_DIR)/test_%: test/test_%.cpp $(wildcard $(SRC_DIR)/*/*.hpp) client/nice_clock.h
	@mkdir -p $(BUILD_DIR)
	$(CXX) -std=c++20 -I. -I$(SRC_DIR) -I$(IMGUI_DIR) -Wall -Wextra -o $@ $<

# The widget test draws in an ImGui context, thus it links the ImGui core.
IMGUI_CORE_OBJS = $(addprefix $(BUILD_DIR)/,imgui.o imgui_draw.o imgui_tables.o imgui_widgets.o)

$(BUILD_DIR)/test_tango_widgets: test/test_tango_widgets.cpp $(SRC_DIR)/gui/TangoDeviceWidgets.hpp $(SRC_DIR)/data/TangoAttribute.hpp $(IMGUI_CORE_OBJS)
	@mkdir -p $(BUILD_DIR)
	$(CXX) -std=c++20 -I. -I$(SRC_DIR) -I$(IMGUI_DIR) -Wall -Wextra -o $@ $< $(IMGUI_CORE_OBJS)

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
# camera_sync    the frame sync settings, and the stream after each write
CAMERA_TESTS = camera_client camera_drag camera_resize camera_sync
CAMERA_EXES = $(addprefix $(BUILD_DIR)/,$(CAMERA_TESTS))

test-camera: $(CAMERA_EXES)
	@for t in $(CAMERA_EXES); do echo "--- $$t"; $$t || exit 1; done

$(BUILD_DIR)/camera_%: test/camera_%.cpp $(SRC_DIR)/devices/TangoFlirCamInterface.hpp $(SRC_DIR)/data/PhotometryRegions.hpp
	@mkdir -p $(BUILD_DIR)
	$(CXX) -std=c++20 -I. -I$(SRC_DIR) -I$(IMGUI_DIR) $(TANGO_INCLUDE) -Wall -Wextra -o $@ $< $(TANGO_LIBS)

# Check of the generic Tango device application against a ThorlabsChopper server on the simulated controller.
# The script of the server adds a temporary device to the Tango database and gives its name to the test.
#   export TANGO_HOST=localhost:10000
#   make test-chopper
test-chopper: $(BUILD_DIR)/chopper_app
	$(TANGO_SERVERS_DIR)/ThorlabsChopper/test/run_test.sh $(TANGO_SERVERS_DIR)/DeviceServers/ThorlabsChopper $(BUILD_DIR)/chopper_app

$(BUILD_DIR)/chopper_app: test/chopper_app.cpp $(SRC_DIR)/apps/TangoDeviceApp.hpp $(SRC_DIR)/devices/TangoGenericInterface.hpp $(SRC_DIR)/core/Whiteboard.hpp $(SRC_DIR)/core/Commands.hpp
	@mkdir -p $(BUILD_DIR)
	$(CXX) -std=c++20 -I. -I$(SRC_DIR) -I$(IMGUI_DIR) $(TANGO_INCLUDE) -Wall -Wextra -o $@ $< $(TANGO_LIBS)

# Check against live nanoFaktur controllers on the LAN. The stages move between 2000 urad and 4000 urad.
#   make test-nf NF_SERIALS="1000003878 1000003879"
NF_SERIALS ?= 1000003878 1000003879

test-nf: $(BUILD_DIR)/nf_controller
	$(BUILD_DIR)/nf_controller $(NF_SERIALS)

$(BUILD_DIR)/nf_controller: test/nf_controller.cpp $(SRC_DIR)/devices/nF_EBD_Controller.hpp
	@mkdir -p $(BUILD_DIR)
	$(CXX) -std=c++20 -I. -I$(SRC_DIR) -Ofast -Wall -Wextra -o $@ $< -lnF_interface_x64 -pthread

# The dependency files record the path a source had when it was compiled, so a move leaves them stale.
clean:
	rm -f $(EXE) $(OBJS) $(DEPENDS) $(TEST_EXES) $(CAMERA_EXES) $(BUILD_DIR)/nice_clock_read $(BUILD_DIR)/nf_controller $(BUILD_DIR)/chopper_app

clean-glfw:
	rm -rf $(GLFW34_DIR)
	rm -f $(DEPENDS)
