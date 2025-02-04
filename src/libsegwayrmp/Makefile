# Makefile for ROS 2 Humble
all: libsegwayrmp

install:
	@cd build && colcon install

uninstall:
	@cd build && colcon uninstall

libsegwayrmp:
	@mkdir -p build
	@mkdir -p bin
	cd build && colcon build --cmake-args -DLIBSEGWAYRMP_BUILD_EXAMPLES=1

clean:
	@cd build && colcon clean
	@rm -rf build bin lib

.PHONY: doc
doc:
	@doxygen doc/Doxyfile
ifeq ($(UNAME),Darwin)
	@open doc/html/index.html
endif

.PHONY: test
test:
	@mkdir -p build
	@mkdir -p bin
	cd build && colcon build --cmake-args -DLIBSEGWAYRMP_BUILD_TESTS=1 -DLIBSEGWAYRMP_BUILD_EXAMPLES=1
	@if test -e bin/libsegwayrmp_tests; then bin/libsegwayrmp_tests; fi

