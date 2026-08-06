.PHONY: test-host test-hardware test-hardware-all test-hardware-slow \
	test-hardware-build

PORT ?= /dev/ttyUSB0
HARDWARE_TEST_APP := $(abspath firmware/test_apps/on_device)
HARDWARE_TEST_BUILD := $(HARDWARE_TEST_APP)/build
HARDWARE_TEST_PYTEST := $(abspath .venv/bin/python) -m pytest
HARDWARE_TEST_ARGS := \
	$(HARDWARE_TEST_APP)/pytest_node_persistence.py \
	--embedded-services=esp,idf \
	--app-path=$(HARDWARE_TEST_APP) \
	--build-dir=$(HARDWARE_TEST_BUILD) \
	--target=esp32c6 \
	--port=$(PORT)

test-host:
	CCACHE_DISABLE=1 cmake -S firmware/tests/host -B firmware/build-host \
		-DCMAKE_BUILD_TYPE=Debug -DCMAKE_C_COMPILER_LAUNCHER=
	CCACHE_DISABLE=1 cmake --build firmware/build-host
	ctest --test-dir firmware/build-host --output-on-failure

test-hardware-build:
	idf.py -C $(HARDWARE_TEST_APP) -B $(HARDWARE_TEST_BUILD) build

test-hardware: test-hardware-build
	CURAG_HARDWARE_TEST_SET=fast $(HARDWARE_TEST_PYTEST) $(HARDWARE_TEST_ARGS)

test-hardware-all: test-hardware-build
	CURAG_HARDWARE_TEST_SET=all $(HARDWARE_TEST_PYTEST) $(HARDWARE_TEST_ARGS)

test-hardware-slow: test-hardware-build
	CURAG_HARDWARE_TEST_SET=slow $(HARDWARE_TEST_PYTEST) $(HARDWARE_TEST_ARGS)
