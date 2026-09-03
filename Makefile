.PHONY: test-host test-receiver test-receiver-host test-receiver-hardware \
		test-receiver-hardware-slow test-receiver-hardware-all \
		test-receiver-hardware-destructive test-hardware test-hardware-all \
		test-hardware-slow test-hardware-build

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
RECEIVER_PYTEST := $(abspath .venv/bin/python) -m pytest
RECEIVER_PYTEST_ARGS := -c $(abspath receiver/pytest.ini)
RECEIVER_TEST_ROOT ?=
CONFIRM_RECEIVER_DESTRUCTIVE ?=

test-host:
	CCACHE_DISABLE=1 cmake -S firmware/tests/host -B firmware/build-host \
		-DCMAKE_BUILD_TYPE=Debug -DCMAKE_C_COMPILER_LAUNCHER=
	CCACHE_DISABLE=1 cmake --build firmware/build-host
	ctest --test-dir firmware/build-host --output-on-failure

test-receiver: test-receiver-host

test-receiver-host:
	PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 $(RECEIVER_PYTEST) $(RECEIVER_PYTEST_ARGS) \
		$(abspath receiver/tests/host)

test-receiver-hardware:
	PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 $(RECEIVER_PYTEST) $(RECEIVER_PYTEST_ARGS) \
		$(abspath receiver/tests/hardware) --receiver-hardware \
		-m 'hardware and not slow and not destructive and not rf_peer'

test-receiver-hardware-slow:
	PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 $(RECEIVER_PYTEST) $(RECEIVER_PYTEST_ARGS) \
		$(abspath receiver/tests/hardware) --receiver-hardware \
		-m 'hardware and slow and not destructive and not rf_peer'

test-receiver-hardware-all:
	PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 $(RECEIVER_PYTEST) $(RECEIVER_PYTEST_ARGS) \
		$(abspath receiver/tests/hardware) --receiver-hardware \
		-m 'hardware and not destructive and not rf_peer'

test-receiver-hardware-destructive:
	@if [ "$(CONFIRM_RECEIVER_DESTRUCTIVE)" != "YES" ]; then \
		echo "Set CONFIRM_RECEIVER_DESTRUCTIVE=YES to run destructive receiver tests."; \
		exit 2; \
	fi
	@if [ -z "$(RECEIVER_TEST_ROOT)" ]; then \
		echo "Set RECEIVER_TEST_ROOT to a dedicated marked absolute directory."; \
		exit 2; \
	fi
	PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 $(RECEIVER_PYTEST) $(RECEIVER_PYTEST_ARGS) \
		$(abspath receiver/tests/hardware) --receiver-hardware \
		--confirm-receiver-destructive \
		--receiver-test-root="$(RECEIVER_TEST_ROOT)" \
		-m 'hardware and destructive and not rf_peer'

test-hardware-build:
	idf.py -C $(HARDWARE_TEST_APP) -B $(HARDWARE_TEST_BUILD) build

test-hardware: test-hardware-build
	CURAG_HARDWARE_TEST_SET=fast $(HARDWARE_TEST_PYTEST) $(HARDWARE_TEST_ARGS)

test-hardware-all: test-hardware-build
	CURAG_HARDWARE_TEST_SET=all $(HARDWARE_TEST_PYTEST) $(HARDWARE_TEST_ARGS)

test-hardware-slow: test-hardware-build
	CURAG_HARDWARE_TEST_SET=slow $(HARDWARE_TEST_PYTEST) $(HARDWARE_TEST_ARGS)
