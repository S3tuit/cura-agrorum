.PHONY: test-host

test-host:
	CCACHE_DISABLE=1 cmake -S firmware/tests/host -B firmware/build-host \
		-DCMAKE_BUILD_TYPE=Debug -DCMAKE_C_COMPILER_LAUNCHER=
	CCACHE_DISABLE=1 cmake --build firmware/build-host
	ctest --test-dir firmware/build-host --output-on-failure
