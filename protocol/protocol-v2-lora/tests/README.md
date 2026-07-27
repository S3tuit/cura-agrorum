# Protocol v2 LoRa tests

These tests keep the generated firmware C codec and receiver Python codec
compatible. They check exact wire bytes, shared validation behavior, generated
file freshness, C memory safety, and broader input spaces.

## Philosophy

- Golden vectors are small, manually reviewed protocol examples. Expected
  bytes must not be calculated by the implementation under test.
- Behavioral tests run against both generated codecs through the same
  interface. Acceptance, rejection, and serialization must agree.
- Explicit negative tests document known protocol constraints. Hypothesis and
  libFuzzer explore combinations that were not anticipated manually.
- Native C tests focus on unsafe boundaries such as null pointers, incorrect
  buffer sizes, out-of-bounds access, and undefined behavior. They do not
  duplicate every semantic test already shared with Python.
- Each layer catches a different class of defect; passing one layer does not
  replace the others.

## Responsibilities

- `conftest.py`: loads golden vectors and the generated Python codec, compiles
  the generated C codec as a temporary shared library, and exposes both through
  a common pytest interface.
- `test_generation.py`: runs the generator in `--check` mode and fails when
  committed generated files are stale.
- `test_golden.py`: checks the reviewed header, nonce, reading, and ACK vectors
  against both codecs.
- `test_validation.py`: applies shared negative and classification tests to
  both codecs.
- `test_properties.py`: uses Hypothesis for C/Python agreement, round trips,
  integer boundaries, valid readings, and arbitrary reading bodies.
- `test_c_sanitized.py` and `c/test_codec_sanitized.c`: compile and execute the
  standalone C boundary harness with ASan and UBSan.
- `test_libfuzzer.py` and `c/fuzz_codec.c`: compile the C fuzz target with
  libFuzzer, ASan, and UBSan, then perform randomized runs over codec
  operations and invariants.

The libFuzzer test reads and extends the ignored local corpus at
`.fuzz-corpus/codec`. Interesting inputs therefore survive between pytest runs.
Set `FUZZ_SEED` to a previously reported seed to repeat its mutation sequence
against the same corpus.

## Running

From the repository root:

```sh
python -m venv .venv
. .venv/bin/activate
python -m pip install -r protocol/protocol-v2-lora/requirements-test.txt
python -m pytest protocol/protocol-v2-lora/tests
```

Pytest orchestrates all Python and native tests; no separate build command is
needed. A C compiler with ASan/UBSan support and Clang with libFuzzer support
must also be installed. `CC` overrides the normal C compiler and `FUZZ_CC`
overrides the fuzz compiler.

Any individual test file can be passed to the same `python -m pytest` command
while developing.
