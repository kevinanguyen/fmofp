# FMOFP OFP (C++) — rules for every contributor, human or Claude

This directory is the flight software: the C++ that replaces FMOFP's Python flight
logic, one process per subsystem, talking only over the simulated MIL-STD-1553B bus.
Everything under `B20SS/ofp/` follows these rules. The compiler flags in
`cmake/flight_code.cmake`, cppcheck, and review enforce them. The Python side
(displays, tools, tests) is not held to them.

Read `B20SS/icd/README.md` before touching anything that goes on the bus.

## Language

- C++17, no compiler extensions. GCC and Clang only.
- Flight targets build with `-fno-exceptions -fno-rtti`: no `try`/`throw`/`catch`,
  no `dynamic_cast`, no `typeid`.
- Every warning is an error. Never add `-Wno-*`, a pragma, or a cast whose only
  purpose is to silence a warning. Fix the code.

## Memory

- No heap allocation after initialisation. In flight code that means no `new`,
  `delete`, `malloc`, `std::vector`, `std::string`, `std::map`, `std::function`,
  or anything else that grows. Use `std::array`, fixed-capacity containers with a
  count, and `std::string_view` / `const char*` for names.
- Every array has a compile-time maximum. Every loop has a fixed upper bound.
- No recursion.

## Interfaces

- `B20SS/icd/fmofp_icd.xml` is the only definition of a bus message. Message
  structs, encoders and decoders are generated from it at build time
  (target `ofp::icd`). Never hand-write a message layout, and never edit a
  generated file.
- Wire fields are built with explicit shifts and masks (`fmofp/bus/word.hpp`).
  Never overlay bit-fields, unions or packed structs on bus words.
- Use `<cstdint>` types wherever a size matters: a bus word is `std::uint16_t`.
  No plain `int` in a wire or interface type.

## Errors and data validity

- A function that can fail returns `fmofp::Status` (`fmofp/bus/status.hpp`) and is
  `[[nodiscard]]`. The caller handles every value it can get back.
- Every measured or computed value that leaves a module carries its SSM
  (validity). An unknown or stale value is `NO_COMPUTED_DATA`, never a
  plausible-looking number.
- Check floating-point inputs at module boundaries: NaN, infinity, range. Never
  compare doubles with `==`.
- No `-ffast-math`. The same inputs produce the same words on every build.

## Structure

- One subsystem is one class with `init()`, `step()` and `status()`. No work in
  constructors, no construction-order surprises.
- Code called from `step()` (rate-group code) does no I/O, no logging, no
  formatted output, and never blocks.
- Keep functions short (aim for under 60 lines). Check preconditions at the top
  and return a `Status`.

## Testing

- Every module has GoogleTest unit tests in `tests/`. Generated code has generated
  round-trip tests.
- A change is done when `cmake --build` is clean under `-Werror`, `ctest` passes,
  the `cppcheck` target is clean, and on Linux the `linux-asan` preset passes too.
- Tests may use exceptions, the heap and the standard library freely. They are
  not flight code.

## Style

- `clang-format` with the `.clang-format` in this directory.
- `snake_case` functions, variables and members; `PascalCase` types;
  `kPascalCase` constants. `UPPER_CASE` only for enumerators generated from the
  ICD, which mirror the ICD names exactly.
- Comments say why, not what. Refer to the ICD by message id (`FCS_ATTITUDE`) and
  to `B20SS/icd/README.md` by section number.
- No commented-out code. No TODO without a name next to it.

## Building

See `README.md` in this directory. Short version, from an MSYS2 UCRT64 shell on
Windows or any shell on Linux:

```bash
cmake --preset linux-gcc      # or windows-ucrt64
cmake --build --preset linux-gcc
ctest --preset linux-gcc
```
