# QR Mapping

## Generating pybind11 stubs

Install `pybind11-stubgen`: https://github.com/sizmailov/pybind11-stubgen


To generate pybind11 stubs, run one of the following commands:

- [CLion] When build target is `cmake-build-debug`:
  ```sh
  pybind11-stubgen qr_mapping.cpp -o cmake-build-debug/src/qr_mapping
  ```
- [CLion] When build target is `cmake-build-release`:
  ```sh
  pybind11-stubgen qr_mapping.cpp -o cmake-build-debug/src/qr_mapping
  ```
- [PDM] When build target is `src`:
  ```sh
  pybind11-stubgen qr_mapping.cpp -o src
  ```
