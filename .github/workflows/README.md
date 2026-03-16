# GitHub Actions Workflows

This document describes the CI/CD workflows in this repository.

## `build.yml` — Build

**Triggers:** Push to any branch (excluding `@pybricks/**` tags) and all pull requests.

This is the main continuous integration workflow. It runs the following jobs in parallel (where possible):

| Job | Description |
|-----|-------------|
| `upload_pr_number` | On pull requests only: saves the PR number as an artifact so `pr-artifact.yml` can post a comment with download links. |
| `mpy_cross` | Builds the MicroPython cross-compiler (`mpy-cross`), which is required by downstream jobs. The compiled binary is uploaded as an artifact. |
| `unix_coverage` | Builds the MicroPython Unix port with coverage instrumentation and runs the full MicroPython test suite, including multi-instance network tests. |
| `firmware` | Builds firmware for every supported hub using a matrix strategy: `cityhub`, `essentialhub`, `movehub`, `nxt`, `primehub`, `technichub`, `ev3`, and `buildhat`. On pull requests each commit in the PR is built individually. On non-master branches the firmware size is recorded to Azure Table Storage. On master the build is tagged with a CI build number. The resulting firmware zip is uploaded as an artifact. |
| `virtualhub` | Builds and tests the virtual-hub port (a Linux/host build used for integration testing) and collects code-coverage data. |
| `pbio` | Builds and tests the `pbio` hardware-abstraction library on the host, builds its Doxygen documentation, and generates an LCOV coverage report. |

## `format.yml` — Format

**Triggers:** Push or pull request when any of the following paths change: C/H source files under `bricks/`, `lib/pbio/`, `py/`, or `pybricks/`; Python files under `bricks/`, `tests/`, or `tools/`; or the workflow file itself.

Runs `tools/codeformat.py` (which wraps `clang-format` for C and `black`/`isort` for Python) and then executes `git diff --exit-code`. The job fails if the formatter produces any changes, enforcing that all committed code is already properly formatted.

## `pr-artifact.yml` — Link Build Artifacts on Pull Request

**Triggers:** Completion of the `build` workflow.

When a `build` workflow run triggered by a pull request finishes successfully, this workflow posts (or updates) a comment on the pull request with direct download links to all build artifacts (firmware zips, etc.) via [nightly.link](https://nightly.link).

## `release.yml` — Upload Release Assets

**Triggers:** Push of a tag matching `v3.*` or `v4.*`.

Builds firmware for all production hubs (`movehub`, `cityhub`, `technichub`, `primehub`, `essentialhub`, `nxt`, `ev3`) and creates a GitHub release. The CHANGELOG.md is used as the release body. Tags that contain `a`, `b`, or `c` (e.g. `v3.5a1`) are published as pre-releases; all others are published as full releases. Each firmware zip is renamed to `pybricks-<hub>-<tag>.zip` before upload.

## `stats.yml` — Stats

**Triggers:** Push to the `master` branch.

Tracks firmware size over time:
1. Builds firmware for any commits that are not yet in the Azure Table Storage database.
2. Downloads commit metadata (size data) from Azure Table Storage.
3. Generates a static web page visualising firmware size trends.
4. Publishes the resulting pages to a remote server via `lftp`.

## `npm-firmware.yml` — Publish Firmware Package

**Triggers:** Push of a tag matching `@pybricks/firmware/**`.

Builds and publishes the `npm/firmware` JavaScript package to the npm registry. This package is used by the [Pybricks Code](https://code.pybricks.com) web IDE to download and install Pybricks firmware onto hubs.

## `npm-mpy-cross.yml` — Publish mpy-cross WebAssembly Package

**Triggers:** Push of a tag matching `@pybricks/mpy-cross-v6/*`.

Builds `mpy-cross` as a WebAssembly binary using Emscripten and publishes it to the npm registry as the `@pybricks/mpy-cross-v6` package. This enables the Pybricks Code IDE to compile MicroPython bytecode entirely in the browser.
