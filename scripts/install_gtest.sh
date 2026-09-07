#!/usr/bin/env bash
#
# Build GoogleTest + GoogleMock and install them into a prefix.
#
# Taproot's unit-test build links against a system-wide gtest/gmock -- upstream's CI image
# ships them preinstalled, so nothing in this repo ever installs them. Windows has no such
# system copy, which is why `scons build-tests` dies on `fatal error: gtest/gtest.h`.
#
# gtest is header-and-library, not header-only, so it has to be built by the *same* compiler
# and C++ standard that SCons uses for the hosted build. Grabbing a prebuilt binary from
# somewhere else compiles fine and then fails at link time with mangled-name errors.
#
# Usage:
#     scripts/install_gtest.sh [prefix]      # default: $HOME/.northstar-gtest
#
# It prints the two lines to export afterwards. Set GTEST_ENV_FILE to have those same
# KEY=VALUE lines appended to a file instead (CI points it at $GITHUB_ENV).

set -euo pipefail

GTEST_VERSION="v1.14.0"
# Must match the standard in taproot/sim-modm/*/modm/SConscript.
CXX_STANDARD="c++20"

PREFIX="${1:-$HOME/.northstar-gtest}"
: "${CXX:=g++}"
: "${AR:=ar}"

# gcc on Windows wants native paths in its search-path variables, and separates them with
# ';' rather than ':' (a bare 'C:' would otherwise read as a path of its own).
if command -v cygpath >/dev/null 2>&1; then
    native_path() { cygpath -w "$1"; }
    PATH_SEP=";"
else
    native_path() { printf '%s' "$1"; }
    PATH_SEP=":"
fi

if [ -f "$PREFIX/lib/libgmock_main.a" ]; then
    echo "GoogleTest already built at $PREFIX (delete that directory to rebuild)."
else
    echo "Building GoogleTest $GTEST_VERSION with $($CXX --version | head -1)"

    work="$(mktemp -d)"
    trap 'rm -rf "$work"' EXIT

    git clone --quiet --depth 1 --branch "$GTEST_VERSION" \
        https://github.com/google/googletest.git "$work/src"

    gt="$work/src/googletest"
    gm="$work/src/googlemock"
    mkdir -p "$PREFIX/include" "$PREFIX/lib" "$work/obj"

    # gtest ships "fused" all-in-one sources, so the four libraries taproot links
    # (see GTEST_LIBS in taproot/SConscript) are four compiles and four archives.
    # No CMake needed, and no chance of it picking a different compiler than we did.
    "$CXX" -std="$CXX_STANDARD" -O2 -isystem "$gt/include" -I"$gt" \
        -c "$gt/src/gtest-all.cc"  -o "$work/obj/gtest-all.o"
    "$CXX" -std="$CXX_STANDARD" -O2 -isystem "$gt/include" -I"$gt" \
        -c "$gt/src/gtest_main.cc" -o "$work/obj/gtest_main.o"
    "$CXX" -std="$CXX_STANDARD" -O2 -isystem "$gm/include" -I"$gm" \
        -isystem "$gt/include" -I"$gt" \
        -c "$gm/src/gmock-all.cc"  -o "$work/obj/gmock-all.o"
    "$CXX" -std="$CXX_STANDARD" -O2 -isystem "$gm/include" -I"$gm" \
        -isystem "$gt/include" -I"$gt" \
        -c "$gm/src/gmock_main.cc" -o "$work/obj/gmock_main.o"

    "$AR" rcs "$PREFIX/lib/libgtest.a"       "$work/obj/gtest-all.o"
    "$AR" rcs "$PREFIX/lib/libgtest_main.a"  "$work/obj/gtest_main.o"
    "$AR" rcs "$PREFIX/lib/libgmock.a"       "$work/obj/gmock-all.o"
    "$AR" rcs "$PREFIX/lib/libgmock_main.a"  "$work/obj/gmock_main.o"

    rm -rf "$PREFIX/include/gtest" "$PREFIX/include/gmock"
    cp -r "$gt/include/gtest" "$gm/include/gmock" "$PREFIX/include/"

    echo "Installed gtest, gtest_main, gmock and gmock_main into $PREFIX"
fi

# CPLUS_INCLUDE_PATH / LIBRARY_PATH are gcc's own search-path variables, so nothing in the
# build files has to know about any of this: SConstruct hands os.environ straight to the
# compiler. Keep whatever was already there.
inc="$(native_path "$PREFIX/include")${CPLUS_INCLUDE_PATH:+$PATH_SEP$CPLUS_INCLUDE_PATH}"
lib="$(native_path "$PREFIX/lib")${LIBRARY_PATH:+$PATH_SEP$LIBRARY_PATH}"

if [ -n "${GTEST_ENV_FILE:-}" ]; then
    printf 'CPLUS_INCLUDE_PATH=%s\n' "$inc" >> "$GTEST_ENV_FILE"
    printf 'LIBRARY_PATH=%s\n'       "$lib" >> "$GTEST_ENV_FILE"
else
    echo
    echo "Add these to the shell you build from:"
    echo
    printf '    export CPLUS_INCLUDE_PATH="%s"\n' "$inc"
    printf '    export LIBRARY_PATH="%s"\n'       "$lib"
fi
