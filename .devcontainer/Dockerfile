FROM ubuntu:22.04
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3.11 python3-pip python3.11-venv \
    build-essential git wget xz-utils ca-certificates sudo openssh-client \
    && rm -rf /var/lib/apt/lists/*

RUN ARCH="$(uname -m)" \
    && case "$ARCH" in \
         x86_64)  GCC_ARCH=x86_64 ;; \
         aarch64) GCC_ARCH=aarch64 ;; \
         *) echo "unsupported arch: $ARCH" >&2; exit 1 ;; \
       esac \
    && wget -qO /tmp/gcc-arm.tar.bz2 \
       "https://developer.arm.com/-/media/Files/downloads/gnu-rm/10.3-2021.10/gcc-arm-none-eabi-10.3-2021.10-${GCC_ARCH}-linux.tar.bz2" \
    && mkdir -p /opt/gcc-arm \
    && tar xjf /tmp/gcc-arm.tar.bz2 -C /opt/gcc-arm --strip-components=1 \
    && rm /tmp/gcc-arm.tar.bz2
    
ENV PATH="/opt/gcc-arm/bin:${PATH}"

# GoogleTest/GoogleMock, which taproot's unit-test build links against system-wide.
# Built here by the repo's own script so the container, CI and any local host build all get
# the same version compiled the same way -- see the header comment in install_gtest.sh for
# why a prebuilt copy from anywhere else fails at link time.
#
# The prefix is /usr/local deliberately. Its include/ and lib/ are already on host gcc's
# default search paths, so `scons run-tests` needs no CPLUS_INCLUDE_PATH or LIBRARY_PATH.
# Setting those globally would also hand a host x86 include and library directory to
# arm-none-eabi-g++ during the firmware build, which is exactly what we don't want.
COPY scripts/install_gtest.sh /tmp/install_gtest.sh
RUN bash /tmp/install_gtest.sh /usr/local && rm /tmp/install_gtest.sh

# Fail the image build, not the student's first `scons run-tests`, if the above ever stops
# landing where gcc looks.
RUN printf '#include <gtest/gtest.h>\n#include <gmock/gmock.h>\nint main(){return 0;}\n' > /tmp/t.cpp \
    && g++ -std=c++20 /tmp/t.cpp -lgtest -lgmock -lgtest_main -o /tmp/t \
    && rm /tmp/t.cpp /tmp/t

RUN pip3 install --no-cache-dir pipenv clang-format==18.1.8

RUN git config --system core.autocrlf false \
    && git config --system core.eol lf \
    && git config --system --add safe.directory '*'

RUN useradd -m -s /bin/bash vscode \
    && echo "vscode ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/vscode
USER vscode