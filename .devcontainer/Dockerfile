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
# The -dev packages normally ship the static libraries; if a release ever ships sources
# only, build and install them from /usr/src/googletest instead. On Linux these land in
# gcc's default search paths, so `scons run-tests` needs no extra environment (unlike
# Windows -- see scripts/install_gtest.sh).
RUN apt-get update && apt-get install -y --no-install-recommends \
        libgtest-dev libgmock-dev cmake \
    && if ! ls /usr/lib/*/libgtest.a >/dev/null 2>&1 && [ -d /usr/src/googletest ]; then \
         cmake -S /usr/src/googletest -B /tmp/gtest-build \
         && cmake --build /tmp/gtest-build -j"$(nproc)" \
         && cmake --install /tmp/gtest-build \
         && rm -rf /tmp/gtest-build; \
       fi \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir pipenv clang-format==18.1.8

RUN git config --system core.autocrlf false \
    && git config --system core.eol lf \
    && git config --system --add safe.directory '*'

RUN useradd -m -s /bin/bash vscode \
    && echo "vscode ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/vscode
USER vscode