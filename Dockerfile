# Base image: Debian 9 (Stretch) for legacy compatibility with libadamapi
FROM debian:9-slim

# Fix repositories for EOL Debian 9
RUN echo 'deb http://archive.debian.org/debian stretch main' > /etc/apt/sources.list && \
    echo 'deb http://archive.debian.org/debian-security stretch/updates main' >> /etc/apt/sources.list

# Enable armhf architecture (matches legacy script)
RUN dpkg --add-architecture armhf

# Install build tools and cross-compiler
# Added libc6-dev-armhf-cross explicitly to ensure linker finds libraries
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    git \
    make \
    automake \
    libtool \
    gcc-arm-linux-gnueabihf \
    libc6-dev-armhf-cross \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# Build libmodbus once
WORKDIR /tmp
RUN git clone https://github.com/stephane/libmodbus.git && \
    cd libmodbus && \
    ./autogen.sh && \
    ./configure --host=arm-linux-gnueabihf --enable-static --disable-shared --without-documentation --prefix=/usr/local && \
    make -C src install && \
    cd .. && \
    rm -rf libmodbus

# Set working directory for the project
WORKDIR /work
