FROM alpine:3.20 AS build
RUN apk add --no-cache \
    g++ \
    cmake

WORKDIR test

COPY src/ ./src
COPY CMakeLists.txt .