# Build

FROM alpine:3.19 as builder

RUN apk update && \
    apk --no-cache add \
        cmake \
        g++ \
        make \
        linux-headers

ADD CMakeLists.txt FindCluonMsc.cmake /opt/sources/
ADD src /opt/sources/src

RUN mkdir /opt/build /opt/out && \
    cd /opt/build && \
    cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/opt/out \
      /opt/sources && \
    make && make install


FROM alpine:3.19

#RUN apk update && \
#    apk --no-cache add \
#        some-runtime-package

COPY --from=builder /opt/out/ /usr
ENTRYPOINT ["/usr/bin/robot-other-kiwi-steering-service"]
