# Build
FROM alpine:3.17 as builder
RUN apk update && \
    apk --no-cache add \
        ca-certificates \
        cmake \
        g++ \
        make \
        linux-headers
RUN apk add libcluon --no-cache --repository \
      https://chrberger.github.io/libcluon/alpine/v3.13 --allow-untrusted
ADD . /opt/sources
WORKDIR /opt/sources
RUN mkdir /tmp/build && cd /tmp/build && \
    cmake /opt/sources && \
    make && make test && cp milestone1 /tmp

# Deploy
FROM alpine:3.17
RUN apk update && \
    apk --no-cache add \
        libstdc++
COPY --from=builder /tmp/milestone1 /usr/bin
CMD ["/usr/bin/milestone1"]

