#!/bin/bash

set -uex

docker images -q nodemcu.build || docker build -f Dockerfile.build -t nodemcu.build .

docker run --rm --name nodemcu-build -u $UID -v $PWD:/src -w /src nodemcu.build \
    make "$@"
docker run --rm --name nodemcu-build -u $UID -v $PWD:/src -w /src nodemcu.build \
    srec_cat -output bin/image.bin -binary bin/0x00000.bin -binary -fill 0xff 0x00000 0x10000 bin/0x10000.bin -binary -offset 0x10000
