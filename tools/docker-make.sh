#!/bin/bash

set -uex

docker images -q nodemcu.build || docker build -f Dockerfile.build -t nodemcu.build .

docker run --rm --name nodemcu-build -u $UID -v $PWD:/src -w /src nodemcu.build make "$@"
