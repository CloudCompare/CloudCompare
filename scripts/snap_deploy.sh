#!/bin/bash

WORKING_DIR="$1"

docker run \
  -v "${WORKING_DIR}":"${WORKING_DIR}" \
  -t ubuntu:xenial \
  sh -c "apt-get -qq update && apt-get -y install snapcraft && cd ${WORKING_DIR} && snapcraft && snapcraft push *.snap --release edge"
