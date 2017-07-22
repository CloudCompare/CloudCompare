#!/bin/bash

WORKING_DIR="$1"

docker run -t \
  -v "${WORKING_DIR}":"${WORKING_DIR}" \
  -e LC_ALL="C.UTF-8" \
  -e LANG="C.UTF-8" \
  ubuntu:xenial \
  sh -c "apt-get -qq update && apt-get -y install snapcraft && cd ${WORKING_DIR} && snapcraft && snapcraft push *.snap --release edge"
