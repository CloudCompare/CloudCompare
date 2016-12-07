#!/bin/bash
#oses = [trusty etc...]
CURR_VERSION=$(git describe --always --tags --long | sed 's/v//g')
CURR_HASH=${CURR_VERSION##*-g}
PREV_VERSION=$(dpkg-parsechangelog --show-field Version)
PREV_HASH=${PREV_VERSION##*-}
echo $CURR_HASH
#collect all [Debian] commits ]PREV_HASH, CUR_HASH]
#git shortlog --grep='\[Debian\]' --oneline
