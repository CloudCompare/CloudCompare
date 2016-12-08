#!/bin/bash
#oses = [trusty etc...]

CURR_VERSION=$(git describe --always --tags --long | sed 's/v//g')
CURR_HASH=${CURR_VERSION##*-g}
PREV_VERSION=$(dpkg-parsechangelog --show-field Version)
PREV_HASH=${PREV_VERSION##*-}
#collect all [Debian] commits and no [NoChangelog] ]PREV_HASH, CUR_HASH]
#finding the children of a commit is too difficult with git, so we will trim [PREV_HASH] by a [NoChangeLog]
CHANGESET=$(git shortlog --grep='\[Debian\]' --oneline $PREV_HASH..$CURR_HASH | sed -e 's/\[Debian\]//g' -e '/\[NoChangelog\]/d' -e 's/ ([0-9]*)//g')
echo $CHANGESET
#Commit: "Update changelog for $CURR_VERSION [NoChangelog]"
#creating a new branch for each OS, in each branch
	#looking for specific rules etc... and make substitution in a changelog
	#commit (--amend ?)
	#Update chroot
	#build (-uc -us ?)
	#Upload to bintray

#TODO chroot cache + chroot no credentials
