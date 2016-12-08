#!/bin/bash
#oses = [trusty, xenial, wheezy etc...]
#TODO a no commit build after failure (but ths should never fail ;)
PACKAGE_NAME="cloudcompare"
TIMESTAMP=$(date -R)
CURR_VERSION=$(git describe --always --tags --long | sed -e 's/v//g' -e 's/-/+/g') #TODO: only first occurence
CURR_HASH=${CURR_VERSION##*+g}
#PREV_VERSION=$(dpkg-parsechangelog --show-field Version | sed -e 's/-/+/g')
#PREV_HASH=${PREV_VERSION##*+g}
#echo $CURR_VERSION
#PREV_HASH=1221f69e

#collect all [Debian] commits and no [NoChangelog] ]PREV_HASH, CUR_HASH]
#finding the children of a commit is too difficult with git, so we will trim [PREV_HASH] by a [NoChangeLog]
CHANGESET=$(git shortlog --grep='\[Debian\]' -w76,2,2 $PREV_HASH..$CURR_HASH | sed -e 's/\[Debian\]/\-/g' -e '/\[NoChangelog\]/d' -e 's/ ([0-9]*)//g')
echo "$CHANGESET"

#Commit: git commit -am "[Debian] Update changelog for $CURR_VERSION [NoChangelog]"

#creating a new branch for each OS, in each branch
	#looking for specific rules etc... and make substitution in a changelog
	#commit (--amend ?)
	#Update chroot
	#build (-uc -us ?)
	#Upload to bintray

#TODO chroot cache + chroot no credentials
