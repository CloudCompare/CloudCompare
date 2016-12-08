#!/bin/bash
#TODO a 'no commit build' to recover after failure (but ths should never fail ;)
#TODO an increment PACKAGE_VERSION build

PACKAGE_VERSION=0
DIST="unstable"
ID="Romain Janvier <romain.janvier@univ-orleans.fr>"
TIMESTAMP=$(date -R)
CURR_UPSTREAM_VERSION=$(git describe --always --tags --long | sed -e 's/v//g' -e 's/-/+/g')
CURR_HASH=${CURR_VERSION##*+g}
#PREV_UPSTREAM_VERSION=$(dpkg-parsechangelog --show-field Version | sed -e 's/-/+/g')
#PREV_HASH=${PREV_VERSION##*+g}
#echo $CURR_VERSION
PREV_HASH=5018adb

#collect all [Debian] commits and no [NoChangelog] ]PREV_HASH, CUR_HASH]
#finding the children of a commit is too difficult with git, so we will trim [PREV_HASH] by a [NoChangeLog]
HEADER="cloudcompare ($CURR_UPSTREAM_VERSION-$PACKAGE_VERSION) $DIST; urgency=low"
CHANGESET=$(git log --grep='\[Debian\]' --oneline $PREV_HASH..$CURR_HASH | cut -c 9- | sed -e 's/^/  /' -e 's/\[Debian\]/\-/g' -e '/\[NoChangelog\]/d' -e 's/ ([0-9]*)//g')
FOOTER=" -- $ID  $TIMESTAMP"
CHANGELOG_ENTRY="$HEADER\n\n$CHANGESET\n\n$FOOTER\n"

echo "$CHANGELOG_ENTRY"
#git commit -am "[Debian] Update changelog for $CURR_VERSION [NoChangelog]"

#oses = [trusty, xenial, wheezy etc...]
#create a new branch for each OS, in each branch
	#looking for specific rules etc... and make substitution in a changelog
	#commit (--amend ?)
	#Update the chroots (check for chroot ?)
	#build (-uc -us ?)
	#Upload to bintray

#TODO chroot cache + chroot no credentials
