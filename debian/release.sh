#!/bin/bash
# TODO a 'no commit build' to recover after failure (but this should never fail ;)
# TODO an increment PACKAGE_VERSION build

PACKAGE_VERSION=0
DIST="unstable"
ID="Romain Janvier <romain.janvier@univ-orleans.fr>"
TIMESTAMP=$(date -R)

CURR_UPSTREAM_VERSION=$(git describe --always --tags --long | sed -e 's/v//g' -e 's/-/+/g')
CURR_HASH=${CURR_UPSTREAM_VERSION##*+g}
PREV_UPSTREAM_VERSION=$(dpkg-parsechangelog --show-field Version)
PREV_HASH=${PREV_UPSTREAM_VERSION%%-*}; PREV_HASH=${PREV_HASH##*+g}

# Collect all [Debian] commits and no [NoChangelog] ]PREV_HASH, CUR_HASH]
# Find the children of a commit is too difficult with git, so we will trim [PREV_HASH] by a [NoChangeLog]
HEADER="cloudcompare ($CURR_UPSTREAM_VERSION-$PACKAGE_VERSION) $DIST; urgency=low"
CHANGESET=$(git log --grep='\[Debian\]' --oneline $PREV_HASH..$CURR_HASH | cut -c 9- | sed -e 's/^/  /' -e 's/\[Debian\]/\-/g' -e '/\[NoChangelog\]/d' -e 's/ ([0-9]*)//g')
FOOTER=" -- $ID  $TIMESTAMP"
CHANGELOG_ENTRY="${HEADER}\n\n${CHANGESET}\n\n${FOOTER}\n"

echo "${CHANGELOG_ENTRY}\n$(cat debian/changelog)" > debian/changelog
git commit -am "[Debian] Update changelog for $CURR_UPSTREAM_VERSION [NoChangelog]"


Build () {
	#TODO: test parameters $1 = dist $2 = arch
	DIST=$1;ARCH=$2
	CHROOTDIR=/var/cache/pbuilder/base-${DIST}-${ARCH}.cow/
	sudo cowbuilder --update --basepath ${CHROOTDIR}
	gbp buildpackage --git-dist=${DIST} --git-arch=${ARCH} --git-pbuilder --git-debian-branch=${DIST} --git-submodules	
	#TODO: build (-uc -us ?)
	#TODO: return status
}

Upload () {
	#TODO: test parameters $1 = dist $2 = arch
	#TODO: create version on BT
	DIST=$1;ARCH=$2
	echo uploading ${DIST} ${ARCH}
}

DISTLIST=( unstable testing xenial yakkety )

for DIST in $DISTLIST
do
	git checkout -b $DIST
	#TODO: Template substitution + commit --amend 
	# eg. DISTNAME in changelog etc...
	Build $DIST amd64
	Build $DIST i386
	#TODO: check return values and update status
	git checkout Debian
	git branch -D $DIST
done

#TODO: chroot cache + chroot in user space ?
