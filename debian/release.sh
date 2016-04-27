#!/bin/bash
distrib=$(dpkg-parsechangelog --show-field Distribution)
upstream=$(dpkg-parsechangelog --show-field Version | cut -d- -f1)
packagepart=$(dpkg-parsechangelog --show-field Version | cut -d- -f2)
derivative=$(echo -n $packagepart | tail -c 1)
debian=$(echo -n $packagepart | cut -c1)

for d in trusty willy xenial
do
    git branch $d
    git checkout $d
    sed -i -e 's/archeos/'${d}'/g' -e 's/'${distribi}'/'${d}'/g' debian/changelog
    git commit -am "Release"
    git-buildpackage -S --git-debian-branch=$d
    dput ppa:romain-janvier/cloudcompare  "../release/cloudcompare_"${upstream}"-"${debian}${d}$derivative"_source.changes"
    git checkout master
    git branch -D $d;
done
rm -rf ../release/cloudcompare*

