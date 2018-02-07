#!/bin/sh

VERSION=` find . -name "*.ino" | xargs -n1 grep -oP "(?<=const char VERSION\[\]=\").*(?=\")"`

git archive -o ARDUINO-$VERSION.tar.gz HEAD
