#!/bin/sh
set -e
if test "$CONFIGURATION" = "Debug"; then :
  cd /Users/krisvers/dev/kobalt/build
  make -f /Users/krisvers/dev/kobalt/build/CMakeScripts/ReRunCMake.make
fi
if test "$CONFIGURATION" = "Release"; then :
  cd /Users/krisvers/dev/kobalt/build
  make -f /Users/krisvers/dev/kobalt/build/CMakeScripts/ReRunCMake.make
fi
if test "$CONFIGURATION" = "MinSizeRel"; then :
  cd /Users/krisvers/dev/kobalt/build
  make -f /Users/krisvers/dev/kobalt/build/CMakeScripts/ReRunCMake.make
fi
if test "$CONFIGURATION" = "RelWithDebInfo"; then :
  cd /Users/krisvers/dev/kobalt/build
  make -f /Users/krisvers/dev/kobalt/build/CMakeScripts/ReRunCMake.make
fi

