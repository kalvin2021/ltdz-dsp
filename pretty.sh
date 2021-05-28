#!/bin/sh
#
# Author:   Kalvin
#
# File      pretty.sh
#
# Helper shell script to run the astyle source code formatting utility.
#
# Usage:
#
#   $ ./pretty.sh filename
#

ASTYLERC="$(dirname $0)/.astylerc"

astyle --options="$ASTYLERC" $*
