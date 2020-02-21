#!/bin/bash
TOOLSDIR=$( dirname "${BASH_SOURCE[0]}" )
if [ "$1" = "-p" ]; then
PRUNE_DIRS=$2
fi
for d in ${PRUNE_DIRS}; do
PRUNE_CMD="${PRUNE_CMD} -path '$d' -prune -o "
done
echo "find . ${PRUNE_CMD} -name '*.c' -o -name '*.cpp' -o -name '*.hpp' -o -name '*.h' -print"
for f in $(find . ${PRUNE_CMD} -path "*.git/*" -prune -o -name '*.c' -print -o -name '*.cpp' -print -o -name '*.hpp' -print -o -name '*.h' -print); do
	echo $f
	astyle --options=${TOOLSDIR}/astylerc --preserve-date --quiet $f
done
