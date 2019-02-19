#!/bin/bash
set -e
for x in *_runtest.sh; do
	echo "Running $x.."
	if ! bash $x &> ${x%.sh}.log; then
		tail ${x%.sh}.log
		echo ERROR
		exit 1
	fi
done

for d in */; do
    if [ -x $d/run-test.sh ]; then
        cd $d
        bash run-test.sh
        cd ..
    fi
done
