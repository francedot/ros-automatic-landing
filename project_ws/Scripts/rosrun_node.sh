#!/bin/bash

# usage ./rosrun_node pkg_name node_name
# Need to be on the ws folder

if [ $# -lt 2 ]
then
	echo "Numero di argomenti sbagliato! $#"
    echo "usage ./rosrun_node pkg_name node_name"
	exit 1
fi

pkg_name="$1"
node_name="$2"

source ./devel/setup.bash

rosrun "$pkg_name" "$node_name"

