#!/bin/bash

# usage ./create_ws workspace_name

if [ $# -ne 1 ]
then
	echo "Numero di argomenti sbagliato! $#"
	echo "usage ./create_ws workspace_name"
	exit 1
fi

ws_name="$1"

# setup bash
source /opt/ros/indigo/setup.bash

src_path=$ws_name
src_path+='/src'

mkdir -p $src_path
echo 'Created '$src_path

cd $src_path

catkin_init_workspace
echo 'Workspace succesfully'"$ws_name"' initialized'

sleep 2

cd ..
catkin_make

sleep 2

source devel/setup.bash