#!/bin/bash

# usage ./create_pkg workspace_name pkg_name [dep1 dep2 ..]

if [ $# -eq 0 ]
then
	echo "Numero di argomenti sbagliato! $#"
	exit 1
fi

ws_name="$1"
pkg_name="$2"
dep_pkgs=""

# echo "$pkg_name"
# exit 0

i=0
for var in $@
do
    let "i++"    
    # echo "$i"
    # skip ws_name and pkg_name
    if [ $i -le "2" ]
    then
        continue
    fi
    dep_pkgs+="$var "
    # echo "$var"
done

# echo "$ccp_arg"
# exit 0

# src_path=$ws_name
# src_path+='/src'

# cd "$src_path"

cd "src"

# Create a new package named beginner_tutorials with std_msgs, rospy, roscpp depndencies
catkin_create_pkg "$pkg_name" "$dep_pkgs"

sleep 3

# Create src folder
cd "$pkg_name"
mkdir src
mkdir msg
mkdir srv
cd ..

# Build the catkin Package
cd ..
catkin_make

sleep 3

# Add the workspace to your ROS environment you need to source the generated setup file:
. devel/setup.bash