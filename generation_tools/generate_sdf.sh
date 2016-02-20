#!/bin/bash

##########################################################################
## Adapted from code written by Nikolaus Wittenstein
## Written by Robert Griffin
##########################################################################

options='fh?'
while getopts $options option ; do
    case $option in
        h  ) usage; exit;;
        \? ) usage; exit;;
        *  ) usage; exit;;
    esac
done

robot=$1
robot=${robot:-escher_hdt}
robot_dir=$robot
if [ $robot = "escher_hdt" ]; then
  robot="escher"
fi

ros_test=`rospack find ${robot}_description | grep "[rospack] Error: package"`

if [ -z "ros_test" ]; then
  echo "Cannot find ROS packages, bailing out."
  exit 1
fi

urdf_dir="`rospack find ${robot}_description`/urdf"
robot_sdf_dir="`rospack find trec_platforms`/gazebo_models"
root_dir="$ASGARD_HOME/Tools/generate_model"

#for KDL
echo -e "*with \nlidar=1  \nfixed_torso=0" > kdl_temp

#for URDF (Ros)
echo -e "*with \nlidar=0  \nfixed_torso=0" > urdf_temp

#for SDF (Gazebo)
echo -e "*with \nlidar=0  \nfixed_torso=0" > sdf_temp
echo -e "*with \nlidar=0  \nfixed_torso=1" > sdf_fixed_temp

for i in kdl_temp urdf_temp sdf_temp sdf_fixed_temp; do
  cat $root_dir/$robot_dir/${robot}_nx_data.txt >> $i

  # reduce_template creates a file named something like "nx_data84". Rename it to something better.
  mv $(./reduce_template.sh $i) $i
done

echo "Generating $robot URDF Models"

# Robot URDF Mesh Model
lua $root_dir/format_model.lua $root_dir/$robot_dir/${robot}_urdf_geometry_mesh.txt $root_dir/$robot_dir/${robot}_urdf_template.txt > mesh_temp
lua $root_dir/format_model.lua urdf_temp mesh_temp > $urdf_dir/${robot}_mesh.xacro

# Robot URDF Bounding Box Model
lua $root_dir/format_model.lua $root_dir/$robot_dir/${robot}_urdf_geometry_bbox.txt $root_dir/$robot_dir/${robot}_urdf_template.txt > bbox_temp
lua $root_dir/format_model.lua urdf_temp bbox_temp > $urdf_dir/${robot}_bbox.xacro


echo "Generating $robot SDF Models"

# Robot SDF Mesh Model
lua $root_dir/format_model.lua $root_dir/$robot_dir/${robot}_sdf_geometry_mesh.txt $root_dir/$robot_dir/${robot}_sdf_template.txt > mesh_temp
lua $root_dir/format_model.lua sdf_temp mesh_temp > $robot_sdf_dir/$robot/${robot}.sdf

# Robot SDF Mesh Fixed Model
lua $root_dir/format_model.lua $root_dir/$robot_dir/${robot}_sdf_geometry_mesh.txt $root_dir/$robot_dir/${robot}_sdf_template.txt > mesh_fixed_temp
lua $root_dir/format_model.lua sdf_fixed_temp mesh_fixed_temp > $robot_sdf_dir/${robot}_fixed/${robot}_fixed.sdf

# Robot SDF Bounding Box Model
lua $root_dir/format_model.lua $root_dir/$robot_dir/${robot}_sdf_geometry_bbox.txt $root_dir/$robot_dir/${robot}_sdf_template.txt > bbox_temp
lua $root_dir/format_model.lua sdf_temp bbox_temp > $robot_sdf_dir/${robot}_bbox/${robot}_bbox.sdf

# Robot SDF Bounding Box Fixed Model
lua $root_dir/format_model.lua $root_dir/$robot_dir/${robot}_sdf_geometry_bbox.txt $root_dir/$robot_dir/${robot}_sdf_template.txt > bbox_fixed_temp
lua $root_dir/format_model.lua sdf_fixed_temp bbox_fixed_temp > $robot_sdf_dir/${robot}_bbox_fixed/${robot}_bbox_fixed.sdf

#KDL Mechanics
echo "Generating $robot KDL Mechanics"
lua $root_dir/format_model.lua kdl_temp $root_dir/$robot_dir/${robot}_kdl_template.txt > $ASGARD_HOME/Platform/gazebo_${robot}/Mechanics/Mechanics.cpp

cp ../../Platform/gazebo_${robot}/Mechanics/Mechanics.cpp $ASGARD_HOME/Platform/$robot/Mechanics/Mechanics.cpp
if [ $robot == "escher" ]; then
    sed -i "s/GazeboEscher/Escher/g" $ASGARD_HOME/Platform/$robot/Mechanics/Mechanics.cpp
    sed -i "s/gazebo_escher/escher/" $ASGARD_HOME/Platform/$robot/Mechanics/Mechanics.cpp
else
    sed -i "s/GazeboThor/Thor/g" $ASGARD_HOME/Platform/$robot/Mechanics/Mechanics.cpp
    sed -i "s/gazebo_thor/thor/" $ASGARD_HOME/Platform/$robot/Mechanics/Mechanics.cpp
fi

rm -f nx_data*
rm -f *temp
