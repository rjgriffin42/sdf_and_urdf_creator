#!/bin/bash

# Script to parse CAD data to create SDFs, URDFs, and KDL models
# Processing files author : Nikolaus Wittenstein
# Author : Robert Griffin <rjgriffin42@gmail.com>

printUsage()
{
  echo ""
  echo "Usage: generate_sdf.sh [-r robot_name] [-m meshes (true:-false)]"
  echo "                       [-t target_directory]"
  echo ""
  echo "Creates sdf and urdf files, as well as a KDL model."
  echo ""
  echo "Valid inputs for the robot target are : "
  echo "   escher"
  echo "   thor"
  echo "   v2exoskeleton"
  echo ""
  echo "Meshes flag creates collisions for SDFs and URDFs using either"
  echo "STL file meshes or bounding boxes."
  echo ""
  echo "The target directory defaults to model_directoy/robot_name"
  echo ""
}

checkDirectory()
{
  target_dir=$1
  echo "Writing to custom directory $target_dir ..."
  if [[ ! -e $target_dir ]]; then
    echo "Making directory $target_dir"
    mkdir -p $target_dir
  fi
}

options='r:m:t:fh?:'
while getopts $options option ; do
    case $option in
        r  ) robot_input=${OPTARG};;
        m  ) use_meshes=${OPTARG};;
        t  ) checkDirectory ${OPTARG};;
        h  ) printUsage; exit 1;;
        \? ) printUsage; exit 1;;
        *  ) printUsage; exit 1;;
    esac
done


robot_input=${robot_input:-escher}
if [ $robot_input == "escher" ]; then
  robot="ESCHER"
  robot_lower_case="escher"
  robot_dir=$robot
elif [ $robot_input == "thor" ]; then
  robot="THOR"
  robot_lower_case="thor"
  robot_dir=$robot
elif [ $robot_input == "v2exoskeleton" ]; then
  robot="V2Exoskeleton"
  robot_lower_case="v2exoskeleton"
  robot_dir=$robot
else
  echo "Valid robot build target not specified."
  echo "Input options are : "
  echo "     escher"
  echo "     thor"
  echo "     v2exoskeleton"
fi

use_meshes=${use_meshes:-false}
script_dir=$(dirname $0)
data_dir=$script_dir/robot_templates/$robot_dir
default_target_dir=$script_dir/../model_directory/$robot_dir
target_dir=${target_dir:-$default_target_dir}

# Starts the building of the seperate files
buildFiles()
{
  #for KDL
  echo -e "*with \nlidar=1" > kdl_temp
  
  #for URDF
  echo -e "*with \nlidar=0" > urdf_temp
  
  #for SDF
  echo -e "*with \nlidar=0" > sdf_temp
  
  for i in kdl_temp urdf_temp sdf_temp sdf_temp; do
    cat $data_dir/${robot_lower_case}_cad_data.txt >> $i
  
    # reduce_template creates a file named something like "nx_data84". Rename it to something better.
    mv $(./reduce_template.sh $i) $i
  done
  
  if [ $use_meshes == "true" ]; then
  
    echo "Generating $robot URDF Mesh Collision Models"
  
    # Robot URDF Mesh Model
    lua $script_dir/format_model.lua $data_dir/${robot_lower_case}_urdf_geometry_mesh.txt $data_dir/${robot_lower_case}_urdf_template.txt > mesh_temp
    lua $script_dir/format_model.lua urdf_temp mesh_temp > $target_dir/${robot_lower_case}.xacro
  
    echo "Generating $robot SDF Mesh Collision Models"
    
    # Robot SDF Mesh Model
    lua $script_dir/format_model.lua $data_dir/${robot_lower_case}_sdf_geometry_mesh.txt $data_dir/${robot_lower_case}_sdf_template.txt > mesh_temp
    lua $script_dir/format_model.lua sdf_temp mesh_temp > $target_dir/${robot_lower_case}.sdf
  else
    echo "Generating $robot URDF Bounding Box Collision Models"
    
    # Robot URDF Bounding Box Model
    lua $script_dir/format_model.lua $data_dir/${robot_lower_case}_urdf_geometry_bbox.txt $data_dir/${robot_lower_case}_urdf_template.txt > bbox_temp
    lua $script_dir/format_model.lua urdf_temp bbox_temp > $target_dir/${robot_lower_case}.xacro
    
    echo "Generating $robot SDF Bounding Box Collision Models"
    
    # Robot SDF Bounding Box Model
    lua $script_dir/format_model.lua $data_dir/${robot_lower_case}_sdf_geometry_bbox.txt $data_dir/${robot_lower_case}_sdf_template.txt > bbox_temp
    lua $script_dir/format_model.lua sdf_temp bbox_temp > $target_dir/${robot_lower_case}.sdf
  fi
  
  #KDL Mechanics
  echo "Generating $robot KDL Mechanics"
  lua $script_dir/format_model.lua kdl_temp $data_dir/${robot_lower_case}_kdl_template.txt > $target_dir/${robot_lower_case}.cpp
  
  rm -f data_container*
  rm -f *temp
}

buildFiles
