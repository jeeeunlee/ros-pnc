#! /bin/bash
PATH_PACKAGE="$(pwd)"
PATH_PNC_DATA=${PATH_PACKAGE}/"experiment_data/RawData"
PATH_GNN_FOLDER="/home/jelee/a_python_ws/graph-nets-physics/magneto-tf2-legnode/dataset/magneto/rawData_icra2/inclined_1"

## directory count
count=0
cd ${PATH_GNN_FOLDER}
for dir in *; do
  test -d "$dir" || continue
  test . = "$dir" && continue
  test .. = "$dir" && continue
  ((count++))
done
echo $count
DIR_NAME=$(printf "rdata%d" $count)
mkdir ${DIR_NAME} 

PATH_GNN_DATA=${PATH_GNN_FOLDER}/${DIR_NAME}


##
declare -a StringArray=( "q_sen.txt" "qdot_sen.txt" "q_des.txt" "qdot_des.txt" "trq.txt"
"contact_onoff_AL_foot_link.txt" "contact_onoff_BL_foot_link.txt" "contact_onoff_AR_foot_link.txt" "contact_onoff_BR_foot_link.txt"
"magnetic_AL_foot_link.txt" "magnetic_AR_foot_link.txt" "magnetic_BL_foot_link.txt" "magnetic_BR_foot_link.txt"
"magnetic_onoff_AL_foot_link.txt" "magnetic_onoff_AR_foot_link.txt" "magnetic_onoff_BL_foot_link.txt" "magnetic_onoff_BR_foot_link.txt"
"pos_al.txt" "pos_ar.txt" "pos_bl.txt" "pos_br.txt" "pose_base.txt" )

cd ${PATH_PNC_DATA}

for filename in ${StringArray[@]}
do
  cp $filename ${PATH_GNN_DATA}
done



