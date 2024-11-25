export SUBSET=0

python ./carla_vqa_generator_main.py --data-directory=../../../Bench2Drive-rep --output-graph-directory=./outgraph --path-keyframes=./keyframes --path-maps=${CARLA_ROOT}/CarlaUE4/Content/Carla/Maps --output-graph-examples-directory=./outexample --save-examples --visualize-projection