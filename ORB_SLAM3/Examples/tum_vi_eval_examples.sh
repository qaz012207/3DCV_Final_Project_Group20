#!/bin/bash
pathDatasetTUM_VI='/home/jeff/3DCV/Final_Project/TUM' #Example, it is necesary to change it by the dataset path
seq='dataset-corridor1_512_16'
txt='dataset-corridor1_512.txt'

# Single Session Example

echo "Launching Magistrale 1 with Stereo-Inertial sensor"
./Stereo-Inertial/stereo_inertial_tum_vi ../Vocabulary/ORBvoc.txt ./Stereo-Inertial/TUM_512.yaml "$pathDatasetTUM_VI"/"$seq"/mav0/cam0/data "$pathDatasetTUM_VI"/"$seq"/mav0/cam1/data ./Stereo-Inertial/TUM_TimeStamps/"$txt" ./Stereo-Inertial/TUM_IMU/"$txt" dataset-corridor1_512_stereoi
echo "------------------------------------"
echo "Evaluation of Magistrale 1 trajectory with Stereo-Inertial sensor"
python2 ../evaluation/evaluate_ate_scale.py "$pathDatasetTUM_VI"/"$seq"/mav0/mocap0/data.csv f_dataset-corridor1_512_stereoi.txt --plot corridor1_512_stereoi.pdf
