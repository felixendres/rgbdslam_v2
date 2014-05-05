#!/bin/bash

echo Feature Type: $1 Bagfile Directory: $2 Output Directory Prefix: $3
if [[ "$1" == "" ]] || test  '!' -d "$2" ; then
  echo "Usage: $0 <feature-type> <directory-where-bagfiles-and-groundtruth-files-are>"
  echo "E.g.: $0 SURF ~/ros/rgbdslam/rgbd_benchmark/benchmark_data/"
  exit
fi
BASE_DIRECTORY=`readlink -f $2` 
pushd $BASE_DIRECTORY > /dev/null


export ROS_MASTER_URI=http://localhost:11382
roscore -p 11382&
sleep 1
for CANDIDATES in 8; do
  rosparam set /rgbdslam/config/predecessor_candidates $CANDIDATES
  rosparam set /rgbdslam/config/neighbor_candidates $CANDIDATES
  rosparam set /rgbdslam/config/min_sampled_candidates $((2,5 * CANDIDATES))
  for OBS_EVAL in  0.00 0.75; do
    rosparam set /rgbdslam/config/observability_threshold $OBS_EVAL
    for RANSAC_ITER in 300; do
      rosparam set /rgbdslam/config/ransac_iterations $RANSAC_ITER

      for DISTANCEMSR in true; do 
        rosparam set /rgbdslam/config/use_root_sift $DISTANCEMSR

        for NN_RATIO in 0.90; do
          rosparam set /rgbdslam/config/nn_distance_ratio $NN_RATIO

          for MIN_MATCHES in 020; do
            rosparam set /rgbdslam/config/min_matches $MIN_MATCHES
            for FEAT_TYPE in SIFTGPU; do 
              echo Evaluating $FEAT_TYPE
              ##Evaluate Feature types
              rosparam set /rgbdslam/config/feature_detector_type $FEAT_TYPE
              rosparam set /rgbdslam/config/feature_extractor_type $FEAT_TYPE

              echo "Will evaluate RGBD-SLAM on the following bagfiles:"
              SELECTION=`ls rgbd_dataset_freiburg2_large_no_loop.bag`
              echo $SELECTION

              for MAXFEATURES in 1200; do
                rosparam set /rgbdslam/config/max_keypoints $MAXFEATURES
                #PARAM_DIRECTORY="$BASE_DIRECTORY/$3/emm__$OBS_EVAL/CANDIDATES_$CANDIDATES/RANSAC_$RANSAC_ITER/SOLVER_$DISTANCEMSR/NN_$NN_RATIO/MIN_MATCHES_$MIN_MATCHES/${FEAT_TYPE}/${MAXFEATURES}_Features/"
                PARAM_DIRECTORY="$BASE_DIRECTORY/$3/emm__$OBS_EVAL/CANDIDATES_$CANDIDATES/RANSAC_$RANSAC_ITER/HellingerDistance_$DISTANCEMSR/NN_$NN_RATIO/MIN_MATCHES_$MIN_MATCHES/${FEAT_TYPE}/${MAXFEATURES}_Features/"
                for bagfile in $SELECTION; do
                  BASE_NAME=`basename $bagfile .bag` 
                  DIRECTORY="$PARAM_DIRECTORY/$BASE_NAME"
                  mkdir -p $DIRECTORY
                  if grep -q Coordinate $DIRECTORY/*estimate.txt 2> /dev/null; then 
                    echo There are already results for $BASE_NAME in $DIRECTORY. Will skip this bagfile
                    continue #don't overwrite existing results
                  fi
                  echo `date +%H:%M:%S` Results for $BASE_NAME are stored in `readlink -f $DIRECTORY`
                  rosparam set /rgbdslam/config/bagfile_name `readlink -f $bagfile`
                  sleep 1
                  roslaunch rgbdslam settings_for_evaluation.launch >  $DIRECTORY/logfile 2>&1
                  rosparam get /rgbdslam/config >>  $DIRECTORY/logfile 2>&1
                  echo `date +%H:%M:%S` Finished processing $BASE_NAME

                  #Move Result files, run evaluation routine
                  mv ${bagfile}?* $DIRECTORY/
                  cp ${BASE_NAME}-groundtruth.txt $DIRECTORY/
                  LAUNCHFILE=`rospack find rgbdslam`/rgbd_benchmark/settings_for_evaluation.launch
                  cp $LAUNCHFILE $DIRECTORY/settings.xml #renamed to avoid name conflict with original file in roslaunch command
                  #cp `readlink -f $0` $DIRECTORY/`basename $0`

                  pushd $DIRECTORY > /dev/null
                  test -e logfile.gz && mv logfile.gz logfile-failed.gz
                  gzip logfile
                  popd > /dev/null
                done
                rosrun rgbdslam summarize_evaluation.sh  $PARAM_DIRECTORY
              done
            done
          done
        done
      done
    done
  done
done
popd > /dev/null
