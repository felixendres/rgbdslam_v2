#!/bin/bash

DIR=`readlink -f $1`
PACKAGEDIR=`rospack find rgbdslam`/
if  test ! -d "$DIR"; then  
  echo "'$DIR' is not a directory" 
  echo "Usage: $0 <directory-where-results-are>"
  echo "e.g., $0 some/path/to/SURF/"
  exit
fi
ERROR_STATISTIC=max
pushd $DIR > /dev/null
for num in 0 1 2 3 4; do 
  if grep "absolute.*error" ${ERROR_STATISTIC}ate_evaluation_${num}.csv &> /dev/null;then
    echo "[0;36mResult exists: ate_evaluation_$num.csv[1;36m[0m "
    #column '-s;' -t  ate_evaluation_$num.csv
    #A bit fancier than the above:
    sed 's/absolute translational error.${ERROR_STATISTIC}/AT-${ERROR_STATISTIC^^}/' ${ERROR_STATISTIC}ate_evaluation_$num.csv | sed 's/"[0-9]* Features /"/g' | column '-s;' -t
    continue;
  fi
  rm -f ${ERROR_STATISTIC}ate_evaluation_$num.csv
  rm -f evaluation_$num.csv

  rm -f eval_translational.txt eval_translational.ate.txt eval_rotational.txt eval_runtime.txt 
  for BASENAME in `ls -d rgbd_dataset_freiburg* strata* 2>/dev/null`; do
    echo -n "$BASENAME ... "
    if test ! -d "$BASENAME";then
      echo "[0;31mNot ready yet[1;31m[0m " $num 
      break;
    fi
    ESTIMATE_FILE=$BASENAME/${BASENAME}.bagiteration_${num}_estimate.txt
    if test ! -f $ESTIMATE_FILE;then
      echo "[0;31mNo estimate at level[1;31m[0m " $num 
      continue;
    fi
    
    if test ! -f $BASENAME/logfile.gz ;then
      if test ! -f $BASENAME/logfile ;then
        echo "[0;31mNo logfile! [1;31m[0m" $num 
        continue;
      fi
      gzip $BASENAME/logfile
    fi
    gunzip -c $BASENAME/logfile.gz > $BASENAME/logfile.txt
    EVAL_FILE=$ESTIMATE_FILE.evaluation
    #if rosrun rgbd_benchmark_tools evaluate_rpe.py --verbose $BASENAME/${BASENAME}-groundtruth.txt $ESTIMATE_FILE > $EVAL_FILE; then
    #if  $PACKAGEDIR/rgbd_benchmark/evaluate_ate.py --plot $BASENAME/$BASENAME.difference_plot$num.png --verbose $BASENAME/${BASENAME}-groundtruth.txt $ESTIMATE_FILE > $EVAL_FILE.ate ; then
    if  $PACKAGEDIR/rgbd_benchmark/evaluate_ate.py --verbose $BASENAME/${BASENAME}-groundtruth.txt $ESTIMATE_FILE > $EVAL_FILE.ate ; then
      #rosrun rgbd_benchmark_tools align_and_plot.py --plot $BASENAME/$BASENAME.alignment_plot$num.png --verbose $BASENAME/${BASENAME}-groundtruth.txt $ESTIMATE_FILE > /dev/null

      #RMSE
      #echo -n "FR${BASENAME#rgbd_dataset_freiburg}; " >> eval_translational.txt
      echo -n "FR${BASENAME#rgbd_dataset_freiburg}; " >> eval_translational.ate.txt
      #echo -n "FR${BASENAME#rgbd_dataset_freiburg}; " >> eval_rotational.txt
      #grep translational_error.rmse $EVAL_FILE |sed "s#$DIR##g" | sed 's/ /; /g' >> eval_translational.txt
      #grep rotational_error.rmse $EVAL_FILE |sed "s#$DIR##g" | sed 's/ /; /g' >> eval_rotational.txt
      grep translational_error.${ERROR_STATISTIC} $EVAL_FILE.ate |sed "s#$DIR##g" | sed 's/ /; /g' >> eval_translational.ate.txt

      #OVERALL RUNTIME
      STARTTIME=`grep "First RGBD-Data Received" $BASENAME/logfile.txt |head -n1|grep -o '14[0-9]*\.'` #timestamp first relevant action
      STARTTIME_NSEC=`grep "First RGBD-Data Received" $BASENAME/logfile.txt |head -n1|grep -o '\.[0-9]*'` #timestamp first relevant action
      ENDTIME=`grep "Finished with optimization iteration $num[^0-9]" $BASENAME/logfile.txt |head -n1|grep -o '14[0-9][0-9]*\.'` #timestamp first relevant action
      ENDTIME_NSEC=`grep "Finished with optimization iteration $num[^0-9]" $BASENAME/logfile.txt |head -n1|grep -o '\.[0-9][0-9]*'` #timestamp first relevant action
      #echo -n "Start; ${STARTTIME%.}.${STARTTIME_NSEC#.};s; End; ${ENDTIME%.}.${ENDTIME_NSEC#.};s;" >> eval_runtime.txt
      timediff=`echo "${ENDTIME%.}.${ENDTIME_NSEC#.} - ${STARTTIME%.}.${STARTTIME_NSEC#.}" |bc` 
      echo -n "Duration; $timediff;s;" >> eval_runtime.txt
      
      if [[ "$NO_OPTIMIZER_EVAL" == "" ]]; then 
        #OPTIMIZER RUNTIME
        OPT_TIME=`grep timiz $BASENAME/logfile.txt |grep -B3 "Finished with optimization iteration $num[^0-9]"  |grep -o 'Optimizer Runtime; [0-9.]*'` #timestamp first relevant action
        if [[ "$OPT_TIME" == "" ]]; then 
          OPT_TIME='Optimizer Runtime; 0.0000'
        fi
        echo -n "${OPT_TIME};s;" >> eval_runtime.txt


        #NUMBER OF NODES
        G2O_LINE=`grep timiz $BASENAME/logfile.txt |grep -B3 "Finished with optimization iteration $num[^0-9]" |grep "Optimization with"|tail -n1`
        NODE_NUM=`echo $G2O_LINE | grep -o '[0-9]* nodes'` #timestamp first relevant action
        EDGE_NUM=`echo $G2O_LINE | grep -o '[0-9]* edges'` #timestamp first relevant action
        if [[ $NODE_NUM == "" ]]; then
          NODE_NUM=1
          EDGE_NUM=1
        fi
        echo -n "Number of Nodes/Edges; ${NODE_NUM% nodes};${EDGE_NUM% edges};" >> eval_runtime.txt
        #echo -n "Number of Edges;${EDGE_NUM#edges= };" >> eval_runtime.txt
      fi
      echo >> eval_runtime.txt

    else
      echo "Evaluation Failed"
    fi
    rm $BASENAME/logfile.txt
  done
  #paste "-d;" eval_rotational.txt eval_translational.txt eval_runtime.txt |sed "s#$DIR/##g" | sed 's/rgbd_dataset_freiburg/FR/g' |sed 's/.evaluation//g' |sed 's/.bagafter._optimization_estimate.txt//g'|sed 's/.bag//g'|sed 's/flowerbouquet/flwrbqt/g' |sed 's/background/bg/g'|sed 's#/FR[^/]*/##g'|sed 's/_/ /g' > evaluation_$num.csv
  experiment=`basename $DIR`
  echo
  echo "${experiment//_/ } ; ;\"${experiment//_/ } ATE ${ERROR_STATISTIC^^}\"; ; ;\"${experiment//_/ } Duration\"; ; ;\"${experiment//_/ } Optimization Time\"; ; ;Nodes;Edges;" > ${ERROR_STATISTIC}ate_evaluation_$num.csv
  paste "-d;" eval_translational.ate.txt eval_runtime.txt |sed "s#$DIR/##g" | sed 's/rgbd_dataset_freiburg/FR/g' |sed 's/.evaluation//g' |sed 's/.bagafter._optimization_estimate.txt//g'|sed 's/.bag//g'|sed 's/flowerbouquet/flwrbqt/g' |sed 's/background/bg/g'|sed 's#/FR[^/]*/##g'|sed 's/_/ /g' >> ${ERROR_STATISTIC}ate_evaluation_$num.csv
  echo ATE Results at Level $num are stored in $DIR/${ERROR_STATISTIC}ate_evaluation_$num.csv
  #column '-s;' -t  ${ERROR_STATISTIC}ate_evaluation_$num.csv
  #A bit fancier than the above:
  sed "s/absolute translational error.${ERROR_STATISTIC}/AT-${ERROR_STATISTIC^^}/" ${ERROR_STATISTIC}ate_evaluation_$num.csv | sed 's/"[0-9]* Features /"/g' | column '-s;' -t
  #echo RPE Results at Level $num are stored in $DIR/evaluation_$num.csv
  #column '-s;' -t  evaluation_$num.csv
done
rm -f eval_translational.txt eval_translational.ate.txt eval_rotational.txt eval_runtime.txt 

popd > /dev/null
