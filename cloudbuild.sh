declare -a ROS_VERSIONS=( "foxy" "galactic" "humble" "rolling" )

ORGANIZATION="huawei"

for VERSION in "${ROS_VERSIONS[@]}"
do
  ROS_VERSION="$VERSION"
  gcloud builds submit --config cloudbuild.yaml . --substitutions=_ROS_VERSION="$ROS_VERSION" --timeout=10000 &
  pids+=($!)
  echo Dispatched on ROS "$ROS_VERSION"
done

for pid in ${pids[*]}; do
  wait "$pid"
done

echo "All builds finished"
