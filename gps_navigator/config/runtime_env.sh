# Source this in every Jetson terminal, including RViz/ROS CLI terminals.
# Does not change ROS_DOMAIN_ID. Remote DDS use needs an explicit network profile.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=1
export CYCLONEDDS_URI="file://$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/cyclonedds.xml"
export OPENCV_FOR_THREADS_NUM=2
export OMP_NUM_THREADS=2
export OPENBLAS_NUM_THREADS=1
