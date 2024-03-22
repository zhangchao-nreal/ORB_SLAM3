echo "Configuring and building Thirdparty/DBoW2 ..."

git submodule update --init proto

source_path=`pwd`
cmake_prefix_path="/home/oem/WkDir/slam-image-processing-for-develop/build/"
cmake_cxx_flags="-fPIC"

cd Thirdparty/DBoW2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
  -DOpenCV_DIR=/home/oem/Libs/opencv4/lib/cmake/opencv4
make -j4

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
  -DEigen3_DIR=$cmake_prefix_path/share/eigen3/cmake
make -j4

cd ../../Sophus

echo "Configuring and building Thirdparty/Sophus ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
  -DEigen3_DIR=$cmake_prefix_path/share/eigen3/cmake

make -j4

cd ../../../

echo "Uncompress vocabulary ..."

cd Vocabulary
if [ ! -e "ORBvoc.txt" ]; then
  tar -xf ORBvoc.txt.tar.gz
fi
cd ..

echo "Configuring and building ORB_SLAM3 ..."

cd $source_path
if [ ! -d "build" ]; then
  mkdir build
fi
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_STANDARD=17 \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  -DOpenCV_DIR=/home/oem/Libs/opencv-3.4.3/share/OpenCV \
  -DPangolin_DIR=/home/oem/Libs/Pangolin/lib/cmake/Pangolin0.6 \
  -DEigen3_DIR=$cmake_prefix_path/share/eigen3/cmake \
  -Dabsl_DIR=$cmake_prefix_path/lib/cmake/absl \
  -Dutf8_range_DIR=$cmake_prefix_path/lib/cmake/utf8_range \
  -DProtobuf_DIR=$cmake_prefix_path/lib/cmake/protobuf
  # -DCMAKE_PREFIX_PATH=$cmake_prefix_path \

make -j4
