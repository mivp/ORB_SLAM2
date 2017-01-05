echo "head1 specific opencv export CMAKE_MODULE_PATH=/cave/dev/openCV/opencv-2.4.13/build/"
echo "Configuring and building Thirdparty/DBoW2 ..."
export CMAKE_MODULE_PATH=/cave/dev/openCV/opencv-2.4.13/build/

cd Thirdparty/DBoW2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

cd ../../../

echo "Uncompress vocabulary ..."

cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..

echo "Configuring and building ORB_SLAM2 ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j
