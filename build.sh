echo "building Sophus..."
cd ThirdParty/Sophus
mkdir build
cd build
cmake ..
make -j4

echo "building calibration tool..."
cd ../../../
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
