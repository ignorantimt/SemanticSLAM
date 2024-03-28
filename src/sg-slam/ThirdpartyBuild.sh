echo "Configuring and building Thirdparty/DBoW2 ..."
cd Thirdparty/DBoW2
rm -r build
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."
rm -r build
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4

cd ../../

echo "building Thirdparty/ncnn ..."

# git clone https://github.com/Tencent/ncnn
cd ncnn
# git submodule update --init
sudo rm -r build
mkdir build
cd build
cmake -DCMAKE_TOOLCHAIN_FILE=../toolchains/host.gcc.toolchain.cmake -DNCNN_DISABLE_RTTI=OFF ..
make -j4
sudo make install

echo "Please continue to manually configure your ncnn ..."
