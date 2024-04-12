echo "Configuring and building Thirdparty/eigen3.4.0 & 3.1.0 ..."
mkdir -p Thirdparty/eigen
cd Thirdparty/eigen
wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
wget https://gitlab.com/libeigen/eigen/-/archive/3.1.0/eigen-3.1.0.tar.gz
tar -xvf eigen-3.4.0.tar.gz 
tar -xvf eigen-3.1.0.tar.gz
mkdir build_3.4.0 build_3.1.0
cd build_3.4.0
cmake ../eigen-3.4.0 -DCMAKE_INSTALL_PREFIX=../install_3.4.0
make install
cd ..
cd build_3.1.0
cmake ../eigen-3.1.0 -DCMAKE_INSTALL_PREFIX=../install_3.1.0
make install