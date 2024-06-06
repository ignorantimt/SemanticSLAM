wget https://github.com/opencv/opencv/archive/3.4.10.zip
unzip 3.4.10.zip -d opencv
mkdir -p opencv/build && cd opencv/build
cmake ../opencv-3.4.10 -DCMAKE_BUILD_TYPE=Release
make -j20
sudo make install

cd ../..

wget https://github.com/stevenlovegrove/Pangolin/archive/refs/tags/v0.6.zip
unzip v0.6.zip -d Pangolin
mkdir -p Pangolin/build && cd Pangolin/build
cmake ../Pangolin-0.6 -DCMAKE_BUILD_TYPE=Release
make -j14
sudo make install

cd ../..

