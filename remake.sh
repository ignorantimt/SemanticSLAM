# cd Pangolin/
# sudo rm -r build
# mkdir build
# cd build
# cmake ..
# make -j4
# sudo make install

# cd ../../

# cd opencv/
# sudo rm -r build
# mkdir build
# cd build
# cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local ..
# sudo make install

# cd ../..

# cd eigen/
# sudo rm -r build
# mkdir build
# cd build
# cmake ..
# sudo make install

# cd ../..

sudo rm -r build install log
colcon build --packages-ignore OpenCV Eigen Pangolin PCL sg-slam