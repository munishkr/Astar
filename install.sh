cd ompl/build/Release
rm -rf *
cmake ../.. -DCMAKE_INSTALL_PREFIX=/usr/local/ompl
make
sudo make install

