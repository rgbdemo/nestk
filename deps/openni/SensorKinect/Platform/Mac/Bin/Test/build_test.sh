#!/bin/sh
base=${PWD}
root_dir=${base}/../../../../
install_dir="./../../../../../OpenNI_TEST"
cmake -DCMAKE_INSTALL_PREFIX=$install_dir -DCMAKE_BUILD_TYPE:STRING=Release -DCREATE_TEST_BUILD:BOOL=ON ${root_dir}

