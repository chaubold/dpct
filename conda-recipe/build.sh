mkdir build
cd build

if [ $(uname) == "Darwin" ]; then
    CC=clang
    CXX=clang++
    CXXFLAGS="-std=c++11 -stdlib=libc++"
    
    export DYLIB="dylib"
    LINKER_FLAGS="-L${PREFIX}/lib"
else
    CC=${PREFIX}/bin/gcc
    CXX=${PREFIX}/bin/g++
    export DYLIB="so"
    LINKER_FLAGS="-Wl,-rpath-link,${PREFIX}/lib -L${PREFIX}/lib"
fi


cmake .. \
    -DCMAKE_C_COMPILER=${CC} \
    -DCMAKE_CXX_COMPILER=${CXX} \
    -DCMAKE_CXX_FLAGS="${CXXFLAGS}" \
    -DCMAKE_OSX_DEPLOYMENT_TARGET=10.9 \
    -DCMAKE_INSTALL_PREFIX=${PREFIX} \
    -DCMAKE_BUILD_TYPE=Release \
    -DPYTHON_EXECUTABLE=${PYTHON} \
    -DPYTHON_LIBRARY=${PREFIX}/lib/libpython2.7.${DYLIB} \
    -DPYTHON_INCLUDE_DIR=${PREFIX}/include/python2.7 \
    -DPYTHON_INCLUDE_DIR2=${PREFIX}/include/python2.7 \
    -DWITH_LOG=OFF

make -j${CPU_COUNT}
make install
