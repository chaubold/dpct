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

PY_VER=$(python -c "import sys; print('{}.{}'.format(*sys.version_info[:2]))")
PY_ABIFLAGS=$(python -c "import sys; print('' if sys.version_info.major == 2 else sys.abiflags)")
PY_ABI=${PY_VER}${PY_ABIFLAGS}

cmake .. \
    -DCMAKE_C_COMPILER=${CC} \
    -DCMAKE_CXX_COMPILER=${CXX} \
    -DCMAKE_CXX_FLAGS="${CXXFLAGS}" \
    -DCMAKE_OSX_DEPLOYMENT_TARGET=10.9 \
    -DCMAKE_INSTALL_PREFIX=${PREFIX} \
    -DCMAKE_BUILD_TYPE=Release \
    -DPYTHON_EXECUTABLE=${PYTHON} \
    -DPYTHON_LIBRARY=${PREFIX}/lib/libpython${PY_ABI}.${DYLIB} \
    -DPYTHON_INCLUDE_DIR=${PREFIX}/include/python${PY_ABI} \
    -DPYTHON_INCLUDE_DIR2=${PREFIX}/include/python${PY_ABI} \
    -DWITH_LOG=OFF

make -j${CPU_COUNT}
make install
