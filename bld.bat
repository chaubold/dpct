mkdir build
cd build

set CONFIGURATION=Release

REM set LINKER_FLAGS="-L${PREFIX}/lib"
REM set DYLIB="dylib"
REM if [ `uname` != "Darwin" ]; then
    REM LINKER_FLAGS="-Wl,-rpath-link,${PREFIX}/lib ${LINKER_FLAGS}"
    REM set DYLIB="so"
REM fi

cmake .. -G "%CMAKE_GENERATOR%" ^
         -DCMAKE_PREFIX_PATH="%LIBRARY_PREFIX%" ^
         -DCMAKE_INSTALL_PREFIX="%LIBRARY_PREFIX%" ^
         -DPYTHON_EXECUTABLE="%PYTHON%" ^
         -DPYTHON_LIBRARY="%PYTHON%" ^
         -DWITH_LOG="OFF"


if errorlevel 1 exit 1

REM cmake .. \
    REM -DCMAKE_C_COMPILER=${PREFIX}/bin/gcc \
    REM -DCMAKE_CXX_COMPILER=${PREFIX}/bin/g++ \
    REM -DCMAKE_OSX_DEPLOYMENT_TARGET=10.7 \
    REM -DCMAKE_INSTALL_PREFIX=${PREFIX} \
    REM -DCMAKE_BUILD_TYPE=Release \
    REM -DPYTHON_EXECUTABLE=${PYTHON} \
    REM -DPYTHON_LIBRARY=${PREFIX}/lib/libpython2.7.${DYLIB} \
    REM -DPYTHON_INCLUDE_DIR=${PREFIX}/include/python2.7 \
    REM -DPYTHON_INCLUDE_DIR2=${PREFIX}/include/python2.7 \
    REM -DWITH_LOG=OFF

cmake --build . --target ALL_BUILD --config %CONFIGURATION%
if errorlevel 1 exit 1

cmake --build . --target INSTALL --config %CONFIGURATION%
if errorlevel 1 exit 1
