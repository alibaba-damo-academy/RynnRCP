#!/bin/bash

# Script for checking, installing, and configuring various dependencies required
# for a robotics project, including libraries, tools.

set -e

compile_cpu=$(( $(nproc)/2 ))
if [ $compile_cpu -lt 1 ]; then
    compile_cpu=1
fi

check_root_dir()
{
    CURRENT_DIR="$(pwd)"
    SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
    PROJECT_ROOT_DIR="$(dirname "$SCRIPT_DIR")"
    cd ${SCRIPT_DIR}
}

check_isInstall_simple()
{
    value_exit=0
    if dpkg -l | grep $1 &> /dev/null; then
        echo "Detected that device has installed $1"
        value_exit=0
    else
        echo "Detected that device has not installed $1"
        value_exit=1
    fi
}

install_cmake()
{
    echo "Checking if the device has installed cmake..."
    sleep 0.2
    check_isInstall_simple cmake
    
    if [ $value_exit -eq 1 ]; then
        sudo apt-get install cmake -y
    fi
}

install_dependence_essential()
{
    echo "Checking if the device has installed build-essential..."
    sleep 0.2
    check_isInstall_simple build-essential
    
    if [ $value_exit -eq 1 ]; then
        sudo apt-get install build-essential -y
    fi
}

install_dependence_python()
{
    echo "Checking if the device has installed python-all-dev..."
    sleep 0.2
    check_isInstall_simple python-all-dev
    
    if [ $value_exit -eq 1 ]; then
        sudo apt-get install python-all-dev -y
    fi
}

install_glib()
{
    echo "Checking if the device has installed libglib2.0-dev..."
    sleep 0.2
    check_isInstall_simple libglib2.0-dev
    
    if [ $value_exit -eq 1 ]; then
        sudo apt-get install libglib2.0-dev -y
    fi

    echo "libglib2.0's config process has ended..."
    sleep 0.2
}

install_google_glog()
{
    echo "Checking if the device has installed libgoogle-glog..."
    sleep 0.2
    check_isInstall_simple libgoogle-glog-dev
    
    if [ $value_exit -eq 1 ]; then
        sudo apt-get install libgoogle-glog-dev -y
    fi

    echo "libgoogle_glog's config process has ended..."
    sleep 0.2
}

install_protobuf()
{
   echo "Checking if protoc is less than 3.0.0 or it is not installed..."
   sleep 0.2
   set +e
   vers=$(protoc --version)
   set -e
   echo "protoc Version:"
   echo $vers
   IFS=' ' read -r -a array <<< "$vers"
   if [[ "${array[1]}" >  "3.0.0" ]]; then
       echo "protoc is >= V3.0.0, protoc does not need to be installed..."
   else
       echo "protoc is less than 3.0.0 or it is not installed ..."
       echo "installing the latest libprotoc..."
       sudo apt install protobuf-compiler libprotobuf-dev  
   fi

   echo "protoc's config process has ended..."
   sleep 0.2
}

install_Openssl3()
{
    if [ ! -f "${PROJECT_ROOT_DIR}/build/third_party/openssl-3.2.0/libcrypto.so.3" ]; then
        echo "Recompiling OpenSSL 3.2.0..."
        mkdir -p ${PROJECT_ROOT_DIR}/build/third_party/
        cp ${PROJECT_ROOT_DIR}/common/third_party/openssl-3.2.0/ ${PROJECT_ROOT_DIR}/build/third_party/openssl-3.2.0/ -r
        cd ${PROJECT_ROOT_DIR}/build/third_party/openssl-3.2.0/
        chmod +x ./config
        chmod +x ./Con*
        
        ./config
        make -j${compile_cpu}
        cd ${PROJECT_ROOT_DIR}/scripts
    else
        echo "OpenSSL has already been compiled in the expected location."
    fi
}

download_third_party_source_code()
{
    if [ ! -d ${PROJECT_ROOT_DIR}/common/third_party/lcm-1.5.0 ]; then
        echo "Downloading LCM source code..."
        cd ${PROJECT_ROOT_DIR}/common/third_party/
        wget https://github.com/lcm-proj/lcm/archive/refs/tags/v1.5.0.tar.gz
        tar -zxvf v1.5.0.tar.gz
        rm -rf v1.5.0.tar.gz

        cd lcm-1.5.0
        sed -i '/add_subdirectory(lcmgen)/d' CMakeLists.txt
        sed -i '/add_subdirectory(lcm-logger)/d' CMakeLists.txt

    fi

    if [ ! -d "${PROJECT_ROOT_DIR}/common/third_party/libpaho.mqtt.c-1.3.14" ]; then
        echo "Downloading libpaho.mqtt.c source code..."
        cd ${PROJECT_ROOT_DIR}/common/third_party/
        wget https://github.com/eclipse-paho/paho.mqtt.c/archive/refs/tags/v1.3.14.tar.gz
        tar -zxvf v1.3.14.tar.gz
        rm -rf v1.3.14.tar.gz
        mv paho.mqtt.c-1.3.14 libpaho.mqtt.c-1.3.14        
    fi
}

install_all_dependence_library()
{
    install_cmake  
    install_dependence_essential  
    install_dependence_python  
    install_glib  
    install_google_glog  
    install_protobuf
}

liblcm_compile()
{
    echo "Compiling LCM..."
    sleep 0.2
    cd ${PROJECT_ROOT_DIR}/build/third_party/lcm-1.5.0/build

    cmake \
        ${PROJECT_ROOT_DIR}/common/third_party/lcm-1.5.0/ \
        -DLCM_ENABLE_EXAMPLES=OFF \
        -DLCM_ENABLE_PYTHON=OFF \
        -DLCM_ENABLE_JAVA=OFF \
        -DLCM_ENABLE_LUA=OFF \
        -DLCM_ENABLE_TESTS=OFF

    make -j$compile_cpu
    
    cd ${PROJECT_ROOT_DIR}
    sleep 0.2
}

libwebsockets_compile()
{
    echo "Compiling libwebsockets..."
    sleep 0.2
    cd ${PROJECT_ROOT_DIR}/build/third_party/libwebsockets-4.0.20/build

    # Use OpenSSL
    export OPENSSL_ROOT_DIR=${PROJECT_ROOT_DIR}/build/third_party/openssl-3.2.0/
    cmake \
        -DOPENSSL_ROOT_DIR=${OPENSSL_ROOT_DIR} \
        -DOPENSSL_INCLUDE_DIR=${OPENSSL_ROOT_DIR}/include \
        -DOPENSSL_CRYPTO_LIBRARY=${OPENSSL_ROOT_DIR}/libcrypto.so \
        -DOPENSSL_SSL_LIBRARY=${OPENSSL_ROOT_DIR}/libssl.so \
        -DCMAKE_C_FLAGS="-Wno-deprecated-declarations" \
        ${PROJECT_ROOT_DIR}/common/third_party/libwebsockets-4.0.20/

    make -j${compile_cpu}
    cd ${PROJECT_ROOT_DIR}
    sleep 0.2
}

libmqtt_compile()
{
    echo "Compiling paho.mqtt.c..."
    sleep 0.2
 
    cd ${PROJECT_ROOT_DIR}/build/third_party/libpaho.mqtt.c-1.3.14/build
    
    if [ -f "./Makefile" ]; then
        echo "MQTT's Makefile exists..."
    else
        cmake ${PROJECT_ROOT_DIR}/common/third_party/libpaho.mqtt.c-1.3.14/
    fi 
    
    make -j${compile_cpu}
 
    cd ${PROJECT_ROOT_DIR}
    sleep 0.2
}

libyaml_cpp_compile()
{
    echo "Compiling libyaml-cpp..."
    sleep 0.2
    
    cd ${PROJECT_ROOT_DIR}/build/third_party/libyaml-cpp-0.7.0/build
    cmake -DBUILD_SHARED_LIBS=ON -DCMAKE_POSITION_INDEPENDENT_CODE=ON ${PROJECT_ROOT_DIR}/common/third_party/libyaml-cpp-0.7.0/
    make -j${compile_cpu}
    
    cd ${PROJECT_ROOT_DIR}
    sleep 0.2
}

third_party_compile()
{
    mkdir -p ${PROJECT_ROOT_DIR}/build/third_party/lcm-1.5.0/build/
    mkdir -p ${PROJECT_ROOT_DIR}/build/third_party/libwebsockets-4.0.20/build/
    mkdir -p ${PROJECT_ROOT_DIR}/build/third_party/libpaho.mqtt.c-1.3.14/build/
    mkdir -p ${PROJECT_ROOT_DIR}/build/third_party/libyaml-cpp-0.7.0/build/  
    cd ../
    
    liblcm_compile  
    libwebsockets_compile  
    libmqtt_compile
    libyaml_cpp_compile
}

robot_server_compile()
{
    echo "Compiling server..."
    sleep 0.2
 
    export LDFLAGS="-L${PROJECT_ROOT_DIR}/build/third_party/openssl-3.2.0/"
    cmake -DCMAKE_C_FLAGS="-Wno-deprecated-declarations" -B build
    cmake --build build --config Release -j ${compile_cpu}
     
    echo "server's compile process has ended..."
    export LIBRARY_PATH=$TMP
    sleep 0.2
}

check_root_dir
install_all_dependence_library 
install_Openssl3
download_third_party_source_code
third_party_compile  
robot_server_compile

#-----------------------config LCM---------------------------------------------------
configLcm()
{
    echo "Ready to configure LCM..."
    echo "Modifying /etc/sysctl.conf... adjust the max size of LCM udp buffer for camera..."
    FILE="/etc/sysctl.conf"
    sudo chmod 666 /etc/sysctl.conf
    SEARCH_STRING="net.core.rmem_max = 16777216"
    CONTENT_TO_ADD="net.core.rmem_max = 16777216"
    if ! grep -qF "$SEARCH_STRING" "$FILE"; then
        echo "$CONTENT_TO_ADD" | sudo tee -a "$FILE"
        echo "$FILE: the max size of LCM udp buffer has been modified..."
    else
        echo "The max size of LCM udp buffer doesn't need modification..."
    fi

    echo "Modifying /etc/sysctl.conf... adjust the default size of LCM udp buffer..."
    SEARCH_STRING="net.core.rmem_default = 16777216"
    CONTENT_TO_ADD="net.core.rmem_default = 16777216"
    if ! grep -qF "$SEARCH_STRING" "$FILE"; then
        echo "$CONTENT_TO_ADD" | sudo tee -a "$FILE"
        echo "$FILE: the default size of LCM udp buffer has been modified..."
    else
        echo "The default size of LCM udp buffer doesn't need modification..."
    fi
    sudo sysctl -p
    sleep 0.2
}
configLcm

cd $CURRENT_DIR
