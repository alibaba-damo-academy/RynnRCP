#!/bin/bash

# OpenRCP Linux deployment script
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
    echo "Checking cmake version compatibility..."
    
    if command -v cmake >/dev/null 2>&1; then
        CMAKE_VERSION=$(cmake --version | head -n1 | cut -d" " -f3)
        CMAKE_MAJOR=$(echo $CMAKE_VERSION | cut -d. -f1)
        CMAKE_MINOR=$(echo $CMAKE_VERSION | cut -d. -f2)
        
        echo "Found CMake version: $CMAKE_VERSION"
        
        if [ $CMAKE_MAJOR -gt 3 ] || [ $CMAKE_MAJOR -eq 3 -a $CMAKE_MINOR -ge 5 ]; then
            echo "CMake version is sufficient for building LCM"
            return 0
        else
            echo "CMake version is too old, attempting to install newer version..."
        fi
    else
        echo "CMake not found, installing..."
    fi
    
    sudo apt-get update
    sudo apt-get install cmake -y
    
    if command -v cmake >/dev/null 2>&1; then
        CMAKE_VERSION=$(cmake --version | head -n1 | cut -d" " -f3)
        CMAKE_MAJOR=$(echo $CMAKE_VERSION | cut -d. -f1)
        CMAKE_MINOR=$(echo $CMAKE_VERSION | cut -d. -f2)
        
        echo "Installed CMake version: $CMAKE_VERSION"
        
        if [ $CMAKE_MAJOR -gt 3 ] || [ $CMAKE_MAJOR -eq 3 -a $CMAKE_MINOR -ge 5 ]; then
            echo "CMake version is now sufficient"
        else
            echo "Warning: CMake version might still be too old. LCM compilation will use compatibility flags."
        fi
    fi
}

install_dependence_essential()
{
    echo "Checking if the device has installed build-essential..."
    check_isInstall_simple build-essential
    
    if [ $value_exit -eq 1 ]; then
        sudo apt-get install build-essential -y
    fi
}

install_dependence_python()
{
    echo "Checking if the device has installed python-all-dev..."
    check_isInstall_simple python-all-dev
    
    if [ $value_exit -eq 1 ]; then
        sudo apt-get install python-all-dev -y
    fi
}

install_glib()
{
    echo "Checking if the device has installed libglib2.0-dev..."
    check_isInstall_simple libglib2.0-dev
    
    if [ $value_exit -eq 1 ]; then
        sudo apt-get install libglib2.0-dev -y
    fi

    echo "libglib2.0's config process has ended..."
}

install_google_glog()
{
    echo "Checking if the device has installed libgoogle-glog..."
    check_isInstall_simple libgoogle-glog-dev
    
    if [ $value_exit -eq 1 ]; then
        sudo apt-get install libgoogle-glog-dev -y
    fi

    echo "libgoogle_glog's config process has ended..."
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
       sudo apt install protobuf-compiler libprotobuf-dev -y
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
    cd ${PROJECT_ROOT_DIR}/build/third_party/lcm-1.5.0/build

    CMAKE_VERSION=$(cmake --version | head -n1 | cut -d" " -f3)
    CMAKE_MAJOR=$(echo $CMAKE_VERSION | cut -d. -f1)
    CMAKE_MINOR=$(echo $CMAKE_VERSION | cut -d. -f2)
    
    CMAKE_FLAGS=""
    
    if [ $CMAKE_MAJOR -gt 3 ] || [ $CMAKE_MAJOR -eq 3 -a $CMAKE_MINOR -ge 5 ]; then
        echo "Using CMake $CMAKE_VERSION - adding compatibility flags for LCM"
        CMAKE_FLAGS="-DCMAKE_POLICY_VERSION_MINIMUM=3.5"
    else
        echo "Using CMake $CMAKE_VERSION - no compatibility flags needed"
    fi

    cmake \
        ${PROJECT_ROOT_DIR}/common/third_party/lcm-1.5.0/ \
        $CMAKE_FLAGS \
        -DLCM_ENABLE_EXAMPLES=OFF \
        -DLCM_ENABLE_PYTHON=OFF \
        -DLCM_ENABLE_JAVA=OFF \
        -DLCM_ENABLE_LUA=OFF \
        -DLCM_ENABLE_TESTS=OFF

    make -j$compile_cpu
    
    cd ${PROJECT_ROOT_DIR}
}

libwebsockets_compile()
{
    echo "Compiling libwebsockets..."
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
}

libmqtt_compile()
{
    echo "Compiling paho.mqtt.c..."
 
    cd ${PROJECT_ROOT_DIR}/build/third_party/libpaho.mqtt.c-1.3.14/build
    
    if [ -f "./Makefile" ]; then
        echo "MQTT's Makefile exists..."
    else
        cmake ${PROJECT_ROOT_DIR}/common/third_party/libpaho.mqtt.c-1.3.14/
    fi 
    
    make -j${compile_cpu}
 
    cd ${PROJECT_ROOT_DIR}
}

libyaml_cpp_compile()
{
    echo "Compiling libyaml-cpp..."
    
    cd ${PROJECT_ROOT_DIR}/build/third_party/libyaml-cpp-0.7.0/build
    cmake -DBUILD_SHARED_LIBS=ON -DCMAKE_POSITION_INDEPENDENT_CODE=ON ${PROJECT_ROOT_DIR}/common/third_party/libyaml-cpp-0.7.0/
    make -j${compile_cpu}
    
    cd ${PROJECT_ROOT_DIR}
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
 
    export LDFLAGS="-L${PROJECT_ROOT_DIR}/build/third_party/openssl-3.2.0/"
    cmake -DCMAKE_C_FLAGS="-Wno-deprecated-declarations" -B build
    cmake --build build --config Release -j ${compile_cpu}
     
    echo "server's compile process has ended..."
    export LIBRARY_PATH=$TMP
}

main()
{
    echo "=================================="
    echo "OpenRCP Linux Deployment"
    echo "=================================="
    echo "Installing system dependencies..."
    echo "=================================="
    
    check_root_dir
    install_all_dependence_library 
    install_Openssl3
    download_third_party_source_code
    third_party_compile  
    robot_server_compile
    configLcm
    
    echo "=================================="
    echo "Linux deployment completed successfully!"
    echo "=================================="
    echo ""
    echo "Next steps:"
    echo "1. Activate your Python virtual environment"
    echo "2. Install Python requirements: pip install -r requirements.txt"
    echo "3. Install the lerobot package: cd robot_motion/robots/lerobot && pip install -e ."
    echo "4. Configure your robot: python example/configure_so100.py"
    echo "5. Run the system: bash example/so100.sh"
    
    cd $CURRENT_DIR
}

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
    sudo sysctl -p || echo "Can't reload sysctl.conf. Please manually reload (sudo sysctl -p) if not in a Docker environment."
}

# Run main function
main "$@"
