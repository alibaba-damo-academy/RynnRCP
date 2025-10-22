#!/bin/bash

set -e

compile_cpu=$(( $(sysctl -n hw.ncpu)/2 ))
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

check_homebrew()
{
    if ! command -v brew &> /dev/null; then
        echo "Homebrew is not installed. Installing Homebrew..."
        /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
        
        if [[ $(uname -m) == "arm64" ]]; then
            echo 'eval "$(/opt/homebrew/bin/brew shellenv)"' >> ~/.zshrc
            eval "$(/opt/homebrew/bin/brew shellenv)"
        fi
    else
        echo "Homebrew is already installed"
    fi
}

install_cmake()
{
    echo "Installing CMake..."
    if ! brew list cmake &> /dev/null; then
        brew install cmake
    else
        echo "CMake is already installed"
    fi
}

install_xcode_tools()
{
    echo "Checking Xcode command line tools..."
    if ! xcode-select -p &> /dev/null; then
        echo "Installing Xcode command line tools..."
        xcode-select --install
        echo "Please complete the Xcode command line tools installation and run this script again."
        exit 1
    else
        echo "Xcode command line tools are already installed"
    fi
}

install_glib()
{
    echo "Installing glib..."
    if ! brew list glib &> /dev/null; then
        brew install glib
    else
        echo "glib is already installed"
    fi
}

install_glog()
{
    echo "Installing glog..."
    if ! brew list glog &> /dev/null; then
        brew install glog
    else
        echo "glog is already installed"
    fi
}

install_yaml_cpp()
{
    echo "Checking yaml-cpp installation..."
    
    if brew list yaml-cpp &> /dev/null; then
        yaml_version=$(brew list --versions yaml-cpp | grep -o '[0-9]\+\.[0-9]\+\.[0-9]\+' | head -1)
        echo "yaml-cpp is already installed (version: $yaml_version)"
    else
        echo "Installing yaml-cpp..."
        brew install yaml-cpp
        yaml_version=$(brew list --versions yaml-cpp | grep -o '[0-9]\+\.[0-9]\+\.[0-9]\+' | head -1)
        echo "Installed yaml-cpp version: $yaml_version"
    fi
}

install_libwebsockets()
{
    echo "Checking libwebsockets installation..."
    
    if brew list libwebsockets &> /dev/null; then
        ws_version=$(brew list --versions libwebsockets | grep -o '[0-9]\+\.[0-9]\+\.[0-9]\+' | head -1)
        echo "libwebsockets is already installed (version: $ws_version)"
        
        # Fix linking issues if they exist
        if ! brew --prefix libwebsockets &> /dev/null; then
            echo "Fixing libwebsockets linking..."
            brew link --overwrite libwebsockets
        fi
    else
        echo "Installing libwebsockets..."
        brew install libwebsockets
        ws_version=$(brew list --versions libwebsockets | grep -o '[0-9]\+\.[0-9]\+\.[0-9]\+' | head -1)
        echo "Installed libwebsockets version: $ws_version"
    fi
}

install_openssl()
{
    echo "Checking OpenSSL installation..."
    
    if brew list openssl &> /dev/null; then
        ssl_version=$(brew list --versions openssl | grep -o '[0-9]\+\.[0-9]\+\.[0-9]\+' | head -1)
        echo "OpenSSL is already installed (version: $ssl_version)"
        
        # Fix linking issues if they exist
        if ! brew --prefix openssl &> /dev/null; then
            echo "Fixing OpenSSL linking..."
            brew link --overwrite openssl
        fi
    else
        echo "Installing OpenSSL..."
        brew install openssl
        ssl_version=$(brew list --versions openssl | grep -o '[0-9]\+\.[0-9]\+\.[0-9]\+' | head -1)
        echo "Installed OpenSSL version: $ssl_version"
    fi
}

install_protobuf()
{
    echo "Installing protobuf via Homebrew..."
    
    # Check for version mismatch and fix it
    protoc_version=""
    if command -v protoc &> /dev/null; then
        protoc_version=$(protoc --version | awk '{print $2}')
        echo "Current protoc version: $protoc_version"
    fi
    
    # Check what protobuf packages are installed
    if brew list protobuf@21 &> /dev/null && brew list protobuf &> /dev/null; then
        echo "Both protobuf and protobuf@21 are installed, this can cause version conflicts"
        echo "Removing protobuf@21 to use the latest protobuf version consistently..."
        brew uninstall protobuf@21
        # Force relink protobuf to make sure the latest version is active
        brew unlink protobuf && brew link protobuf
    fi
    
    if brew list protobuf &> /dev/null; then
        # Get the full version string, handle both old and new version formats
        protobuf_version=$(brew list --versions protobuf | head -1 | awk '{print $2}')
        echo "protobuf is already installed (version: $protobuf_version)"
        
        # Force relink to ensure consistency
        echo "Ensuring protobuf is properly linked..."
        brew unlink protobuf 2>/dev/null || true
        brew link protobuf
    else
        echo "Installing protobuf..."
        brew install protobuf
        protobuf_version=$(brew list --versions protobuf | head -1 | awk '{print $2}')
        echo "Installed protobuf version: $protobuf_version"
    fi
    
    # Verify installation
    if ! command -v protoc &> /dev/null; then
        echo "Error: protoc not found in PATH after installation"
        echo "Trying to fix PATH..."
        export PATH="/opt/homebrew/bin:$PATH"
        if ! command -v protoc &> /dev/null; then
            echo "Error: protoc still not found"
            exit 1
        fi
    fi
    
    # Verify pkg-config can find protobuf
    if ! pkg-config --exists protobuf; then
        echo "Warning: pkg-config cannot find protobuf, trying to fix linking..."
        brew link --overwrite protobuf 2>/dev/null || true
        
        if ! pkg-config --exists protobuf; then
            echo "Error: protobuf development libraries not properly installed"
            echo "Attempting to fix by reinstalling protobuf..."
            brew uninstall --ignore-dependencies protobuf
            brew install protobuf
        fi
    fi
    
    # Verify versions match
    new_protoc_version=$(protoc --version | awk '{print $2}')
    pkg_version=$(pkg-config --modversion protobuf 2>/dev/null || echo 'not found')
    
    echo "Final protoc version: $new_protoc_version"
    echo "protobuf pkg-config version: $pkg_version"
    
    # Check if versions are compatible (major.minor should match)
    protoc_major_minor=$(echo $new_protoc_version | cut -d. -f1-2)
    pkg_major_minor=$(echo $pkg_version | cut -d. -f1-2)
    
    if [ "$protoc_major_minor" != "$pkg_major_minor" ] && [ "$pkg_version" != "not found" ]; then
        echo "Warning: protoc ($new_protoc_version) and protobuf libraries ($pkg_version) versions don't match"
        echo "This may cause compilation issues with generated protobuf files"
    fi
}

install_additional_deps()
{
    echo "Installing additional dependencies..."
    
    # Install wget if not available
    if ! command -v wget &> /dev/null; then
        echo "Installing wget..."
        brew install wget
    fi
    
    # Install pkg-config
    if ! brew list pkg-config &> /dev/null; then
        brew install pkg-config
    fi
    
    # Install LCM via Homebrew to avoid build path issues
    if command -v lcm-gen &> /dev/null && brew list lcm &> /dev/null; then
        echo "LCM is already installed and working"
    elif brew list lcm &> /dev/null; then
        echo "LCM package exists but may have linking issues, skipping reinstall"
    else
        echo "Installing LCM via Homebrew..."
        brew install lcm
    fi
}

download_third_party_source_code()
{
    # Skip LCM download since we're using Homebrew version
    echo "Using LCM from Homebrew instead of building from source"

    # Download Paho MQTT (still need to build from source)
    if [ ! -d "${PROJECT_ROOT_DIR}/common/third_party/libpaho.mqtt.c-1.3.14" ]; then
        echo "Downloading libpaho.mqtt.c source code..."
        cd ${PROJECT_ROOT_DIR}/common/third_party/
        wget https://github.com/eclipse-paho/paho.mqtt.c/archive/refs/tags/v1.3.14.tar.gz
        tar -zxvf v1.3.14.tar.gz
        rm -rf v1.3.14.tar.gz
        mv paho.mqtt.c-1.3.14 libpaho.mqtt.c-1.3.14        
    fi
    
    # Fix CMake version compatibility for all third-party libraries
    echo "Fixing CMake version compatibility for third-party libraries..."
    find ${PROJECT_ROOT_DIR}/common/third_party/ -name "CMakeLists.txt" -exec sed -i '' 's/cmake_minimum_required([^)]*VERSION [0-9.]*[^)]*)/cmake_minimum_required(VERSION 3.5)/' {} \;
}

compile_third_party_libraries()
{
    echo "Compiling third-party libraries..."
    
    # Set up environment variables for macOS with Homebrew
    export PKG_CONFIG_PATH="/opt/homebrew/lib/pkgconfig:$PKG_CONFIG_PATH"
    export LDFLAGS="-L/opt/homebrew/lib $LDFLAGS"
    export CPPFLAGS="-I/opt/homebrew/include $CPPFLAGS"
    
    echo "Environment setup for third-party compilation:"
    echo "PKG_CONFIG_PATH: $PKG_CONFIG_PATH"
    echo "LDFLAGS: $LDFLAGS"
    echo "CPPFLAGS: $CPPFLAGS"
    
    # Only compile Paho MQTT (LCM is from Homebrew)
    echo "Using LCM from Homebrew, only compiling Paho MQTT..."
    
    # Create build directory for Paho MQTT only
    mkdir -p ${PROJECT_ROOT_DIR}/build/third_party/libpaho.mqtt.c-1.3.14/build/
    
    # Compile Paho MQTT
    echo "Compiling paho.mqtt.c..."
    cd ${PROJECT_ROOT_DIR}/build/third_party/libpaho.mqtt.c-1.3.14/build
    cmake ${PROJECT_ROOT_DIR}/common/third_party/libpaho.mqtt.c-1.3.14/ \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_PREFIX_PATH="/opt/homebrew"
    
    if [ $? -ne 0 ]; then
        echo "Paho MQTT CMake configuration failed!"
        exit 1
    fi
    
    make -j${compile_cpu}
    
    if [ $? -ne 0 ]; then
        echo "Paho MQTT compilation failed!"
        exit 1
    fi
    
    echo "Third-party libraries compiled successfully!"
}

compile_robot_server()
{
    echo "Compiling robot server..."
    cd ${PROJECT_ROOT_DIR}
    
    # Set up environment for compilation
    export PKG_CONFIG_PATH="/opt/homebrew/lib/pkgconfig:$PKG_CONFIG_PATH"
    export LDFLAGS="-L/opt/homebrew/lib"
    export CPPFLAGS="-I/opt/homebrew/include"
    
    # Verify protobuf is available for CMake
    echo "Verifying protobuf availability for CMake..."
    if ! pkg-config --exists protobuf; then
        echo "Error: protobuf not found by pkg-config"
        echo "PKG_CONFIG_PATH: $PKG_CONFIG_PATH"
        echo "Available .pc files in /opt/homebrew/lib/pkgconfig:"
        ls /opt/homebrew/lib/pkgconfig/proto* 2>/dev/null || echo "No protobuf .pc files found"
        exit 1
    fi
    
    echo "protobuf found: $(pkg-config --modversion protobuf)"
    echo "protobuf cflags: $(pkg-config --cflags protobuf)"
    echo "protobuf libs: $(pkg-config --libs protobuf)"
    
    # Clean previous build
    if [ -d "build" ]; then
        rm -rf build
    fi
    
    # Get the actual protobuf prefix to ensure we're using the right version
    PROTOBUF_PREFIX=$(brew --prefix protobuf)
    echo "Using protobuf from: $PROTOBUF_PREFIX"
    
    # Configure CMake with explicit protobuf paths
    cmake -DCMAKE_C_FLAGS="-Wno-deprecated-declarations" \
          -DCMAKE_CXX_FLAGS="-Wno-deprecated-declarations" \
          -DCMAKE_BUILD_TYPE=Release \
          -DCMAKE_PREFIX_PATH="/opt/homebrew;$PROTOBUF_PREFIX" \
          -DProtobuf_ROOT="$PROTOBUF_PREFIX" \
          -DPROTOBUF_ROOT="$PROTOBUF_PREFIX" \
          -B build
    
    if [ $? -ne 0 ]; then
        echo "CMake configuration failed. Checking for protobuf..."
        echo "Homebrew protobuf location: $(brew --prefix protobuf 2>/dev/null || echo 'not found')"
        exit 1
    fi
    
    cmake --build build --config Release -j ${compile_cpu}
    
    if [ $? -eq 0 ]; then
        echo "Robot server compilation completed successfully!"
    else
        echo "Robot server compilation failed!"
        exit 1
    fi
}

config_lcm_mac()
{
    echo "Configuring LCM for macOS..."
    echo "Adjusting UDP buffer settings for macOS..."
    
    # macOS uses different sysctl parameters
    echo "Checking current UDP buffer sizes..."
    current_max=$(sysctl -n net.inet.udp.maxdgram 2>/dev/null || echo "65536")
    current_recv=$(sysctl -n net.inet.udp.recvspace 2>/dev/null || echo "42080")
    
    echo "Current UDP max datagram: $current_max"
    echo "Current UDP receive space: $current_recv"
    
    if [ "$current_max" -lt 65536 ]; then
        echo "Consider increasing net.inet.udp.maxdgram if you experience UDP issues"
        echo "Run: sudo sysctl -w net.inet.udp.maxdgram=65536"
    fi
    
    if [ "$current_recv" -lt 65536 ]; then
        echo "Consider increasing net.inet.udp.recvspace if you experience UDP issues"
        echo "Run: sudo sysctl -w net.inet.udp.recvspace=65536"
    fi
}

main()
{
    echo "=================================="
    echo "OpenRCP On macOS Deployment"
    echo "=================================="
    echo "yaml-cpp: 0.8.0 (Homebrew)"
    echo "libwebsockets: 4.3.5 (Homebrew)"
    echo "protobuf: Latest (Homebrew)"
    echo "openssl: 3.5.1 (Homebrew)"
    echo "=================================="
    
    check_root_dir
    check_homebrew
    install_cmake
    install_xcode_tools
    install_glib
    install_glog
    install_yaml_cpp
    install_libwebsockets
    install_openssl
    install_additional_deps
    install_protobuf
    download_third_party_source_code
    compile_third_party_libraries
    compile_robot_server
    config_lcm_mac
    
    echo "=========================="
    echo "macOS deployment completed successfully!"
    echo "=========================="
    echo ""
    echo "Dependency versions installed:"
    echo "- CMake: $(cmake --version | head -1)"
    echo "- protoc: $(protoc --version)"
    echo "- protobuf pkg-config: $(pkg-config --modversion protobuf 2>/dev/null || echo 'not available')"
    echo "- OpenSSL: $(brew list --versions openssl)"
    echo "- yaml-cpp: $(brew list --versions yaml-cpp)"
    echo "- libwebsockets: $(brew list --versions libwebsockets)"
    echo ""
    echo "Next steps:"
    echo "1. Activate your Python virtual environment"
    echo "2. Install Python requirements: pip install -r requirements.txt"
    echo "3. Install the lerobot package: cd robot_motion/robots/lerobot && pip install -e ."
    echo "4. Configure your robot: python example/configure_so100.py"
    echo "5. Run the system: bash example/so100.sh"
    echo "6. If you encounter 'LCM self test failed', configure local loopback multicast:"
    echo "- sudo route -nv delete 224.0.0/4"
    echo "- sudo route -nv add -net 224.0.0.0/4 -interface lo0"

    cd $CURRENT_DIR
}

main "$@"