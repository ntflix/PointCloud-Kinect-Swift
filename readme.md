```
brew install cmake pkg-config libusb glfw jpeg-turbo

git clone https://github.com/OpenKinect/libfreenect2.git
cd libfreenect2
mkdir build && cd build

CC=/usr/bin/clang \
CXX=/usr/bin/clang++ \
cmake .. \
  -DCMAKE_INSTALL_PREFIX=$HOME/.local/libfreenect2 \
  -DBUILD_EXAMPLES=OFF \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  ..

cmake --build .
cmake --install .
```

```
export PKG_CONFIG_PATH="$HOME/.local/libfreenect2/lib/pkgconfig:$PKG_CONFIG_PATH"
swift build
```
