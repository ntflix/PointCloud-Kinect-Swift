Use Xbox Kinect V2 with Swift and `libfreenect2` to generate point clouds (and frames of point clouds over multiple frames/seconds).

Clone, build, and install `libfreenect2` first:

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

> Those instructions differ from the actual `libfreenect2` instructions as 1) the official instructions don't work for me on macOS and 2) want it installed as a library to `~/.local/libfreenect2`

Use its `PKG_CONFIG` doodahs to build and run with Swift:

```
export PKG_CONFIG_PATH="$HOME/.local/libfreenect2/lib/pkgconfig:$PKG_CONFIG_PATH"
swift build
```

Example use in `.vscode/launch.json`.
