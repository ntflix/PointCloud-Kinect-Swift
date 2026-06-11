// swift-tools-version: 6.3
import PackageDescription

let package = Package(
    name: "PointCloud",
    products: [
        .executable(name: "PointCloud", targets: ["PointCloud"])
    ],
    targets: [
        .systemLibrary(
            name: "Clibfreenect2",
            pkgConfig: "libfreenect2",
            providers: [
                .brew(["libusb", "glfw", "jpeg-turbo"]),
                .apt(["libusb-1.0-0-dev", "libglfw3-dev", "libturbojpeg0-dev"]),
            ]
        ),

        .target(
            name: "CKinect2Bridge",
            dependencies: ["Clibfreenect2"],
            publicHeadersPath: "include",
            cxxSettings: [
                .define("LIBFREENECT2_THREADING_TINYTHREAD")
            ]
        ),

        .executableTarget(
            name: "PointCloud",
            dependencies: ["CKinect2Bridge"]
        ),

        .testTarget(
            name: "PointCloudTests",
            dependencies: ["PointCloud"]
        ),
    ],
    swiftLanguageModes: [.v6],
    cxxLanguageStandard: .cxx17,
)
