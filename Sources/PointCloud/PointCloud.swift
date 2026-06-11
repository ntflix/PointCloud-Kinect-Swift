import CKinect2Bridge
import Foundation

@main
struct PointCloud {
    static func main() {
        guard let dev = kinect2_open_default() else {
            fputs("Failed to open Kinect v2 device\n", stderr)
            exit(1)
        }
        defer { kinect2_close(dev) }

        guard kinect2_start(dev) == 0 else {
            fputs("Failed to start Kinect streams\n", stderr)
            exit(1)
        }
        defer { kinect2_stop(dev) }

        let maxPoints = Int(kinect2_depth_width() * kinect2_depth_height())
        var points = Array(
            repeating: kinect2_point_t(x: 0, y: 0, z: 0, r: 0, g: 0, b: 0, a: 0),
            count: maxPoints
        )

        var count = 0

        for frameIndex in 0..<1200 {
            guard kinect2_wait_frame(dev, 1000) == 0 else {
                print("frame \(frameIndex): timeout")
                continue
            }

            var row: Int32 = -1
            var col: Int32 = -1
            var rawDepth: Float = 0
            var x: Float = 0
            var y: Float = 0
            var z: Float = 0

            if kinect2_debug_first_valid_xyz_pixel(dev, &row, &col, &rawDepth, &x, &y, &z) == 0 {
                print(
                    "first valid XYZ pixel: row=\(row) col=\(col) rawDepth=\(rawDepth) xyz=(\(x), \(y), \(z))"
                )
            } else {
                print("no valid XYZ pixel found")
            }

            var cx: Float = 0
            var cy: Float = 0
            var cz: Float = 0
            if kinect2_debug_center_xyz(dev, &cx, &cy, &cz) == 0 {
                print("center xyz=(\(cx), \(cy), \(cz))")
            }

            let validDepth = Int(kinect2_count_valid_depth_pixels(dev))
            count = points.withUnsafeMutableBufferPointer { buf in
                Int(kinect2_get_point_cloud(dev, buf.baseAddress, buf.count, 0))
            }

            print("frame \(frameIndex): validDepth=\(validDepth) validPoints=\(count)")

            if validDepth > 10000 && count > 10000 {
                break
            }
        }

        guard count > 0 else {
            fputs("No valid points captured\n", stderr)
            exit(2)
        }

        do {
            try writePLY(points: points, count: count, to: URL(fileURLWithPath: "frame.ply"))
            print("Wrote frame.ply with \(count) points")
        } catch {
            fputs("PLY write failed: \(error)\n", stderr)
            exit(3)
        }
    }

    static func writePLY(points: [kinect2_point_t], count: Int, to url: URL) throws {
        var text = """
            ply
            format ascii 1.0
            element vertex \(count)
            property float x
            property float y
            property float z
            property uchar red
            property uchar green
            property uchar blue
            end_header

            """

        for i in 0..<count {
            let p = points[i]
            text += "\(p.x) \(p.y) \(p.z) \(p.r) \(p.g) \(p.b)\n"
        }

        try text.write(to: url, atomically: true, encoding: .utf8)
    }
}
