import CKinect2Bridge
import Foundation

@main
struct PointCloud {

    static func main() {
        // ── Parse args ───────────────────────────────────────────────────
        let args = CommandLine.arguments
        var captureDuration: Double = 5.0
        var outputDir = URL(fileURLWithPath: FileManager.default.currentDirectoryPath)

        var i = 1
        while i < args.count {
            switch args[i] {
            case "--seconds", "-s":
                i += 1
                if i < args.count, let v = Double(args[i]) {
                    captureDuration = v
                } else {
                    fputs("Expected a number after \(args[i - 1])\n", stderr)
                    exit(1)
                }
            case "--output", "-o":
                i += 1
                if i < args.count {
                    outputDir = URL(fileURLWithPath: args[i])
                } else {
                    fputs("Expected a path after \(args[i - 1])\n", stderr)
                    exit(1)
                }
            default:
                fputs("Unknown argument: \(args[i])\n", stderr)
                exit(1)
            }
            i += 1
        }

        // ── Create output directory ───────────────────────────────────────
        do {
            try FileManager.default.createDirectory(
                at: outputDir,
                withIntermediateDirectories: true
            )
        } catch {
            fputs("Cannot create output directory '\(outputDir.path)': \(error)\n", stderr)
            exit(1)
        }

        print("Capturing for \(captureDuration)s → \(outputDir.path)")

        // ── Open device ───────────────────────────────────────────────────
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

        // ── Per-frame point buffer ────────────────────────────────────────
        let maxPoints = Int(kinect2_depth_width() * kinect2_depth_height())
        var points = Array(
            repeating: kinect2_point_t(x: 0, y: 0, z: 0, r: 0, g: 0, b: 0, a: 0),
            count: maxPoints
        )

        // ── Capture loop ──────────────────────────────────────────────────
        let startTime = Date()
        var frameIndex = 0
        var savedFrames = 0

        while Date().timeIntervalSince(startTime) < captureDuration {
            guard kinect2_wait_frame(dev, 2000) == 0 else {
                fputs("frame \(frameIndex): timeout\n", stderr)
                frameIndex += 1
                continue
            }

            let count = points.withUnsafeMutableBufferPointer { buf in
                Int(kinect2_get_point_cloud(dev, buf.baseAddress, buf.count, 0))
            }

            let elapsed = Date().timeIntervalSince(startTime)
            print(String(format: "[%6.2fs] frame %04d: %d points", elapsed, frameIndex, count))

            if count > 0 {
                let filename = String(format: "frame_%04d.ply", savedFrames)
                let url = outputDir.appendingPathComponent(filename)
                do {
                    try PLYWriter.writeBinary(points: points, count: count, to: url)
                    savedFrames += 1
                } catch {
                    fputs("Failed to write \(filename): \(error)\n", stderr)
                }
            }

            frameIndex += 1
        }

        if savedFrames == 0 {
            fputs("No valid frames captured in \(captureDuration)s\n", stderr)
            exit(2)
        }

        print("Done. \(savedFrames) frames written to \(outputDir.path)")
    }

    // ── PLY writer ────────────────────────────────────────────────────────
    static func writePLY(points: [kinect2_point_t], count: Int, to url: URL) throws {
        // Pre-allocate: header ~120 bytes + ~40 bytes per point
        var text = String()
        text.reserveCapacity(128 + count * 40)

        text += "ply\n"
        text += "format ascii 1.0\n"
        text += "element vertex \(count)\n"
        text += "property float x\n"
        text += "property float y\n"
        text += "property float z\n"
        text += "property uchar red\n"
        text += "property uchar green\n"
        text += "property uchar blue\n"
        text += "end_header\n"

        for i in 0..<count {
            let p = points[i]
            text += "\(p.x) \(p.y) \(p.z) \(p.r) \(p.g) \(p.b)\n"
        }

        try text.write(to: url, atomically: true, encoding: .utf8)
    }
}
