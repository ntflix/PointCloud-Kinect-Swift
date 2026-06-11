// PLYWriter.swift
import CKinect2Bridge
import Foundation

/// Writes a point cloud as a binary-little-endian PLY file.
///
/// Layout per vertex (15 bytes, matching header order):
///   float32 x  (4 bytes)
///   float32 y  (4 bytes)
///   float32 z  (4 bytes)
///   uint8   red   (1 byte)
///   uint8   green (1 byte)
///   uint8   blue  (1 byte)
///
/// The header is ASCII; the body begins immediately after the final
/// newline of "end_header\n" with no padding or alignment.
enum PLYWriter {

    // MARK: - Public API

    static func writeBinary(
        points: [kinect2_point_t],
        count: Int,
        to url: URL
    ) throws {
        precondition(count <= points.count)

        let header = makeHeader(vertexCount: count)
        let headerBytes = Array(header.utf8)

        let bytesPerVertex = 15  // 3×float32 + 3×uint8
        var body = [UInt8]()
        body.reserveCapacity(count * bytesPerVertex)

        for i in 0..<count {
            let p = points[i]
            appendFloat32(&body, p.x)
            appendFloat32(&body, p.y)
            appendFloat32(&body, p.z)
            body.append(p.r)
            body.append(p.g)
            body.append(p.b)
        }

        var fileData = Data(headerBytes)
        fileData.append(contentsOf: body)

        try fileData.write(to: url, options: .atomic)
    }

    // MARK: - Helpers

    private static func makeHeader(vertexCount: Int) -> String {
        var h = ""
        h += "ply\r\n"
        h += "format binary_little_endian 1.0\r\n"
        h += "element vertex \(vertexCount)\r\n"
        h += "property float x\r\n"
        h += "property float y\r\n"
        h += "property float z\r\n"
        h += "property uchar red\r\n"
        h += "property uchar green\r\n"
        h += "property uchar blue\r\n"
        h += "end_header\r\n"
        return h
    }

    /// Appends a Float as 4 bytes, little-endian.
    /// On Apple Silicon and x86 the host is already little-endian,
    /// but we use CFSwapInt32HostToLittle for portability.
    @inline(__always)
    private static func appendFloat32(_ buffer: inout [UInt8], _ value: Float) {
        var bits = value.bitPattern  // UInt32, same bit pattern
        bits = CFSwapInt32HostToLittle(bits)  // no-op on LE hardware
        buffer.append(UInt8(bits & 0xFF))
        buffer.append(UInt8((bits >> 8) & 0xFF))
        buffer.append(UInt8((bits >> 16) & 0xFF))
        buffer.append(UInt8((bits >> 24) & 0xFF))
    }
}
