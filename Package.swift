// swift-tools-version:5.3
// The swift-tools-version declares the minimum version of Swift required to build this package.

import PackageDescription

let package = Package(
    name: "SQNPGRemote",
    products: [
        // Products define the executables and libraries a package produces, and make them visible to other packages.
        .library(
            name: "SQNPG",
            targets: ["SQNPG"]),
    ],
    dependencies: [],
    targets: [
        .binaryTarget(name: "SQNPG", url: "https://signalquest.net/sq-survey/packages/iOS/SQNPG/v1.2.0b/SQNPG.xcframework.zip", checksum: "484b46e7d504c5f2189bcbca3ee3f96b55df92246386951e544fec6c6648f7e0")
    ]
)
