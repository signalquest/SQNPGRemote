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
        .binaryTarget(name: "SQNPG", url: "https://www.sqsurvey.com/packages/iOS/SQNPG/v1.2.0/SQNPG.xcframework.zip", checksum: "5c0418e3d86859f2d1846f1c31a6d71fd851f08e75aa1949db9e7f4565dbd86c")
    ]
)
