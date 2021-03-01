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
        .binaryTarget(name: "SQNPG", url: "https://www.sqsurvey.com/packages/iOS/SQNPG/v1.0.1/SQNPG.xcframework.zip", checksum: "2822c4a3560b0716c8b601f753dfca31ae83954fe119a2f19e285104f17e774c")
    ]
)
