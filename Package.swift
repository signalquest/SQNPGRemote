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
        .binaryTarget(name: "SQNPG", url: "https://www.sqsurvey.com/packages/iOS/SQNPG/v1.3.0/SQNPG.xcframework.zip", checksum: "86877a940168378cf19c02685446be42ffa5cd94b6a74f3e8227d85451d41a4d")
    ]
)
