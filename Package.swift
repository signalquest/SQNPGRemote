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
        .binaryTarget(name: "SQNPG", url: "https://www.sqsurvey.com/packages/iOS/SQNPG/v1.1.0/SQNPG.xcframework.zip", checksum: "74d7d63f4bdbd0aafb1f6854db5bd13723bbd3cf64600d56694f7d4b63b9694b")
    ]
)
