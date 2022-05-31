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
        .binaryTarget(name: "SQNPG", url: "https://www.sqsurvey.com/packages/iOS/SQNPG/v1.3.1/SQNPG.xcframework.zip", checksum: "7c2f3b1942bc5f45b1c5f0576095c2b5f9c325bccdcea0e0e293c040f53d5b4e")
    ]
)
