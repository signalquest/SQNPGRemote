// swift-tools-version:5.3
// The swift-tools-version declares the minimum version of Swift required to build this package.

import PackageDescription

let package = Package(
    name: "SQNPGRemote",
    products: [
        // Products define the executables and libraries a package produces, and make them visible to other packages.
        .library(
            name: "SQNPG",
            targets: ["SQNPG"]
        ),
    ],
    dependencies: [],
    targets: [
        .binaryTarget(
            name: "SQNPG",
            url: "https://signalquest.net/sq-survey/packages/iOS/SQNPG/v1.3.0b/SQNPG.xcframework.zip",
            checksum: "cefdf05b150b71250df7991ef0986e6272152c2baf64666d7f5755b8778e47d1"
        )
    ]
)
