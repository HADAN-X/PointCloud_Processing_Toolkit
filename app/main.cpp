#include <iostream>
#include <string_view>

namespace {

void printHelp() {
    std::cout
        << "PointCloud Processing Toolkit\n\n"
        << "Usage:\n"
        << "  pointcloud_tool --help\n"
        << "  pointcloud_tool --version\n";
}

}  // namespace

int main(int argc, char* argv[]) {
    if (argc == 1) {
        printHelp();
        return 0;
    }

    if (argc != 2) {
        std::cerr << "Error: expected exactly one argument.\n";
        printHelp();
        return 2;
    }

    const std::string_view argument{argv[1]};

    if (argument == "--help" || argument == "-h") {
        printHelp();
        return 0;
    }

    if (argument == "--version") {
        std::cout << "pointcloud_tool " << PCT_VERSION << '\n';
        return 0;
    }

    std::cerr << "Error: unknown argument: " << argument << '\n';
    printHelp();
    return 2;
}
