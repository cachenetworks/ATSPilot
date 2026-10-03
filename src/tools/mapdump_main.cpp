// Map research and validation tool.
//
//   atspilot_mapdump ls   <archive.scs> [dir]        list a directory inside an archive
//   atspilot_mapdump cat  <archive.scs> <file> [out] extract one file
//   atspilot_mapdump build <game_dir> [cache_file]   parse the full map and print statistics
//   atspilot_mapdump locate <cache_file> <x> <z> [heading]   localize a world position

#include <fstream>
#include <iostream>
#include <string>

#include "map/HashFs.h"

using namespace atspilot;

int runMapCommands(int argc, char** argv);  // defined in mapdump_map.cpp

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "usage: atspilot_mapdump ls|cat|build|locate ...\n";
        return 2;
    }
    const std::string cmd = argv[1];
    if (cmd == "ls" || cmd == "cat") {
        std::string err;
        auto a = HashFsArchive::open(argv[2], &err);
        if (!a) {
            std::cerr << "open failed: " << err << "\n";
            return 1;
        }
        std::cerr << "HashFS v" << a->version() << ", " << a->entryCount() << " entries\n";
        if (cmd == "ls") {
            const auto l = a->list(argc > 3 ? argv[3] : "");
            if (!l) {
                std::cerr << "directory not found\n";
                return 1;
            }
            for (const auto& d : l->subdirectories) std::cout << d << "/\n";
            for (const auto& f : l->files) std::cout << f << "\n";
            return 0;
        }
        if (argc < 4) return 2;
        const auto data = a->read(argv[3], &err);
        if (!data) {
            std::cerr << "read failed: " << err << "\n";
            return 1;
        }
        if (argc > 4) {
            std::ofstream(argv[4], std::ios::binary).write(data->data(), static_cast<std::streamsize>(data->size()));
        } else {
            std::cout.write(data->data(), static_cast<std::streamsize>(data->size()));
        }
        return 0;
    }
    return runMapCommands(argc, argv);
}
