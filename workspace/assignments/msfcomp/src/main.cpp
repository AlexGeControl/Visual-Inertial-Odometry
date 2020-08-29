#include <iostream>
#include <fstream> 

#include <string>
#include <vector>

#include <boost/program_options.hpp>

#include "record.hpp"

namespace po = boost::program_options;

enum INPUT_FILE_TYPE {
    SOURCE = 0,
    TARGET,
    NUM_TYPES
};

std::string ReadFile(const std::string &filename) {
    std::ifstream ifs(filename.c_str(), std::ios::in | std::ios::binary | std::ios::ate);

    std::ifstream::pos_type filesize = ifs.tellg();
    ifs.seekg(0, std::ios::beg);

    std::vector<char> bytes(filesize);
    ifs.read(bytes.data(), filesize);

    return std::string(bytes.data(), filesize);
}

int main(int argc, char *argv[]) {
    // parse command line options:
    try
    {   
        // command line arguments:
        int timestamp;

        po::options_description desc{"Compare file content for further analysis"};
        desc.add_options()
            ("help,h", "Show help info")
            ("input_files", po::value<std::vector<std::string>>(), "Input files")
            ("timestamp,t",  po::value<int>(&timestamp), "Time reference")
            ("ignore_key,i", po::value<std::vector<std::string>>()->multitoken(), "Exclude the listed keys from comparison");
        po::positional_options_description pos;
        pos.add("input_files", 2);

        po::variables_map vm;
        po::store(
            po::command_line_parser(argc, argv).options(desc).positional(pos).run(), 
            vm
        );
        po::notify(vm);

        if (vm.count("help")) {
            std::cout << desc << std::endl;
        } else {
            // parse arguments:
            const std::vector<std::string> &input_files = vm["input_files"].as<std::vector<std::string>>();
            const std::vector<std::string> &ignore_keys = vm["ignore_key"].as<std::vector<std::string>>();

            std::cout << "Command-Line Arguments:" << std::endl;

            std::cout << "\tinput_files: " << std::endl;
            for (size_t i = 0; i < input_files.size(); ++i) {
                std::cout << "\t\t" << input_files.at(i) << std::endl;
            }
            std::cout << "\ttimestamp: " << timestamp << std::endl;
            std::cout << "\tignore_key: " << std::endl;
            for (size_t i = 0; i < ignore_keys.size(); ++i) {
                std::cout << "\t\t" << ignore_keys.at(i) << std::endl;
            }

            // load input files:
            msfcomp::Record source, target;
            if (
                !source.Load(ReadFile(input_files.at(INPUT_FILE_TYPE::SOURCE))) ||
                !target.Load(ReadFile(input_files.at(INPUT_FILE_TYPE::TARGET)))
            ) {
                std::cerr << "Invalid content. Please check your input format." << std::endl;
                return EXIT_FAILURE; 
            } else {
                // perform compare:
                source.Compare(target, ignore_keys);
            }
        }
    }
    catch (const boost::program_options::error &ex)
    {
        std::cerr << ex.what() << '\n';
        return EXIT_FAILURE;
    } 

    return EXIT_SUCCESS;
}