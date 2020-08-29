#include <iostream>

#include <boost/program_options.hpp>

int main(int argc, char *argv[]) {
    // parse command line options:
    try
    {
        boost::program_options::options_description desc{"Compare file content for further analysis"};
        desc.add_options()
            ("help,h", "Show help info")
            ("timestamp,t", "Time reference")
            ("ignore_key,i", "Exclude the listed keys from comparison");

        boost::program_options::variables_map vm;
        boost::program_options::store(parse_command_line(argc, argv, desc), vm);

        if (vm.count("help")) {
            std::cout << desc << std::endl;
        } else {
            // parse arguments:
            const int &TIMESTAMP = vm["timestamp"].as<int>();
            const std::string &IGNORE_KEYS = vm["ignore_key"].as<std::string>();

            std::cout << "msfcomp:" << std::endl;
            std::cout << "\ttimestamp: " << TIMESTAMP << std::endl;
            std::cout << "\tignore_key: " << IGNORE_KEYS << std::endl;
        }
    }
    catch (const boost::program_options::error &ex)
    {
        std::cerr << ex.what() << '\n';
        return EXIT_FAILURE;
    } 

    return EXIT_SUCCESS;
}