#include <iostream>
#include <string>

#include <boost/program_options.hpp>

#include "record.hpp"

int main(int argc, char *argv[]) {
    // parse command line options:
    try
    {
        boost::program_options::options_description desc{"Compare file content for further analysis"};
        desc.add_options()
            ("help,h", "Show help info")
            ("timestamp,t",  boost::program_options::value<int>()->default_value(0), "Time reference")
            ("ignore_key,i", boost::program_options::value<std::string>()->default_value(""), "Exclude the listed keys from comparison");

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

            const std::string content("id 000001 /e sz /tcnt 4054 /vwpsum 785473289.09 /vwpvs 85024326 /vwp 9.238218 /hp 9.28 /lp 9.17 /uptcnt 888 /upvwpsum 214065838.39 /upvs 23152952 /upvwp 9.245725 /tcnt10b18 -99999 /vwpsum10b18 -99999 /vwpvs10b18 -99999 /vwp10b18 -99999 /twp 9.236318 /lgts 1909900 /lgtp 9.28 /twb 9.235732 /twa 9.245784 /tws 0.010053 /twbs -99999 /twas -99999 /twss -99999 /hb 9.27 /lb 9.17 /ha 9.28 /la 9.18 /bvs 43103391 /avs 39779389 /intvs 85024326 /vs 85024326 /otp -99999 /pb -99999 /pa -99999 /omp -99999 /op 9.18 /ob 9.18 /oa 9.19 /ccp 9.24 /tp 9.23 /cb 9.23 /ca 9.24 /bp 9.24 /ap 9.25 /clb 9.24 /cla 9.25 /rcvt 1479704303.077500 /;");
            
            msfcomp::Record record;

            if (!record.Load(content)) {
                std::cerr << "Invalid content. Please check your input format." << std::endl;
                return EXIT_FAILURE; 
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