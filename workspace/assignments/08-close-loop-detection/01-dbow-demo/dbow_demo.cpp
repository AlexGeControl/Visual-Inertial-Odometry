#include <iostream>
#include <string>
#include <vector>
#include <map>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp> 

#include <DBoW3/DBoW3.h>

struct ImageWithDescriptor {
    std::string filename;
    cv::Mat descriptor;
};

typedef std::map<size_t, ImageWithDescriptor> ImageTable;

const char kBuildDBoWWindow[] = "Build DBoW";

ImageTable GetImageTable(const std::string &input_dir) {
    boost::filesystem::path data_path(input_dir);
    boost::filesystem::directory_iterator end_itr;

    ImageTable image_table;

    // ORB detector:
    cv::Ptr<cv::ORB> detector = cv::ORB::create();
    // iterate through the data directory:
    for (boost::filesystem::directory_iterator itr(data_path); itr != end_itr; ++itr)
    {
        // only process regular file:
        if (boost::filesystem::is_regular_file(itr->path())) {
            ImageWithDescriptor image_with_descriptor;

            // get filename:
            image_with_descriptor.filename = itr->path().string();
            // load:
            cv::Mat image = cv::imread(image_with_descriptor.filename, cv::IMREAD_GRAYSCALE);
            // detect:
            std::vector<cv::KeyPoint> keypoints;
            // get descriptor. default descriptor will have shape [32, 500]:
            detector->detectAndCompute(image, cv::noArray(), keypoints, image_with_descriptor.descriptor);

            // update:
            image_table.emplace(image_table.size(), image_with_descriptor);
        }
    }

    return image_table;
}

void BuildVocabulary(
    const std::string output_voc_filename, const ImageTable &image_table,
    const int K = 10, const int LEVEL = 5
) {
    // build dictionary:
    DBoW3::Vocabulary voc(K, LEVEL, DBoW3::TF_IDF, DBoW3::L1_NORM);
    
    // extract features:
    std::vector<cv::Mat> features;
    for (size_t i = 0; i < image_table.size(); ++i) {
        features.push_back(image_table.at(i).descriptor);
    }

    // create dictionary:
    voc.create(features);

    // save:
    voc.save(output_voc_filename);
}

void BuildDatabase(
    const std::string output_db_filename, 
    const DBoW3::Vocabulary &voc,
    const ImageTable &image_table
) {
    DBoW3::Database db(voc, false, 0);

    // add documents:
    for (size_t i = 0; i < image_table.size(); ++i) {
        db.add(image_table.at(i).descriptor);
    }

    db.save(output_db_filename);
}

void DBoWDemo(const std::string &input_dir) {
    // get features:
    ImageTable image_table = GetImageTable(input_dir);

    // create vocabulary:
    BuildVocabulary("voc.yaml.gz", image_table);
    DBoW3::Vocabulary voc("voc.yaml.gz");

    // create database:
    BuildDatabase("db.yaml.gz", voc, image_table);
    DBoW3::Database db("db.yaml.gz");

    // testcase 01 -- match images against themselves:
    std::cout << "Matching images against themselves (0 low, 1 high): " << std::endl;
    for(size_t i = 0; i < image_table.size(); i++)
    {
        // identify target:
        std::string filename_target = boost::filesystem::path(image_table.at(i).filename).filename().string();
        DBoW3::BowVector target;
        voc.transform(image_table.at(i).descriptor, target);

        // for each source image, compute its L1-similarity with target:
        for(size_t j = 0; j < image_table.size(); j++)
        {   
            std::string filename_source = boost::filesystem::path(image_table.at(j).filename).filename().string();
            DBoW3::BowVector source;
            voc.transform(image_table.at(i).descriptor, source);

            double score = voc.score(target, source);
            
            std::cout << "Image " << filename_source << " vs Image " << filename_target << ": " << score << std::endl;
        }
        std::cout << std::endl;
    }

    // testcase 02 -- kNN using database:
    for(size_t i = 0; i < image_table.size(); i++)
    {
        std::string filename_target = boost::filesystem::path(image_table.at(i).filename).filename().string();
        DBoW3::QueryResults results;
        db.query(image_table.at(i).descriptor, results, 4);

        std::cout << "Top 4 Most Similar Images to Image " << filename_target << ": " << std::endl;

        for (const DBoW3::Result &result: results) {
            std::string filename_source = boost::filesystem::path(image_table.at(result.Id).filename).filename().string();
            std::cout << "\tImage " << filename_source << ", Score" << result.Score << std::endl;
        } 
    }
}

int main(int argc, char *argv[]) {
    // parse command line options:
    try
    {
        boost::program_options::options_description desc{"Build DBoW on input images"};
        desc.add_options()
            ("help,h", "Show help info")
            ("input,i", boost::program_options::value<std::string>()->default_value("../data"), "Input Directory")
            ("output,o", boost::program_options::value<std::string>()->default_value("."), "Output Directory");

        boost::program_options::variables_map vm;
        boost::program_options::store(parse_command_line(argc, argv, desc), vm);

        if (vm.count("help")) {
            std::cout << desc << std::endl;
        } else {
            // parse arguments:
            const std::string &INPUT_DIR = vm["input"].as<std::string>();
            const std::string &OUTPUT_DIR = vm["output"].as<std::string>();

            std::cout << "[Build DBoW]:" << std::endl;
            std::cout << "\tInput Directory: " << INPUT_DIR << std::endl;
            std::cout << "\tOutput Directory: " << OUTPUT_DIR << std::endl;

            DBoWDemo(INPUT_DIR);
        }
    }
    catch (const boost::program_options::error &ex)
    {
        std::cerr << ex.what() << '\n';
        return EXIT_FAILURE;
    } 

    return EXIT_SUCCESS;
}