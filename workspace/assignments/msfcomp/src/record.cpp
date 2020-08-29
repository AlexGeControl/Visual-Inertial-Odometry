#include "record.hpp"

#include <iostream>
#include <sstream>

#include <ctime>
#include <locale>

#include <set>

#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>

namespace msfcomp {

void FormatTime(const double &epoch_time, char* time_output) {
    // std::locale::global(std::locale("ja_JP.utf8"));
    std::time_t t = static_cast<int>(epoch_time);
    struct tm * timeinfo = std::localtime(&t);

    if (std::strftime(time_output, 64, "%H:%M:%S", timeinfo)) {
        std::cout << time_output << std::endl;;
    }
}

bool Record::Load(const std::string &content) {
    boost::regex regex_content(CONTENT_FORMAT);

    // check file format:
    if (boost::regex_match(content, regex_content, boost::match_extra)) {
        boost::regex regex_key_value(KEY_VALUE_FORMAT);

        // extract fields:
        boost::sregex_token_iterator iter(content.begin(), content.end(), regex_key_value, 0);
        boost::sregex_token_iterator end;
        for( ; iter != end; ++iter ) {
            std::string field = *iter;
            boost::smatch what;

            boost::regex_search(field, what, regex_key_value);

            // update fields:
            const std::string &key = what[1];
            const std::string &value = what[2];
            fields_.insert(
                std::pair<std::string, std::string>(key,value) 
            );
        }

        // finally:
        return true;
    } 
    
    return false;
}

void Record::Compare(Record &record, const int &timestamp, const std::vector<std::string> &ignore_keys) {
    // init ignore key set:
    std::set<std::string> ignore_key_set;
    for (const std::string &ignore_key: ignore_keys) {
        ignore_key_set.insert(ignore_key);
    }

    // summary output:
    std::vector<std::string> ok, error;

    // create ID:
    const std::string id = fields_.find("id")->second;
    const std::string market = fields_.find("/e")->second;
    const std::string signature = id + "." + market;
    
    // create timestamp:
    const std::string &source_rcvt = fields_.find("/rcvt")->second;
    const std::string &target_rcvt = record.fields_.find("/rcvt")->second;

    char source_time[64], target_time[64];

    double source_epoch_time = boost::lexical_cast<double>(source_rcvt) + timestamp;
    double target_epoch_time = boost::lexical_cast<double>(target_rcvt) + timestamp;

    FormatTime(source_epoch_time, source_time);
    FormatTime(target_epoch_time, target_time);

    // output buffers:
    std::stringstream fields_ok, fields_error;
    std::stringstream details;

    for (const auto &field: fields_) {
        // identify key:
        const std::string &key = field.first;

        // skip id and timestamp:
        if (!key.compare("id") || !key.compare("/rcvt")) {
            continue;
        }

        // identify field name:
        const std::string field_name = key.substr(1, std::string::npos);

        // skip keys in ignore key set:
        if (ignore_key_set.find(field_name) != ignore_key_set.end()) {
            continue;
        }

        if (record.fields_.find(key) != record.fields_.end()) {
            // extract values:
            const std::string &source_value = field.second;
            const std::string &target_value = record.fields_.find(key)->second;

            // compare:
            const std::string status = (source_value.compare(target_value)) ? "ERROR" : "OK";

            // update result:
            details << "\t| " << signature << " | "
                      << field_name << " | "
                      << source_value << " | "
                      << " <--> "
                      << target_value << " | "
                      << status << " |" 
                      << std::endl;

            // add to group OK:
            ok.push_back(key);
            fields_ok << field_name << " ";
        } else {
            // add to group ERROR:
            error.push_back(key);
            fields_error << field_name << " ";
        }
    }

    // display comparison results:
    std::cout << "Timestamp:" << std::endl
              << "\tSource: " << source_time << "." << source_rcvt.substr(11, std::string::npos) << " GMT" << std::endl
              << "\tTarget: " << target_time << "." << target_rcvt.substr(11, std::string::npos) << " GMT" << std::endl
              << "Summary:" << std::endl
              << "\t| " << signature << " | OK | " << fields_ok.str() << "|" << std::endl
              << "\t| " << signature << " | ERROR | " << fields_error.str() << "|" << std::endl
              << "Details:" << std::endl
              << details.str() << std::endl;
}

} // namespace msfcomp