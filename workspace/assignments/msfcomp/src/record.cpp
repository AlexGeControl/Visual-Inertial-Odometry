#include "record.hpp"

#include <iostream>
#include <sstream>

#include <boost/regex.hpp>

namespace msfcomp {

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

void Record::Compare(Record &record) {
    // summary output:
    std::vector<std::string> ok, error;

    // create ID:
    const std::string id = fields_.find("id")->second;
    const std::string market = fields_.find("/e")->second;
    const std::string signature = id + "." + market;

    // output buffers:
    std::stringstream fields_ok, fields_error;
    std::stringstream details;

    for (const auto &field: fields_) {
        // identify field:
        const std::string &key = field.first;
        const std::string field_name = key.substr(1, std::string::npos);

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
    std::cout << "Summary:" << std::endl
              << "\t| " << signature << " | OK | " << fields_ok.str() << "|" << std::endl
              << "\t| " << signature << " | ERROR | " << fields_error.str() << "|" << std::endl
              << "Details:" << std::endl
              << details.str() << std::endl;
}

} // namespace msfcomp