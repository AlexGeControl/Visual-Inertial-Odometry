#include "record.hpp"

#include <iostream>
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
        for (const auto &field: fields_) {
            std::cout << "\t\t" << field.first << ", " << field.second << std::endl;
        }
        
        return true;
    } 
    
    return false;
}

} // namespace msfcomp