#include "record.hpp"

#include <iostream>
#include <boost/regex.hpp>

namespace msfcomp {

bool Record::Load(const std::string &content) {
    boost::regex regex_content(CONTENT_FORMAT);

    if (boost::regex_match(content, regex_content, boost::match_extra)) {
        boost::regex regex_key_value(KEY_VALUE_FORMAT);

        boost::sregex_token_iterator iter(content.begin(), content.end(), regex_key_value, 0);
        boost::sregex_token_iterator end;
        
        for( ; iter != end; ++iter ) {
            std::cout<<*iter<<std::endl;
        }

        return true;
    } 
    
    return false;
}

} // namespace msfcomp