#include "ConfigFile.h"
#include <stdlib.h>
#include <sstream>
// #include <algorithm>
// #include <cctype>

bool ConfigFile::load(const char *fname){
    char *endptr;
    std::string s;
    std::vector<std::string> strVec;
    std::ifstream ifs;

    if(!inputs.empty())
        inputs.clear();
    ifs.open(fname);
    if(!ifs)
        return false;
    while(getline(ifs,s)){
        strVec = split(s,':');
        if(strVec.size() == 2)
            inputs[strVec[0]] = strtof(strVec[1].c_str(),&endptr);
    }

    return true;
}

//http://faithandbrave.hateblo.jp/entry/2014/05/01/171631 より抜粋
std::vector<std::string> split(const std::string& input, char delimiter)
{
    std::istringstream stream(input);

    std::string field;
    std::vector<std::string> result;
    while (std::getline(stream, field, delimiter)) {
        result.push_back(field);
    }
    return result;
}

