#ifndef CONFIGFILE_H
#define CONFIGFILE_H

#include "mbed.h"
#include <vector>
#include <string>
#include <map>
#include <fstream>

class ConfigFile
{
public:
    ConfigFile(){};

    bool load(const char *fname);
    float get(const std::string& param){return inputs[param];}
    int size(){return inputs.size();}
    const std::map<std::string, float>& map(){return inputs;}

private:
    std::map<std::string, float> inputs;

};

static std::vector<std::string> split(const std::string& input, char delimiter);


#endif //CONFIGFILE_H