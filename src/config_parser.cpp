// Copyright 2019 Zhihao Zhang License MIT

#include "config_parser.hpp"

#include <iostream>
#include <string>
#include <fstream>
#include <algorithm>

namespace assignment2
{
ConfigReader::ConfigReader(std::istream & config_file)
{ 
    std::string line;
    int lineCount = 1;

    while(getline(config_file, line)){
        // printing config line with the line number
        std::cout << "Line" << " " << lineCount << " : " << line << '\n';
        lineCount++;

        // breaking up config into key and value and writing it into the unordered map
        line.erase(std::remove_if(line.begin(), line.end(), isspace), line.end());
            auto delimiterPos = line.find(":");
            auto key = line.substr(0, delimiterPos);
            auto keyVal = line.substr(delimiterPos + 1);
            ConfigReader::config_.insert ({key,keyVal});
    }
    // iterating and prininting out the key and value from the map
    for (auto& it: config_) {
        std::cout << "key: "<< it.first <<" " << "value: " << it.second << "\n";
    }
        
}

auto ConfigReader::find_config(std::string const & key,
    std::string const & default_value) const -> std::string
{
    
    return std::string{};
}

ConfigParser::ConfigParser(ConfigReader const & config)
    : zid_{config.find_config("zid", std::string{"z0000000"})}
    // TODO(STUDENT): CODE HERE
    , joy_config_{4,4,3,0.01,0.01}
    , kinematic_config_{1,1,1,1}
    , refresh_period_{1}
{
    // TODO(STUDENT): CODE HERE
}

auto ConfigParser::get_zid() const -> std::string { return zid_; }

auto ConfigParser::get_refresh_period() const -> std::chrono::milliseconds
{
    return refresh_period_;
}

auto ConfigParser::get_joystick_config() const -> JoystickConfig
{
    return joy_config_;
}

auto ConfigParser::get_kinematic_config() const -> KinematicLimits
{
    return kinematic_config_;
}
} // namespace assignment2
