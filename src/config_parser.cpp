// Copyright 2019 Zhihao Zhang License MIT

#include "config_parser.hpp"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>

namespace assignment2
{
ConfigReader::ConfigReader(std::istream & config_file)
{
    std::string line;
    int lineCount = 1;

    while (getline(config_file, line))
    {
        // printing config line with the line number
        std::cout << "Line"
                  << " " << lineCount << " : " << line << '\n';
        lineCount++;

        // breaking up config into key and value and writing it into the
        // unordered map
        line.erase(
            std::remove_if(line.begin(), line.end(), isspace), line.end());
        auto delimiterPos = line.find(":");
        auto key = line.substr(0, delimiterPos);
        auto keyVal = line.substr(delimiterPos + 1);
        ConfigReader::config_.insert({key, keyVal});
    }
    // iterating and prininting out the key and value from the map
    for (auto & it : config_)
    {
        std::cout << "key: " << it.first << " "
                  << "value: " << it.second << "\n";
    }
}

auto ConfigReader::find_config(std::string const & key,
    std::string const & default_value) const -> std::string
{
    std::string configValue;
    // iterate through map keys to find matching key
    for (auto & it : config_)
    {
        if (it.first == key)
        {
            // return value to key if keys match
            configValue = {it.second};
            return configValue;
            
        }
    }
    // return default value if no matches found
    configValue = {default_value};
    return configValue;
}

ConfigParser::ConfigParser(ConfigReader const & config)
    // parsing key and default values into the finder to find a config value
    // that matches the key or return the default if no match is found
    : zid_{config.find_config("zid", std::string{"z0000000"})}
    , joy_config_{
        std::stod(config.find_config("speed_plus_axis",std::string{"0"})),
        std::stod(config.find_config("speed_minus_axis",std::string{"1"})),
        std::stod(config.find_config("steering_deadzone",std::string{"0.1"})),
        std::stod(config.find_config("speed_deadzone",std::string{"0.1"}))
    
    }
    , kinematic_config_{
        std::stod(config.find_config("max_linear_speed",std::string{"5"})),
        std::stod(config.find_config("max_angular_speed",std::string{"5"})),
        std::stod(config.find_config("max_linear_acceleration",std::string{"3"})),
        std::stod(config.find_config("max_angular_acceleration",std::string{"3"}))
    }
    , refresh_period_{
        std::stoi(config.find_config("refresh_rate",std::string{"10"}))
    }
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
