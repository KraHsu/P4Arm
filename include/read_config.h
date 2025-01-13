#pragma once

#include <nlohmann/json.hpp>
#include <nlohmann/json_fwd.hpp>

std::pair<std::string, int> parse_ip_port(const std::string& input);
nlohmann::json read_config(std::string const& file_path);
nlohmann::json get_default_config();
nlohmann::json get_config_schema();