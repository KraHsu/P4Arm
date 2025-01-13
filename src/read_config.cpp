#include <fmt/base.h>
#include <fmt/format.h>
#include <nlohmann/json-schema.hpp>
#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>
#include <regex>
#include <stdexcept>
#include <string>
#include <read_config.h>

// 命名空间简化
using json = nlohmann::json;
using namespace nlohmann::json_schema;

#define DEBUG(...) \
  spdlog::log(spdlog::source_loc{__FILE__, __LINE__, SPDLOG_FUNCTION}, spdlog::level::debug, __VA_ARGS__)
#define INFO(...) spdlog::log(spdlog::source_loc{__FILE__, __LINE__, SPDLOG_FUNCTION}, spdlog::level::info, __VA_ARGS__)
#define WARN(...) spdlog::log(spdlog::source_loc{__FILE__, __LINE__, SPDLOG_FUNCTION}, spdlog::level::warn, __VA_ARGS__)
#define ERROR(...) spdlog::log(spdlog::source_loc{__FILE__, __LINE__, SPDLOG_FUNCTION}, spdlog::level::err, __VA_ARGS__)

// 辅助函数：递归将 YAML 节点转换为 nlohmann::json
json yaml_to_json(const YAML::Node& node) {
  if (node.IsScalar()) {
    try {
      return node.as<int>();
    } catch (...) {
    }
    try {
      return node.as<double>();
    } catch (...) {
    }
    try {
      return node.as<bool>();
    } catch (...) {
    }
    return node.as<std::string>();
  } else if (node.IsSequence()) {
    json array = json::array();
    for (auto it = node.begin(); it != node.end(); ++it) {
      array.push_back(yaml_to_json(*it));
    }
    return array;
  } else if (node.IsMap()) {
    json obj = json::object();
    for (auto it = node.begin(); it != node.end(); ++it) {
      obj[it->first.as<std::string>()] = yaml_to_json(it->second);
    }
    return obj;
  }
  return nullptr;
}

std::pair<std::string, int> parse_ip_port(const std::string& input) {
  // 匹配 "IP:PORT" 格式
  const std::regex pattern(R"((\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}):(\d+))");

  std::smatch match;
  if (!std::regex_match(input, match, pattern)) {
    throw std::invalid_argument(fmt::format("不是合法格式（IP:PORT）：{}", input));
  }

  std::string ip = match[1];
  int port = std::stoi(match[2]);

  std::istringstream ip_stream(ip);
  std::string segment;
  int segment_value;
  while (std::getline(ip_stream, segment, '.')) {
    segment_value = std::stoi(segment);
    if (segment_value < 0 || segment_value > 255) {
      throw std::invalid_argument(fmt::format("IP 地址格式错误: {}", segment));
    }
  }

  if (port < 1 || port > 65535) {
    throw std::invalid_argument(fmt::format("PORT 范围是 [1, 65535]: {}", port));
  }

  return {ip, port};
}

json get_default_config() {
  return R"({"Actor":{"ArmL":"Hand","ArmR":"Hand","OffsetL":{"Position":[0,0,0],"Euler":[0,0,0]},"OffsetR":{"Position")"
         R"(:[0,0,0],"Euler":[0,0,0]}},"Address":{"ArmL":"192.168.1.18:8080","ArmR":"192.168.2.18:8080","HandL":"192.1)"
         R"(68.12.210:6000","HandR":"192.168.11.210:6000"},"Speed":10})"_json;
}

json get_config_schema() {
  return R"({"type":"object","properties":{"Actor":{"type":"object","properties":{"ArmL":{"type":"string","enum":["Han)"
         R"(d","Grip"]},"ArmR":{"type":"string","enum":["Hand","Grip"]},"OffsetL":{"type":"object","properties":{"Posi)"
         R"(tion":{"type":"array","items":{"type":"number"},"minItems":3,"maxItems":3},"Euler":{"type":"array","items")"
         R"(:{"type":"number"},"minItems":3,"maxItems":3}},"required":["Position","Euler"]},"OffsetR":{"type":"object")"
         R"(,"properties":{"Position":{"type":"array","items":{"type":"number"},"minItems":3,"maxItems":3},"Euler":{"t)"
         R"(ype":"array","items":{"type":"number"},"minItems":3,"maxItems":3}},"required":["Position","Euler"]}},"requ)"
         R"(ired":["ArmL","ArmR","OffsetL","OffsetR"]},"Address":{"type":"object","properties":{"ArmL":{"type":"string)"
         R"("},"ArmR":{"type":"string"},"HandL":{"type":"string"},"HandR":{"type":"string"}},"required":["ArmL","ArmR")"
         R"(]},"Speed":{"type":"integer","minimum":1,"maximum":100}},"required":["Actor","Address","Speed"]})"_json;
}

json read_config(const std::string& file_path) {
  try {
    json_validator validator;
    validator.set_root_schema(get_config_schema());

    json default_config = get_default_config();

    YAML::Node yaml_config = YAML::LoadFile(file_path);
    json loaded_config = yaml_to_json(yaml_config);

    try {
      validator.validate(loaded_config);
    } catch (const std::exception& e) {
      WARN("配置文件格式错误：{}使用默认配置", e.what());
    }

    default_config.merge_patch(loaded_config);

    validator.validate(default_config);

    INFO("使用配置：{}", default_config.dump());

    return default_config;
  } catch (const YAML::Exception& e) {
    throw std::runtime_error("读取 YAML 配置文件失败：" + std::string(e.what()));
  } catch (const std::exception& e) {
    throw std::runtime_error("配置文件验证失败：" + std::string(e.what()));
  }
}