#include <boost/preprocessor/cat.hpp>
#include <boost/preprocessor/stringize.hpp>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/spdlog.h>
#include <filesystem>

#define GENERATE_LOGGER(name)                                                                        \
  constexpr auto __LOGGER_NAME = "Hsu " BOOST_PP_STRINGIZE(name) " Logger";                          \
  namespace {                                                                                        \
  std::shared_ptr<spdlog::logger> BOOST_PP_CAT(name, _logger) = []() {                               \
    std::filesystem::create_directories("./log");                                                    \
    static std::shared_ptr<spdlog::logger> LOGGER = spdlog::get(__LOGGER_NAME);                      \
    if (!LOGGER) {                                                                                   \
      LOGGER = spdlog::basic_logger_mt(__LOGGER_NAME, "./log/Hsu_" BOOST_PP_STRINGIZE(name) ".log"); \
      LOGGER->set_pattern("[%H:%M:%S.%e] [thread %t] [%^%l%$] %s:%# %! %v");                         \
      LOGGER->info("==== New ====");                                                                 \
    }                                                                                                \
    return LOGGER;                                                                                   \
  }();                                                                                               \
  }

#define DEBUG(...)           \
  spdlog::get(__LOGGER_NAME) \
      ->log(spdlog::source_loc{__FILE__, __LINE__, SPDLOG_FUNCTION}, spdlog::level::debug, __VA_ARGS__)
#define INFO(...)            \
  spdlog::get(__LOGGER_NAME) \
      ->log(spdlog::source_loc{__FILE__, __LINE__, SPDLOG_FUNCTION}, spdlog::level::info, __VA_ARGS__)
#define WARN(...)            \
  spdlog::get(__LOGGER_NAME) \
      ->log(spdlog::source_loc{__FILE__, __LINE__, SPDLOG_FUNCTION}, spdlog::level::warn, __VA_ARGS__)
#define ERROR(...)           \
  spdlog::get(__LOGGER_NAME) \
      ->log(spdlog::source_loc{__FILE__, __LINE__, SPDLOG_FUNCTION}, spdlog::level::err, __VA_ARGS__)