#include <boost/preprocessor/cat.hpp>
#include <boost/preprocessor/stringize.hpp>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/spdlog.h>
#include <filesystem>

#define GENERATE_LOGGER(name)                                                                                   \
  namespace {                                                                                                   \
  std::shared_ptr<spdlog::logger> BOOST_PP_CAT(name, _logger)() {                                               \
    std::filesystem::create_directories("./log");                                                               \
    static std::shared_ptr<spdlog::logger> LOGGER = spdlog::get("Hsu " BOOST_PP_STRINGIZE(name) " Logger");     \
    if (!LOGGER) {                                                                                              \
      LOGGER = spdlog::basic_logger_mt("Hsu " BOOST_PP_STRINGIZE(name) " Logger",                               \
                                                                 "./log/Hsu_" BOOST_PP_STRINGIZE(name) ".log"); \
      LOGGER->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [thread %t] [%^%l%$] %v");                                    \
      LOGGER->info("==== New ====");                                                                            \
    }                                                                                                           \
    return LOGGER;                                                                                              \
  }                                                                                                             \
                                                                                                                \
  template <typename... Args>                                                                                   \
  __attribute__((always_inline)) void DEBUG(fmt::format_string<Args...> fmt, Args&&... args) {                  \
    BOOST_PP_CAT(name, _logger)()->debug(fmt, std::forward<Args>(args)...);                                     \
  }                                                                                                             \
  template <typename T>                                                                                         \
  __attribute__((always_inline)) void DEBUG(const T& msg) {                                                     \
    BOOST_PP_CAT(name, _logger)()->debug(msg);                                                                  \
  }                                                                                                             \
  template <typename... Args>                                                                                   \
  __attribute__((always_inline)) void INFO(fmt::format_string<Args...> fmt, Args&&... args) {                   \
    BOOST_PP_CAT(name, _logger)()->info(fmt, std::forward<Args>(args)...);                                      \
  }                                                                                                             \
  template <typename T>                                                                                         \
  __attribute__((always_inline)) void INFO(const T& msg) {                                                      \
    BOOST_PP_CAT(name, _logger)()->info(msg);                                                                   \
  }                                                                                                             \
  template <typename... Args>                                                                                   \
  __attribute__((always_inline)) void WARN(fmt::format_string<Args...> fmt, Args&&... args) {                   \
    BOOST_PP_CAT(name, _logger)()->warn(fmt, std::forward<Args>(args)...);                                      \
  }                                                                                                             \
  template <typename T>                                                                                         \
  __attribute__((always_inline)) void WARN(const T& msg) {                                                      \
    BOOST_PP_CAT(name, _logger)()->warn(msg);                                                                   \
  }                                                                                                             \
  template <typename... Args>                                                                                   \
  __attribute__((always_inline)) void ERROR(fmt::format_string<Args...> fmt, Args&&... args) noexcept(false) {  \
    BOOST_PP_CAT(name, _logger)()->error(fmt, std::forward<Args>(args)...);                                     \
    throw std::runtime_error(fmt::format(fmt, std::forward<Args>(args)...));                                    \
  }                                                                                                             \
  template <typename T>                                                                                         \
  __attribute__((always_inline)) void ERROR(const T& msg) noexcept(false) {                                     \
    BOOST_PP_CAT(name, _logger)()->error(msg);                                                                  \
    throw std::runtime_error(fmt::format("{}", msg));                                                           \
  }                                                                                                             \
  }