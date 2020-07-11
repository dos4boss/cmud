#pragma once

#ifndef SPDLOG_ACTIVE_LEVEL
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE
#endif

#include "spdlog/spdlog.h"
#include "spdlog/common.h"
#include "spdlog/details/circular_q.h"
#include "spdlog/details/log_msg_buffer.h"
#include "spdlog/details/null_mutex.h"

#include <mutex>
#include <string>
#include <vector>
#include <string_view>


namespace logger {

  template <std::string_view const &... Strs> struct string_view_join {
    // Helper to get a string literal from a std::array
    template <std::size_t N, std::array<char, N> const &S, typename>
    struct to_char_array;
    template <std::size_t N, std::array<char, N> const &S, std::size_t... I>
    struct to_char_array<N, S, std::index_sequence<I...>> {
      static constexpr const char value[]{S[I]..., 0};
    };
    // Join all strings into a single std::array of chars
    static constexpr auto impl() noexcept {
      constexpr std::size_t len = (Strs.size() + ... + 0);
      std::array<char, len + 1> arr{};
      auto append = [i = 0, &arr](auto const &s) mutable {
                      for (auto c : s)
                        arr[i++] = c;
                    };
      (append(Strs), ...);
      arr[len] = 0;
      return arr;
    }
    // Give the joined string static storage
    static constexpr auto arr = impl();
    // Convert to a string literal, then view as a std::string_view
    static constexpr std::string_view value =
      to_char_array<arr.size(), arr,
                    std::make_index_sequence<arr.size()>>::value;
  };

  namespace format {
    // Formatting codes
    inline constexpr std::string_view reset {"\033[m"};
    inline constexpr std::string_view bold = "\033[1m";
    inline constexpr std::string_view dark = "\033[2m";
    inline constexpr std::string_view underline = "\033[4m";
    inline constexpr std::string_view blink = "\033[5m";
    inline constexpr std::string_view reverse = "\033[7m";
    inline constexpr std::string_view concealed = "\033[8m";
    inline constexpr std::string_view clear_line = "\033[K";
  } // namespace format

  namespace fg {
    // Foreground colors
    inline constexpr std::string_view black = "\033[30m";
    inline constexpr std::string_view red = "\033[31m";
    inline constexpr std::string_view green = "\033[32m";
    inline constexpr std::string_view yellow = "\033[33m";
    inline constexpr std::string_view blue = "\033[34m";
    inline constexpr std::string_view magenta = "\033[35m";
    inline constexpr std::string_view cyan = "\033[36m";
    inline constexpr std::string_view white = "\033[37m";
  }

  namespace bg {
  /// Background colors
    inline constexpr std::string_view on_black = "\033[40m";
    inline constexpr std::string_view on_red = "\033[41m";
    inline constexpr std::string_view on_green = "\033[42m";
    inline constexpr std::string_view on_yellow = "\033[43m";
    inline constexpr std::string_view on_blue = "\033[44m";
    inline constexpr std::string_view on_magenta = "\033[45m";
    inline constexpr std::string_view on_cyan = "\033[46m";
    inline constexpr std::string_view on_white = "\033[47m";
  }
} // namespace logger

constexpr std::string_view comb = logger::string_view_join<logger::fg::yellow, logger::format::bold>::value;

namespace spdlog {
  namespace sinks {
    template<typename ConsoleMutex>
    class ringbuffer_color_sink final : public sink
    {
    public:
      using mutex_t = typename ConsoleMutex::mutex_t;
      explicit ringbuffer_color_sink(size_t n_items, color_mode mode)
          : q_{n_items},
            mutex_(ConsoleMutex::mutex()),
            formatter_(details::make_unique<spdlog::pattern_formatter>())
      {
        set_color_mode(mode);
        colors_[level::trace] = ::logger::fg::white;
        colors_[level::debug] = ::logger::fg::cyan;
        colors_[level::info] = ::logger::fg::green;
        colors_[level::warn] = ::logger::string_view_join<::logger::fg::yellow, ::logger::format::bold>::value;
        colors_[level::err] = ::logger::string_view_join<::logger::fg::red, ::logger::format::bold>::value;
        colors_[level::critical] = ::logger::string_view_join<::logger::format::bold, ::logger::bg::on_red>::value;
        colors_[level::off] = ::logger::format::reset;
      }

      SPDLOG_INLINE void set_pattern(const std::string &pattern) override {
        std::lock_guard<mutex_t> lock(mutex_);
        formatter_ =
            std::unique_ptr<spdlog::formatter>(new pattern_formatter(pattern));
      }

      SPDLOG_INLINE void set_formatter(std::unique_ptr<spdlog::formatter> sink_formatter) override {
        std::lock_guard<mutex_t> lock(mutex_);
        formatter_ = std::move(sink_formatter);
      }

      SPDLOG_INLINE void set_color(level::level_enum color_level,
                                   std::string color) {
        std::lock_guard<mutex_t> lock(mutex_);
        colors_[color_level] = color;
      }

      SPDLOG_INLINE void set_color_mode(color_mode mode) {
        switch (mode) {
        case color_mode::always:
          should_do_colors_ = true;
          return;
        case color_mode::automatic:
          should_do_colors_ = details::os::is_color_terminal();
          return;
        case color_mode::never:
          should_do_colors_ = false;
          return;
        }
      }

      std::vector<details::log_msg_buffer> last_raw(size_t lim = 0) {
        std::lock_guard<mutex_t> lock(mutex_);
        auto items_available = q_.size();
        auto n_items = lim > 0 ? (std::min)(lim, items_available) : items_available;
        std::vector<details::log_msg_buffer> ret;
        ret.reserve(n_items);
        for (size_t i = (items_available - n_items); i < items_available; i++) {
          ret.push_back(q_.at(i));
        }
        return ret;
      }

      std::vector<std::string> last_formatted(size_t lim = 0)
      {
          std::lock_guard<mutex_t> lock(mutex_);
          auto items_available = q_.size();
          auto n_items = lim > 0 ? (std::min)(lim, items_available) : items_available;
          std::vector<std::string> ret;
          ret.reserve(n_items);
          for (size_t i = (items_available - n_items); i < items_available; i++)
          {
            memory_buf_t formatted;
            auto &msg = q_.front(); //  at(i);
            msg.color_range_start = 0;
            msg.color_range_end = 0;
            formatter_->format(msg, formatted);
            if(should_do_colors_ && msg.color_range_end > msg.color_range_start) {
              memory_buf_t formatted_colored;
              formatted_colored.append(formatted.data(), formatted.data() + msg.color_range_start);
              formatted_colored.append(colors_[msg.level].begin(), colors_[msg.level].end());
              formatted_colored.append(formatted.data() + msg.color_range_start,
                                       formatted.data() + msg.color_range_end);
              formatted_colored.append(::logger::format::reset.begin(),
                                       ::logger::format::reset.end());
              formatted_colored.append(formatted.data() + msg.color_range_end,
                                       formatted.data() + formatted.size());
              ret.push_back(fmt::to_string(formatted_colored));
            }
            else
              ret.push_back(fmt::to_string(formatted));
            q_.pop_front();
          }
          return ret;
      }

    protected:
      void log(const details::log_msg &msg) override {
        q_.push_back(details::log_msg_buffer{msg});
    }
    void flush() override {}

private:
    details::circular_q<details::log_msg_buffer> q_;
    mutex_t &mutex_;
    std::unique_ptr<spdlog::formatter> formatter_;
    bool should_do_colors_;
    std::array<std::string_view, level::n_levels> colors_;
};

using ringbuffer_color_sink_mt = ringbuffer_color_sink<details::console_mutex>;
using ringbuffer_color_sink_st = ringbuffer_color_sink<details::console_nullmutex>;

} // namespace sinks

} // namespace spdlog

namespace logger {

  class LoggerSink {
  public:
    static auto &getInstance() {
      static auto instance = std::make_shared<spdlog::sinks::ringbuffer_color_sink_mt>(10,
                                                                                       spdlog::color_mode::automatic);
      if(!initialized_) {
        instance->set_level(spdlog::level::trace);
        instance->set_pattern("%^[%n] [%l] %v%$");
        initialized_ = true;
      }

      return instance;
    }

    static std::string get() {
      const auto res_vector = getInstance()->last_formatted();
      std::string result;
      for(const auto &res : res_vector)
        result += res;
      return result;
    }

  private:
    LoggerSink() { initialized_ = false; }
    inline static bool initialized_;

  public:
    LoggerSink(LoggerSink const &) = delete;
    void operator=(LoggerSink const &) = delete;
  };

  class RAIIFlush {
  public:
    RAIIFlush(std::ostream &out)
      : out_(out) {
    }

    ~RAIIFlush() {
      const auto &res_vec = LoggerSink::get();
      for(const auto &res : res_vec)
        out_ << res;
    }

  private:
    std::ostream &out_;
  };

}

#define MAKE_LOCAL_LOGGER(x) auto local_logger = \
  spdlog::logger(x, logger::LoggerSink::getInstance());

#define LOGGER_TRACE(...) SPDLOG_LOGGER_TRACE(&local_logger, __VA_ARGS__)
#define LOGGER_DEBUG(...) SPDLOG_LOGGER_DEBUG(&local_logger, __VA_ARGS__)
#define LOGGER_INFO(...) SPDLOG_LOGGER_INFO(&local_logger, __VA_ARGS__)
#define LOGGER_WARN(...) SPDLOG_LOGGER_WARN(&local_logger, __VA_ARGS__)
#define LOGGER_ERROR(...) SPDLOG_LOGGER_ERROR(&local_logger, __VA_ARGS__)
#define LOGGER_CRITICAL(...) SPDLOG_LOGGER_CRITICAL(&local_logger, __VA_ARGS__)


// #define LOGGER_DEBUG(...)                                                      \
//   (&local_logger)->log(spdlog::source_loc{__FILE__, __LINE__, SPDLOG_FUNCTION}, spdlog::level::trace, __VA_ARGS__)
// #define LOGGER_INFO(...)                                                       \
//   (&local_logger)->log(spdlog::source_loc{__FILE__, __LINE__, SPDLOG_FUNCTION}, spdlog::level::trace, __VA_ARGS__)
// #define LOGGER_WARN(...)                                                       \
//   (&local_logger)->log(spdlog::source_loc{__FILE__, __LINE__, SPDLOG_FUNCTION}, spdlog::level::trace, __VA_ARGS__)
// #define LOGGER_ERROR(...)                                                      \
//   (&local_logger)->log(spdlog::source_loc{__FILE__, __LINE__, SPDLOG_FUNCTION}, spdlog::level::trace, __VA_ARGS__)
// #define LOGGER_CRITICAL(...)                                                   \
//   (&local_logger)->log(spdlog::source_loc{__FILE__, __LINE__, SPDLOG_FUNCTION}, spdlog::level::trace, __VA_ARGS__)
