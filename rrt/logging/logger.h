// logger.h

#ifndef LOGGER_H
#define LOGGER_H

#include <string>
#include <fstream>
#include <unordered_map>
#include <sstream>
#include <stdexcept>
#include <util/util.h>

template <typename Loggable>
class Logger {
public:
    explicit Logger(const std::string& fp, bool _with_episode = true, bool _header = true)
        : filepath_(fp),
          file_(fp),
          header_written_(!_header),
          episode_(0),
          with_episode_(_with_episode) {
        if (!file_.is_open()) {
            throw std::runtime_error("Failed to open log file.");
        }
    }

    ~Logger() {
        if (file_.is_open()) {
            file_.close();
        }
    }

    void log(const Loggable& loggable) {
        if (!header_written_) {
            if (with_episode_) {
                file_ << "episode,";
            }
            file_ << loggable.log_header() << "\n";
            header_written_ = true;
        }
        if (with_episode_) {
            file_ << episode_ << ',';
        }
        file_ << loggable.log() << "\n";
        file_.flush();
    }

    void increment_episode() {
        ++episode_;
    }

private:
    std::string filepath_;
    std::ofstream file_;
    bool header_written_;
    int episode_;
    bool with_episode_;
};

// Partial specialization for std::unordered_map<Key, Loggable>.
// Allows for convenient logging of related items.
template <typename Key, typename Loggable>
class Logger<std::unordered_map<Key, Loggable>> {
public:
    explicit Logger(const std::string& fp, bool _with_episode = true, bool _header = true)
        : filepath_(fp),
          file_(fp),
          header_written_(!_header),
          episode_(0),
          with_episode_(_with_episode) {
        if (!file_.is_open()) {
            throw std::runtime_error("Failed to open log file.");
        }
    }

    ~Logger() {
        if (file_.is_open()) {
            file_.close();
        }
    }

    void log(const std::unordered_map<Key, Loggable>& loggable_map) {
        if (!header_written_) {
            std::ostringstream header_stream;
            if (with_episode_) {
                header_stream << "episode,";
            }
            bool first = true;
            for (const auto& [key, item] : loggable_map) {
                if (!first) {
                    header_stream << ',';
                }
                std::string with_suffix = rrt::util::append_suffix_to_list(item.log_header(), key);
                header_stream << with_suffix;
                first = false;
            }
            file_ << header_stream.str() << "\n";
            header_written_ = true;
        }

        std::ostringstream log_stream;
        if (with_episode_) {
            log_stream << episode_ << ',';
        }
        bool first = true;
        for (const auto& [key, item] : loggable_map) {
            if (!first) {
                log_stream << ',';
            }
            log_stream << item.log();
            first = false;
        }
        auto str = log_stream.str();
        file_ << str << "\n";
        file_.flush();
    }

    void increment_episode() {
        ++episode_;
    }

private:
    std::string filepath_;
    std::ofstream file_;
    bool header_written_;
    int episode_;
    bool with_episode_;
};

#endif // LOGGER_H
