//
// Created by Alex Gisi on 10/3/24.
//

#ifndef RRT_PLANNER_LOGGER_H
#define RRT_PLANNER_LOGGER_H

#include <string>
#include <fstream>

template <typename Loggable>
class Logger {
public:
    explicit Logger(const std::string& fp, const bool _with_episode = true, const bool _header = true)
    : filepath(fp), file(fp), header_written(!_header), episode(0), with_episode(_with_episode) {
        if (!file.is_open()) {
            throw std::runtime_error("Failed to open log file.");
        }
    }

    ~Logger() {
        if (file.is_open()) {
            file.close();
        }
    }

    void log(Loggable loggable) {
        if (!header_written) {
            if (with_episode) {
                file << "episode,";
            }
            file << loggable.log_header() << "\n";
            header_written = true;
        }
        if (with_episode) {
            file << episode << ',';
        }
        file << loggable.log() << "\n";
    }

    void increment_episode() {
        ++episode;
    }

private:
    std::string filepath;
    std::ofstream file;
    bool header_written;
    int episode;
    bool with_episode;
};

#endif //RRT_PLANNER_LOGGER_H
