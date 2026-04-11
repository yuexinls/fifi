#pragma once
#include <iostream>
#include <sstream>
#include <iomanip>
#include <string>
#include <unordered_map>
#include <chrono>

// thresholds for warnings
constexpr float WARN_SPEED_LINEAR  = 20.0f;
constexpr float WARN_SPEED_ANGULAR = 20.0f;
constexpr float WARN_PENETRATION   = 0.5f;
constexpr float WARN_IMPULSE       = 50.0f;
constexpr float WARN_BAD_NORMAL_SQ = 0.99f; // normal.lengthSq() should be ~1 😻

// throttled logger
struct DebugLog {
    static DebugLog& get() {
        static DebugLog inst;
        return inst;
    }

    void warn(const std::string& key, const std::string& msg,
              double interval = 1.0)
    {
        double now = timeNow();
        auto   it  = m_lastPrint.find(key);
        if (it == m_lastPrint.end() || now - it->second >= interval) {
            std::cerr << "[WARN] " << msg << "\n";
            m_lastPrint[key] = now;
        }
        m_warnCount[key]++;
    }

    void info(const std::string& msg) {
        std::cout << "[INFO] " << msg << "\n";
    }

    int count(const std::string& key) const {
        auto it = m_warnCount.find(key);
        return it != m_warnCount.end() ? it->second : 0;
    }

    void reset() {
        m_lastPrint.clear();
        m_warnCount.clear();
    }

private:
    std::unordered_map<std::string, double> m_lastPrint;
    std::unordered_map<std::string, int>    m_warnCount;

    static double timeNow() {
        using namespace std::chrono;
        static auto start = steady_clock::now();
        return duration<double>(steady_clock::now() - start).count();
    }
};

#define DWARN(key, msg)            DebugLog::get().warn(key, msg)
#define DWARN3(key, msg, interval) DebugLog::get().warn(key, msg, interval)
#define DINFO(msg)                 DebugLog::get().info(msg)


