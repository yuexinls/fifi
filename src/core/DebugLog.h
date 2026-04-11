#pragma once
#include <iostream>
#include <string>
#include <unordered_map>
#include <GLFW/glfw3.h>

// throttled logger
struct DebugLog {
    static DebugLog& get() { static DebugLog inst; return inst; }

    void warn(const std::string& key, const std::string& msg,
              double interval = 1.0)
    {
        double now = glfwGetTime();
        auto it    = m_lastPrint.find(key);
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
};

#define DWARN(key, msg) DebugLog::get().warn(key, msg)
#define DINFO(msg)      DebugLog::get().info(msg)


