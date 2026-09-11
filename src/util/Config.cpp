#include "util/Config.h"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <fstream>
#include <sstream>

std::string trim(const std::string& s) {
    size_t a = 0, b = s.size();
    while (a < b && std::isspace((unsigned char)s[a])) ++a;
    while (b > a && std::isspace((unsigned char)s[b - 1])) --b;
    return s.substr(a, b - a);
}

std::vector<std::string> splitString(const std::string& s, char sep) {
    std::vector<std::string> out;
    std::string cur;
    for (char c : s) {
        if (c == sep) { out.push_back(cur); cur.clear(); }
        else cur += c;
    }
    out.push_back(cur);
    return out;
}

std::string toLower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return (char)std::tolower(c); });
    return s;
}

static std::string stripComment(const std::string& line) {
    bool inQuote = false;
    for (size_t i = 0; i < line.size(); ++i) {
        char c = line[i];
        if (c == '"') inQuote = !inQuote;
        if (!inQuote && (c == '#' || c == ';')) return line.substr(0, i);
    }
    return line;
}

static std::string sectionOf(const std::string& fullKey) {
    size_t dot = fullKey.find('.');
    return dot == std::string::npos ? std::string() : fullKey.substr(0, dot);
}
static std::string nameOf(const std::string& fullKey) {
    size_t dot = fullKey.find('.');
    return dot == std::string::npos ? fullKey : fullKey.substr(dot + 1);
}

bool Config::loadFile(const std::string& path, std::string* err) {
    std::ifstream f(path);
    if (!f) { if (err) *err = "cannot open config file: " + path; return false; }
    path_ = path;
    std::string line, section;
    int lineNo = 0;
    while (std::getline(f, line)) {
        ++lineNo;
        std::string s = trim(stripComment(line));
        if (s.empty()) continue;
        if (s.front() == '[' && s.back() == ']') { section = toLower(trim(s.substr(1, s.size() - 2))); continue; }
        size_t eq = s.find('=');
        if (eq == std::string::npos) {
            if (err) *err = path + ":" + std::to_string(lineNo) + ": expected key = value";
            return false;
        }
        std::string key = toLower(trim(s.substr(0, eq)));
        std::string val = trim(s.substr(eq + 1));
        if (val.size() >= 2 && val.front() == '"' && val.back() == '"') val = val.substr(1, val.size() - 2);
        kv_[section.empty() ? key : section + "." + key] = val;
    }
    return true;
}

bool Config::saveFile(const std::string& path) const {
    // Rewrite: keep the original structure, replace values of known keys, append new keys to their section.
    std::vector<std::string> lines;
    {
        std::ifstream f(path);
        std::string l;
        while (f && std::getline(f, l)) lines.push_back(l);
    }
    std::map<std::string, std::string> pending = kv_;
    std::vector<std::string> out;
    std::string section;
    auto flushSection = [&](const std::string& sec) {
        std::vector<std::string> keys;
        for (auto& kv : pending)
            if (sectionOf(kv.first) == sec) keys.push_back(kv.first);
        for (auto& k : keys) {
            out.push_back(nameOf(k) + " = " + pending[k]);
            pending.erase(k);
        }
    };
    for (size_t i = 0; i < lines.size(); ++i) {
        const std::string& line = lines[i];
        std::string s = trim(stripComment(line));
        if (!s.empty() && s.front() == '[' && s.back() == ']') {
            flushSection(section);
            section = toLower(trim(s.substr(1, s.size() - 2)));
            out.push_back(line);
            continue;
        }
        size_t eq = s.find('=');
        if (!s.empty() && eq != std::string::npos) {
            std::string key = toLower(trim(s.substr(0, eq)));
            std::string full = section.empty() ? key : section + "." + key;
            auto it = pending.find(full);
            if (it != pending.end()) {
                std::string comment;
                size_t c = line.find('#');
                if (c != std::string::npos) comment = "   " + line.substr(c);
                out.push_back(key + " = " + it->second + comment);
                pending.erase(it);
                continue;
            }
        }
        out.push_back(line);
    }
    flushSection(section);
    std::string lastSec = "<none>";
    for (auto& kv : pending) {
        std::string sec = sectionOf(kv.first);
        if (sec != lastSec) { out.push_back(""); out.push_back("[" + sec + "]"); lastSec = sec; }
        out.push_back(nameOf(kv.first) + " = " + kv.second);
    }
    std::ofstream f(path);
    if (!f) return false;
    for (auto& l : out) f << l << "\n";
    return true;
}

void Config::set(const std::string& key, const std::string& value) { kv_[toLower(key)] = value; }

bool Config::setFromArg(const std::string& kv, std::string* err) {
    size_t eq = kv.find('=');
    if (eq == std::string::npos) { if (err) *err = "expected section.key=value, got: " + kv; return false; }
    set(trim(kv.substr(0, eq)), trim(kv.substr(eq + 1)));
    return true;
}

bool Config::has(const std::string& key) const { return kv_.count(toLower(key)) > 0; }

std::string Config::getString(const std::string& key, const std::string& def) const {
    auto it = kv_.find(toLower(key));
    return it == kv_.end() ? def : it->second;
}

double Config::getDouble(const std::string& key, double def) const {
    auto it = kv_.find(toLower(key));
    if (it == kv_.end()) return def;
    char* end = nullptr;
    double v = std::strtod(it->second.c_str(), &end);
    return (end && end != it->second.c_str()) ? v : def;
}

int Config::getInt(const std::string& key, int def) const {
    auto it = kv_.find(toLower(key));
    if (it == kv_.end()) return def;
    char* end = nullptr;
    long v = std::strtol(it->second.c_str(), &end, 10);
    return (end && end != it->second.c_str()) ? (int)v : def;
}

bool Config::getBool(const std::string& key, bool def) const {
    auto it = kv_.find(toLower(key));
    if (it == kv_.end()) return def;
    std::string v = toLower(trim(it->second));
    if (v == "1" || v == "true" || v == "yes" || v == "on") return true;
    if (v == "0" || v == "false" || v == "no" || v == "off") return false;
    return def;
}

std::vector<double> Config::getDoubleList(const std::string& key, const std::vector<double>& def) const {
    auto it = kv_.find(toLower(key));
    if (it == kv_.end()) return def;
    std::vector<double> out;
    for (auto& p : splitString(it->second, ',')) {
        std::string t = trim(p);
        if (t.empty()) continue;
        out.push_back(std::atof(t.c_str()));
    }
    return out.empty() ? def : out;
}
