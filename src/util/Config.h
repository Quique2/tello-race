#pragma once
#include <map>
#include <string>
#include <vector>

// Minimal INI-style configuration: "[section]" headers and "key = value" lines.
// Keys are addressed as "section.key". '#' and ';' start comments.
class Config {
public:
    bool loadFile(const std::string& path, std::string* err = nullptr);
    bool saveFile(const std::string& path) const;   // rewrites the file keeping structure/comments
    void set(const std::string& key, const std::string& value);
    bool setFromArg(const std::string& kv, std::string* err = nullptr);   // "section.key=value"
    bool has(const std::string& key) const;

    std::string getString(const std::string& key, const std::string& def) const;
    double getDouble(const std::string& key, double def) const;
    int getInt(const std::string& key, int def) const;
    bool getBool(const std::string& key, bool def) const;
    std::vector<double> getDoubleList(const std::string& key, const std::vector<double>& def) const;

    const std::map<std::string, std::string>& all() const { return kv_; }
    const std::string& path() const { return path_; }

private:
    std::map<std::string, std::string> kv_;
    std::string path_;
};

std::string trim(const std::string& s);
std::vector<std::string> splitString(const std::string& s, char sep);
std::string toLower(std::string s);
