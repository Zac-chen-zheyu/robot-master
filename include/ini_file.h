#pragma once
// ---------------------------------------------------------------------------
//  ini_file.h —— 极简 INI 读取（支持 [section]，; 与 # 注释）
// ---------------------------------------------------------------------------
#include <cstdlib>
#include <fstream>
#include <map>
#include <string>

namespace mortat
{

    class IniFile
    {
    public:
        bool load(const std::string &path, std::string *err = nullptr)
        {
            std::ifstream in(path.c_str());
            if (!in)
            {
                if (err)
                    *err = "打不开配置文件：" + path;
                return false;
            }

            std::string line;
            std::string section;
            while (std::getline(in, line))
            {
                if (!line.empty() && line[line.size() - 1] == '\r')
                    line.erase(line.size() - 1);
                line = trim(line);
                if (line.empty() || line[0] == ';' || line[0] == '#')
                    continue;

                if (line[0] == '[')
                {
                    const size_t e = line.find(']');
                    if (e != std::string::npos)
                        section = trim(line.substr(1, e - 1));
                    continue;
                }

                const size_t eq = line.find('=');
                if (eq == std::string::npos)
                    continue;

                std::string k = trim(line.substr(0, eq));
                std::string v = trim(line.substr(eq + 1));

                const size_t c = v.find_first_of(";#");
                if (c != std::string::npos)
                    v = trim(v.substr(0, c));
                if (k.empty())
                    continue;

                const std::string s = section.empty() ? std::string("global") : section;
                kv_[s + "." + k] = v;
            }
            return true;
        }

        bool has(const std::string &sec, const std::string &key) const
        {
            return kv_.find(sec + "." + key) != kv_.end();
        }

        std::string getStr(const std::string &sec, const std::string &key,
                           const std::string &def) const
        {
            const std::map<std::string, std::string>::const_iterator it =
                kv_.find(sec + "." + key);
            return (it == kv_.end()) ? def : it->second;
        }

        double getDbl(const std::string &sec, const std::string &key, double def) const
        {
            const std::string s = getStr(sec, key, std::string());
            if (s.empty())
                return def;
            char *end = nullptr;
            const double v = std::strtod(s.c_str(), &end);
            return (end == s.c_str()) ? def : v;
        }

        int getInt(const std::string &sec, const std::string &key, int def) const
        {
            const std::string s = getStr(sec, key, std::string());
            if (s.empty())
                return def;
            char *end = nullptr;
            const long v = std::strtol(s.c_str(), &end, 10);
            return (end == s.c_str()) ? def : (int)v;
        }

        bool getBool(const std::string &sec, const std::string &key, bool def) const
        {
            std::string s = getStr(sec, key, std::string());
            for (size_t i = 0; i < s.size(); ++i)
            {
                if (s[i] >= 'A' && s[i] <= 'Z')
                    s[i] = (char)(s[i] - 'A' + 'a');
            }
            if (s.empty())
                return def;
            if (s == "1" || s == "true" || s == "yes" || s == "on")
                return true;
            if (s == "0" || s == "false" || s == "no" || s == "off")
                return false;
            return def;
        }

    private:
        static std::string trim(const std::string &s)
        {
            const size_t a = s.find_first_not_of(" \t");
            if (a == std::string::npos)
                return std::string();
            const size_t b = s.find_last_not_of(" \t");
            return s.substr(a, b - a + 1);
        }

        std::map<std::string, std::string> kv_;
    };

} // namespace mortat
