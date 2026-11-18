// ---------------------------------------------------------------------------
//  detect_images.cpp —— 离线批量验证识别
//
//  用法:
//    ./detect_images 图片目录 [--out 调试图目录] [--config config/aim.ini]
//    ./detect_images a.png b.png ...
//
//  输出一张表：文件名 / 是否找到 / 三角尖端 / 圆方是否同色 / 高分区侧 / 编号 / 置信度
//  可以拿 附件_识别高分区 里那 8 张图直接验。
// ---------------------------------------------------------------------------
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#if defined(__linux__) || defined(__APPLE__)
#include <dirent.h>
#endif

#include "aim_config.h"
#include "hiscore_detector.h"

namespace
{

    bool isImageName(const std::string &n)
    {
        const size_t dot = n.find_last_of('.');
        if (dot == std::string::npos)
            return false;
        std::string ext = n.substr(dot + 1);
        for (size_t i = 0; i < ext.size(); ++i)
        {
            if (ext[i] >= 'A' && ext[i] <= 'Z')
                ext[i] = (char)(ext[i] - 'A' + 'a');
        }
        return ext == "png" || ext == "jpg" || ext == "jpeg" || ext == "bmp" || ext == "webp";
    }

    std::string joinPath(const std::string &dir, const std::string &name)
    {
        if (dir.empty())
            return name;
        if (dir[dir.size() - 1] == '/')
            return dir + name;
        return dir + "/" + name;
    }

    std::vector<std::string> listImages(const std::string &dir)
    {
        std::vector<std::string> out;
#if defined(__linux__) || defined(__APPLE__)
        DIR *d = opendir(dir.c_str());
        if (!d)
            return out;
        while (struct dirent *e = readdir(d))
        {
            const std::string n = e->d_name;
            if (n == "." || n == "..")
                continue;
            if (isImageName(n))
                out.push_back(joinPath(dir, n));
        }
        closedir(d);
#else
        (void)dir;
#endif
        std::sort(out.begin(), out.end());
        return out;
    }

} // namespace

int main(int argc, char **argv)
{
    using namespace mortat;

    if (argc < 2)
    {
        std::printf("用法: detect_images <图片目录|图片文件...> [--out 调试图目录] [--config x.ini]\n");
        return 1;
    }

    std::string outDir;
    std::string cfgPath;
    std::vector<std::string> files;
    std::vector<std::string> dirs;

    for (int i = 1; i < argc; ++i)
    {
        const std::string a(argv[i]);
        if (a == "--out" && i + 1 < argc)
        {
            outDir = argv[++i];
        }
        else if (a == "--config" && i + 1 < argc)
        {
            cfgPath = argv[++i];
        }
        else if (isImageName(a))
        {
            files.push_back(a);
        }
        else
        {
            dirs.push_back(a);
        }
    }

    AppConfig cfg;
    if (!cfgPath.empty())
    {
        std::string err;
        if (!cfg.load(cfgPath, &err))
        {
            std::printf("!! %s\n", err.c_str());
        }
    }

    for (size_t i = 0; i < dirs.size(); ++i)
    {
        const std::vector<std::string> got = listImages(dirs[i]);
        files.insert(files.end(), got.begin(), got.end());
    }
    if (files.empty())
    {
        std::printf("!! 没找到图片\n");
        return 2;
    }

    HisCoreDetector detector(cfg.detect);

    std::printf("%-28s %-5s %-4s %-5s %-7s %-3s %-5s %-7s %s\n",
                "文件", "找到", "尖端", "同色", "高分区", "id", "评分", "中心x", "底板宽");
    std::printf("--------------------------------------------------------------------------------\n");

    int ok = 0;
    int total = 0;
    for (size_t i = 0; i < files.size(); ++i)
    {
        const cv::Mat img = cv::imread(files[i], cv::IMREAD_COLOR);
        if (img.empty())
        {
            std::printf("%-28s  读不出来，跳过\n", files[i].c_str());
            continue;
        }
        ++total;
        const DetectResult r = detector.detect(img);
        ++ok;

        std::string base = files[i];
        const size_t slash = base.find_last_of('/');
        if (slash != std::string::npos)
            base = base.substr(slash + 1);

        if (!r.found)
        {
            std::printf("%-28s %-5s\n", base.c_str(), "否");
            continue;
        }
        const TagInfo &t = r.tag;
        std::printf("%-28s %-5s %-4s %-5s %-7s %-3d %-5.2f %-7.1f %.1f%s\n",
                    base.c_str(), "是", sideName(t.apex),
                    t.same_color ? "是" : "否", sideName(t.high),
                    t.id, t.score, t.center.x, t.width_px,
                    t.shape_only ? "  (退化:只有彩色图形)" : "");

        if (!outDir.empty() && !r.debug.empty())
        {
            const std::string p = joinPath(outDir, "dbg_" + base);
            cv::imwrite(p, r.debug);
        }
    }

    std::printf("--------------------------------------------------------------------------------\n");
    std::printf("共 %d 张，处理 %d 张\n", total, ok);
    return 0;
}
