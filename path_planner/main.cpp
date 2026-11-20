#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <queue>
#include <fstream>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <zbar.h> // 引入 ZBar

// 用于实现终端免回车监听 ESC 退出
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

using namespace std;
using namespace cv;

// 辅助函数：终端无阻塞监听按键
int kbhit()
{
    struct termios oldt, newt;
    int ch;
    int oldf;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);
    if (ch != EOF)
    {
        ungetc(ch, stdin);
        return 1;
    }
    return 0;
}

// ================= 1. 扫码模块（迭代为 ZBar） =================
string scanQRCode()
{
    VideoCapture cap(0); // 如果打不开外置摄像头，改成 1
    if (!cap.isOpened())
    {
        cerr << "错误：无法打开摄像头！请检查连接。" << endl;
        return "";
    }

    zbar::ImageScanner scanner;
    scanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);

    Mat frame, gray;
    string decodedInfo = "";

    cout << "=== ZBar 扫码模式已启动 ===" << endl;
    cout << "请将二维码对准摄像头... (按 ESC 键取消扫码)" << endl;

    while (true)
    {
        cap >> frame;
        if (frame.empty())
            break;

        cvtColor(frame, gray, COLOR_BGR2GRAY);

        int width = gray.cols;
        int height = gray.rows;
        uchar *raw = (uchar *)(gray.data);
        zbar::Image image(width, height, "Y800", raw, width * height);

        scanner.scan(image);

        for (zbar::Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol)
        {
            if (symbol->get_type_name() == "QR-Code")
            {
                decodedInfo = symbol->get_data();
                break;
            }
        }

        if (!decodedInfo.empty())
        {
            cout << "扫码成功！" << endl;
            break;
        }

        putText(frame, "Scanning... (ESC to cancel)", Point(20, 50), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 0, 255), 2);
        imshow("Scanner", frame);

        if (waitKey(10) == 27)
            break; // ESC 退出扫码
    }
    cap.release();
    destroyWindow("Scanner");
    return decodedInfo;
}

// ================= 2. 解析数据（保持原样） =================
bool parseQRData(const string &qrStr, int &rows, int &cols, pair<int, int> &start, pair<int, int> &end, vector<vector<int>> &grid)
{
    size_t colonPos = qrStr.find(':');
    if (colonPos == string::npos)
        return false;

    string head = qrStr.substr(0, colonPos);
    string tail = qrStr.substr(colonPos + 1);

    for (char &ch : head)
        if (ch == ',')
            ch = ' ';
    stringstream ssHead(head);
    int sr, sc, er, ec;
    if (!(ssHead >> rows >> cols >> sr >> sc >> er >> ec))
        return false;
    start = {sr, sc};
    end = {er, ec};

    vector<int> flatCosts;
    stringstream ssTail(tail);
    string token;
    while (ssTail >> token)
    {
        size_t dashPos = token.find('-');
        if (dashPos != string::npos)
        {
            int val = stoi(token.substr(0, dashPos));
            int count = stoi(token.substr(dashPos + 1));
            for (int i = 0; i < count; ++i)
                flatCosts.push_back(val);
        }
    }

    if (flatCosts.size() != rows * cols)
    {
        cerr << "数据长度不匹配! 期望 " << rows * cols << " 个, 实际 " << flatCosts.size() << " 个" << endl;
        return false;
    }

    grid.assign(rows, vector<int>(cols, 0));
    for (int i = 0; i < rows; ++i)
    {
        for (int j = 0; j < cols; ++j)
        {
            grid[i][j] = flatCosts[i * cols + j];
        }
    }
    return true;
}

// ================= 3. Dijkstra 寻路（保持原样） =================
struct State
{
    int cost;
    int r, c;
    bool operator>(const State &other) const
    {
        return cost > other.cost;
    }
};

vector<pair<int, int>> dijkstra(const vector<vector<int>> &grid, pair<int, int> start, pair<int, int> end, int &minCost)
{
    int rows = grid.size();
    int cols = grid[0].size();
    vector<vector<int>> dist(rows, vector<int>(cols, 1e9));
    vector<vector<pair<int, int>>> prev(rows, vector<pair<int, int>>(cols, {-1, -1}));
    priority_queue<State, vector<State>, greater<State>> pq;

    dist[start.first][start.second] = 0;
    pq.push({0, start.first, start.second});

    int dr[] = {-1, 1, 0, 0};
    int dc[] = {0, 0, -1, 1};

    while (!pq.empty())
    {
        State curr = pq.top();
        pq.pop();
        int d = curr.cost, r = curr.r, c = curr.c;

        if (d > dist[r][c])
            continue;
        if (r == end.first && c == end.second)
            break;

        for (int i = 0; i < 4; ++i)
        {
            int nr = r + dr[i], nc = c + dc[i];
            if (nr >= 0 && nr < rows && nc >= 0 && nc < cols && grid[nr][nc] != 0)
            {
                int newCost = d + grid[nr][nc];
                if (newCost < dist[nr][nc])
                {
                    dist[nr][nc] = newCost;
                    prev[nr][nc] = {r, c};
                    pq.push({newCost, nr, nc});
                }
            }
        }
    }

    vector<pair<int, int>> path;
    minCost = dist[end.first][end.second];
    if (minCost == 1e9)
        return path;

    pair<int, int> curr = end;
    while (curr != make_pair(-1, -1) && curr != start)
    {
        path.push_back(curr);
        curr = prev[curr.first][curr.second];
    }
    if (curr == start)
        path.push_back(start);
    reverse(path.begin(), path.end());
    return path;
}

// ================= 4. 可视化（仅修改图片弹出的等待逻辑） =================
void visualize(const vector<vector<int>> &grid, pair<int, int> start, pair<int, int> end, const vector<pair<int, int>> &path, int minCost, const string &filename)
{
    int rows = grid.size(), cols = grid[0].size();
    int cellSize = 80, margin = 40;

    Mat img(rows * cellSize + margin * 2, cols * cellSize + margin * 2, CV_8UC3, Scalar(255, 255, 255));

    for (int r = 0; r < rows; ++r)
    {
        for (int c = 0; c < cols; ++c)
        {
            int val = grid[r][c];
            Scalar color;
            if (val == 0)
                color = Scalar(60, 60, 60); // 障碍物
            else if (val == 1)
                color = Scalar(110, 180, 110); // 代价1
            else if (val == 3)
                color = Scalar(40, 160, 230); // 代价3
            else if (val == 8)
                color = Scalar(60, 70, 200); // 代价8
            else
                color = Scalar(200, 200, 200);

            int x1 = margin + c * cellSize, y1 = margin + r * cellSize;
            int x2 = x1 + cellSize, y2 = y1 + cellSize;

            rectangle(img, Point(x1, y1), Point(x2, y2), color, FILLED);
            rectangle(img, Point(x1, y1), Point(x2, y2), Scalar(255, 255, 255), 2);

            if (val != 0)
                putText(img, to_string(val), Point(x1 + 10, y1 + 25), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 255, 255), 1);
        }
    }

    for (size_t i = 0; i < path.size() - 1; ++i)
    {
        Point p1(margin + path[i].second * cellSize + cellSize / 2, margin + path[i].first * cellSize + cellSize / 2);
        Point p2(margin + path[i + 1].second * cellSize + cellSize / 2, margin + path[i + 1].first * cellSize + cellSize / 2);
        line(img, p1, p2, Scalar(255, 105, 180), 8);
    }

    Point sPt(margin + start.second * cellSize + cellSize / 2, margin + start.first * cellSize + cellSize / 2);
    Point ePt(margin + end.second * cellSize + cellSize / 2, margin + end.first * cellSize + cellSize / 2);

    circle(img, sPt, 20, Scalar(255, 255, 255), FILLED);
    putText(img, "S", Point(sPt.x - 10, sPt.y + 10), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 0, 0), 2);

    circle(img, ePt, 20, Scalar(255, 255, 255), FILLED);
    putText(img, "E", Point(ePt.x - 10, ePt.y + 10), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 0, 0), 2);

    putText(img, "Total Cost: " + to_string(minCost), Point(margin, margin - 10), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 0, 0), 2);

    imwrite(filename, img);
    imshow("Result", img);

    cout << "【提示】请在结果图片窗口上按 [ESC] 键关闭并继续。" << endl;
    while (true)
    {
        if (waitKey(10) == 27)
            break; // 按 ESC 关闭结果图
    }
    destroyWindow("Result");
}

// ================= 5. 导出 JSON（保持原样） =================
void exportJSON(const vector<pair<int, int>> &path, int minCost, const string &filename)
{
    ofstream out(filename);
    out << "{\n  \"total_cost\": " << minCost << ",\n  \"path\": [\n";
    for (size_t i = 0; i < path.size(); ++i)
    {
        out << "    [" << path[i].first << ", " << path[i].second << "]";
        if (i < path.size() - 1)
            out << ",";
        out << "\n";
    }
    out << "  ]\n}\n";
    out.close();
    cout << "JSON 结果已保存至: " << filename << endl;
}

// ================= 6. 主函数（加入 ESC 免回车退出） =================
int main()
{
    cout << "===========================================" << endl;
    cout << "  福州大学竞技机器人 - 最短路径识别系统" << endl;
    cout << "===========================================" << endl;

    while (true)
    {
        cout << "\n【待命状态】按 [回车键] 启动摄像头扫码 (或直接按 [ESC] 退出程序)..." << endl;

        // 替换原来的 getline，改为无阻塞监听
        while (true)
        {
            if (kbhit())
            {
                char c = getchar();
                if (c == 27)
                { // ESC
                    cout << "程序已安全退出。" << endl;
                    return 0;
                }
                else if (c == '\n' || c == '\r')
                { // 回车
                    break;
                }
            }
            usleep(50000); // 降低 CPU 占用
        }

        cout << "\n--- 启动摄像头扫码 ---" << endl;
        string qrData = scanQRCode();

        if (qrData.empty())
        {
            cerr << "未获取到二维码数据（或按了ESC取消），请重新尝试..." << endl;
            continue;
        }

        cout << "成功识别二维码: " << qrData << endl;

        int rows, cols;
        pair<int, int> start, end;
        vector<vector<int>> grid;

        if (!parseQRData(qrData, rows, cols, start, end, grid))
        {
            cerr << "解析失败！数据格式错误，请重新扫码..." << endl;
            continue;
        }

        cout << "--- 开始 Dijkstra 寻路 ---" << endl;
        int minCost = 0;
        vector<pair<int, int>> path = dijkstra(grid, start, end, minCost);

        if (path.empty())
        {
            cout << "未找到可行路径！请检查地图数据。" << endl;
            continue;
        }

        cout << "寻路成功！最小总代价: " << minCost << endl;

        visualize(grid, start, end, path, minCost, "result.png");
        exportJSON(path, minCost, "result.json");

        cout << "裁判确认完毕后，准备识别下一个二维码。" << endl;
    }

    return 0;
}