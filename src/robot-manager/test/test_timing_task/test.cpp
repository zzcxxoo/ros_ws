#include <iostream>
#include <fstream>
#include <unordered_map>
#include "nlohmann/json.hpp"
#include <ctime>

using json = nlohmann::json;
using namespace std;

std::pair<int, int> fmtHourAndMin(std::string str){
    auto res = std::make_pair(-1, -1); 

    int pos = str.find(':');
    int sz = str.size();

    if(pos != str.npos){
        res.first = std::stoi(str.substr(0, pos));
        res.second = std::stoi(str.substr(pos+1, sz-pos-1));
    }

    return res;   
}

void mapTojson()
{
    unordered_map<std::string, json> map;

    json j = R"(
        {
            "name": "haha",
            "age": 50
        }
    )"_json;

    map["5"] = j;
    j["name"] = "xixi";
    j["age"] = 30;
    map["6"] = j;

    ofstream out("/home/hgy/9tian_ws/src/robot-manager/test/test_timing_task/test.json");
    json jj = map;
    cout << jj << endl;
    out << jj;
    out.close();
}

void jsonToMap()
{
    ifstream inf("/home/hgy/9tian_ws/src/robot-manager/test/test_timing_task/test.json");
    json j = json::parse(inf);
    unordered_map<std::string, json> map = j;

    decltype(map)::const_iterator it;
    for(it=map.begin(); it!=map.end(); it++){
        cout << it->first << endl;
        cout << it->second << endl;
    }
    inf.close();
}

void test_fmt_hour_and_min()
{
    string str = "19:23";
    
    auto r = fmtHourAndMin(str);
    cout << r.first << endl;
    cout << r.second << endl;
}

void fmtTime(){
    auto now = std::time(0);
    tm *ltm = localtime(&now);
    char cr[64];
    std::sprintf(cr, "%04d-%02d-%02d&%02d-%02d-%02d", 1900 + ltm->tm_year, ltm->tm_mon, 1 + ltm->tm_mday, ltm->tm_hour, ltm->tm_min, ltm->tm_sec);
    cout << std::string(cr) << endl;
}

std::string genUUID() noexcept
{
    auto now = std::time(0);
    tm *ltm = localtime(&now);
    char cr[64];
    std::sprintf(cr, "%02d-%02d-%02d&%02d-%02d-%02d", ltm->tm_year - 100, 1 + ltm->tm_mon,
                ltm->tm_mday, ltm->tm_hour, ltm->tm_min, ltm->tm_sec);
    return std::string(cr);
}

void test_save_to_json()
{
    json j;
    std::ofstream out("/home/hgy/9tian_ws/src/robot-manager/test/test_timing_task/test.json");
    out << j;
    out.close();

    std::ifstream inf("/home/hgy/9tian_ws/src/robot-manager/test/test_timing_task/test.json");
    j = json::parse(inf);
    cout << j.is_null() << endl;
}

int main(int argc, char const *argv[])
{
    // test_fmt_hour_and_min();
    // mapTojson();
    // jsonToMap();
    // fmtTime();
    cout << genUUID() << endl;

    // test_save_to_json();

    return 0;
}
