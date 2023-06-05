#include <string>
#include <vector>

#define PI 3.1415926

template <class T>
T cropNum(T x, T low, T up){
    return std::max(low, std::min(up, x));
}

class MapImageTrans
{
public:

    int _h, _w;
    std::string _fn;
    float _rel;
    std::vector<float> _ori;
    float _cth, _sth;

public:    
    void setYaml(std::string yn);
    std::vector<float> imageToMap(const std::vector<float>& src);
    std::vector<float> mapToImage(const std::vector<float>& src);

    template <class T>
    static T toRad(T r){
        // crop to -pi~pi
        T res = r / 180.0 * PI;
        while(res > PI)   res -= 2*PI;
        while(res < -PI)  res += 2*PI;
        return res;
    }

    template <class T>
    static T toAng(T a){
        // crop to 0~360
        T res = a * 180.0 / PI;
        while(res < 0)      res += 360.0;
        while(res > 360)   res -= 360.0;
        return res;
        
    }
    
};