#ifndef TDT_CONFIG_H
#define TDT_CONFIG_H

#include "base_msg.h"

#include <yaml-cpp/yaml.h>
class tdtconfig {
public:
    static void Init();

public:
    //! 以下变量的名字全部是大写，这样一看就知道是宏
    // 按照run_config.yaml里面的顺序定义，这样就很容易修改了
    static int CAMERA;
    static int MARKER;
    static int CALIBRATE;
    static int RECORDER;
    static int LOG;
    static int INDEX;
    static int BACKTREACK;
    static int USART;
    static int MANUAL;
    static bool ifInit ;
    // static bool O3ENABLE;
private:
    static void Read(const YAML::Node &root, const std::string& node, int &param);
};

#endif // TDT_CONFIG_H