#include "config.h"
#include <string>
#include <fstream>

// 默认全为false
int tdtconfig::CAMERA;
int tdtconfig::MARKER;
int tdtconfig::CALIBRATE;
int tdtconfig::RECORDER;
int tdtconfig::LOG;
int tdtconfig::MANUAL;
int tdtconfig::INDEX;
int tdtconfig::BACKTREACK;
int tdtconfig::USART;
bool tdtconfig::ifInit;
// bool tdtconfig::O3ENABLE=false;

void tdtconfig::Init() {
    std::ifstream file("./config/run_config.yaml");
    if(!file.is_open()) {
      TDT_ERROR("run_config.yaml不存在或者打不开");
    }
    YAML::Node root = YAML::Load(file);
    Read(root, "CAMERA", CAMERA);
    Read(root, "MARKER", MARKER);
    //    Read(fs, "CALIBRATE", CALIBRATE);      //calibrate参数只有标定中用到，不用从文件中读取
    Read(root, "RECORDER", RECORDER);
    Read(root, "LOG", LOG);
    Read(root, "INDEX", INDEX);
    Read(root, "BACKTREACK", BACKTREACK);
    Read(root, "USART", USART);
    Read(root, "MANUAL", MANUAL);
    Read(root, "MANUAL", MANUAL);
    file.close();

    ifInit = true ;
    // Read(fs,"O3ENABLE",O3ENABLE);
}

void tdtconfig::Read(const YAML::Node &root, const std::string& node, int &param) {
    if (root[node].IsNull()) {
        TDT_ERROR("run_config中不存在 %s ", node.c_str());
    } else {
        param = root[node].as<int>();
    }
}