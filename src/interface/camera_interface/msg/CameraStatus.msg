std_msgs/Header header

string camera_name    # 相机名称
bool is_running       # 当前运行状态
float32 current_fps   # 当前帧率
string message        # 额外状态信息 (如 "OK", "Error", "Overheating" 等)