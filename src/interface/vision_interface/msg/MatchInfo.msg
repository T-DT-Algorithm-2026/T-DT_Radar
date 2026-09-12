int8 self_color #0是蓝 2是红
int16 match_time # 比赛时间，若尚未开始则发送比赛开始倒计时时间的负数，若比赛结束发送-100 若未连接至裁判系统发送-200
uint8[16] robot_hp #参考0x0003
uint8[6] marks
uint8 ultimate
uint32 eventtype
uint8 large_energy_status # 己方大能量机关状态：0 未激活，1 已激活，2 正在激活
uint8 dart_hit_target # 最近一次己方飞镖击中的目标
uint8 dart_selected_target # 飞镖当前选定要击打的目标
uint8 sentry_go_kill
# bit 12~15：对方被瞄准、对方被反制、己方被瞄准、己方被反制
bool[4] mark_progress
