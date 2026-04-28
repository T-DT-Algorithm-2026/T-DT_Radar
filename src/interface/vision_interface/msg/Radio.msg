uint16[6] hp
uint16[5] fire

uint16 economy
uint16 uneconomy

bool supply_zone            # 对方补给区占领状态 (0:未占领, 1:已占领)
uint8 center_highland       # 对方中央高地的占领状态 (0:未占领, 1:对方, 2:己方)
bool trapezoid_highland     # 对方梯形高地的占领状态 (0:未占领, 1:已占领)
uint8 fortress              # 对方堡垒增益点的占领状态 (0:未, 1:对方, 2:己方, 3:双方)
uint8 outpost               # 对方前哨站增益点的占领状态 (0:未, 1:对方, 2:己方)
bool base                   # 对方基地增益点的占领状态 (0:未占领, 1:已占领)

bool tunnel_enemy_front     # 飞坡前隧道(靠近对方)检测到模块
bool tunnel_enemy_back      # 飞坡后隧道(靠近对方)检测到模块
bool tunnel_self_front      # 飞坡前隧道(靠近己方)检测到模块
bool tunnel_self_back       # 飞坡后隧道(靠近己方)检测到模块

bool module_highland        # 高地上部检测到模块
bool module_fly_ramp_back   # 飞坡后部检测到模块
bool module_highway_upper   # 公路上部检测到模块

uint8[5] heal
uint16[5] cooldown
uint8[5] defence
uint8[5] undefence
uint16[5] attack            #增益

uint8[6] key


