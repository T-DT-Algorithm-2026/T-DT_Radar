import rclpy
from rclpy.node import Node
from vision_interface.msg import MatchInfo, Radar2Sentry 
import ollama
import re

from rclpy.qos import qos_profile_sensor_data

class MatchInfoSubscriber(Node):
    def __init__(self):
        super().__init__('match_info_subscriber')
        self.subscription = self.create_subscription(
            MatchInfo,
            'match_info',
            self.listener_callback,
            10
        )
        
        self.enemy_hp = []
        self.self_hp = []
            
        self.subscription2 = self.create_subscription(
            Radar2Sentry,
            'Radar2Sentry',
            self.listener_callback2,
            qos_profile_sensor_data
        )
        
        self.enemy_pos = []
        
        self.messages = [{'role': 'system', 'content': '全国大学生机器人大赛 RoboMaster 机甲大师超级对抗赛（RMUC, RoboMaster University Championship），侧重考察参赛队员对理工学科的综合应用与工程实践能力，充分融合了“机器视觉”、“嵌入式系统设计”、“机械工程”、“自主导航”、“人机交互”等众多机器人相关技术学科，同时创新性地将流行呈现方式与机器人竞技相结合，使机器人对抗更加直观激烈，吸引了众多的科技爱好者和社会公众的广泛关注。规则概述在2025赛季中，对战双方需自主研发不同种类和功能的机器人，在指定的比赛场地内进行战术对抗，通过操控机器人发射弹丸攻击对方机器人和基地。比赛结束时，基地剩余血量高的一方获得比赛胜利。你是一个强大的具有大局观的哨兵操作ai, 你的任务是保护我方基地，消灭敌方机器人。你需要根据敌方机器人的位置和血量，来决定你的行动。你的回答不应该包括其他自然语言,需要回答一个json,包括30字以内的局势分析以及哨兵目标的二维坐标。回答例如:{"target": [5.23, 5.14], "analysis": "敌方机器人在我方基地附近, 血量较低, 我方机器人在进攻,我应该拦截敌方机器人"},如果某位置是0,则视为该机器人位置没有被发现,不要视为真实位置,前后两次给出的目标点和分析都不应该完全相同,你给出的目标点也不应该是0'}]
        
        
    def listener_callback(self, msg):
        
        self.enemy_hp = msg.robot_hp[:8]
        self.self_hp = msg.robot_hp[8:]
        
        message = f'敌方1号英雄机位置: {self.enemy_pos[0]}, 血量: {self.enemy_hp[0]}; 敌方2号工程机位置: {self.enemy_pos[1]}, 血量: {self.enemy_hp[1]}; 敌方3号步兵位置: {self.enemy_pos[2]}, 血量: {self.enemy_hp[2]}; 敌方4号步兵位置: {self.enemy_pos[3]}, 血量: {self.enemy_hp[3]}; 敌方5号步兵位置: {self.enemy_pos[4]}, 血量: {self.enemy_hp[4]}; 敌方6号哨兵位置: {self.enemy_pos[5]}, 血量: {self.enemy_hp[5]}; 我方哨兵血量: {self.self_hp[5]},如果某位置是0,则视为该机器人位置没有被发现,不要视为真实位置,前后两次给出的目标点和分析都不应该完全相同'
        self.messages.append({'role': 'user', 'content': message})
        response = ollama.chat(
            model="deepseek-r1:8b",
            messages=self.messages
        )
        
        self.messages.append(response.message)
        
        content = response.message['content']
        
        target_match = re.search(r'"target": \[(.*?)\]', content)
        analysis_match = re.search(r'"analysis": "(.*?)"', content)

        if target_match and analysis_match:
            target = target_match.group(0)
            analysis = analysis_match.group(0)
            
            print(target)
            print(analysis)
        else:
            print("解析失败")
        
        
        
    def listener_callback2(self, msg):
        self.enemy_pos = list(zip(msg.radar_enemy_x, msg.radar_enemy_y))
        
        


def main(args=None):
    rclpy.init(args=args)
    node = MatchInfoSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
