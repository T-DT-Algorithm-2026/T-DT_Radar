## 1、配置
databag_tool 目录下的配置文件 config_record.yaml

其中参数：
1. 设置需要录制的话题: ``xxxx_record ``
```
localization_record:
  - /livox/lidar
  - /livox/imu
  - /ll/gps

navigation_record:
  - /genshin/way

```
这意味着将开启两个数据集writer，分别储存于数据集文件夹下的不同子文件夹，也就是说这两个 section分开录制，其中localization_record录制话题包括``"/livox/lidar" "/livox/imu" "/ll/gps" ``。书写要符合格式，section的名字以 ``_record`` 结尾，话题格式如例

2. 设置数据集储存母文件夹: ``bags_relative_path``
设置相对于yaml文件所在文件夹的储存位置：``最终path = "...src/databag_tool" + bags_relative_path , 用于 record 和 merge``

3. 设置录制强制停止时间:``forced_stop_time``
强制停止时间，单次开机自启或启动后的停止时间

## 2、录制运行

可执行节点为 databag_tool包下的 record 
```bash
ros2 run databag_tool record
```

## 3、merge操作
在实际运行情况中，如果是赛场上的直接关机，当前写入的数据集文件一定是损坏的，为了处理这种情况，设置为每一段时间自动保存一个bag，所以需要将多个bag进行merge操作       
可执行节点为 databag_tool包下的 merge       
```bash
ros2 run databag_tool merge_bags
```         
执行，将检查数据集储存母文件夹下所有可merge子子数据集的子数据集文件夹，未merge的将自动merge，如果包含损坏的bag，那么运行过程终端虽然提示报错，但并不会将报错的bag写入merged_bag，仍然会得出可用的merged_bag数据集


