# ARMOR_AUTO_AIM
作者:五邑大学imca战队算法组谢炜坡。
本项目为装甲板预测部分。项目基于ubuntu平台使用VScode开发,主要用于Robomaster比赛中测试各机动目标模型对装甲板预测的效果以及相关参数调试。
### 功能实现
* 基于特征匹配的装甲板识别功能
* 基于Singer模型和匀加速模型的装甲板X轴像素坐标卡尔曼预测
* 实时显示预测坐标
* 保存预测参数并图线绘制
### 依赖环境
ros2 opencv c++
### 问题
当前问题:预测存在延迟 
        rqt显示图像卡顿 用cv显示没问题
### 使用
在终端中输入
colcon build
source install/setup.bash 
ros2 run image_publish camera 
ros2 run armor_dectetor armor_node 
ros2 run kalman_predict predict 
