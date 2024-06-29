# 功能包介绍

## 1.代码介绍
本功能包主要功能是实现对给定的路径进行跟随，实现自动从A点到B点的效果。订阅/path_reference话题，获得要跟随的路径，然后生成对应的torque和steer并以话题/cmd_raw发布。

过程中用到的话题可参考[topiclist.md](topiclist.md)