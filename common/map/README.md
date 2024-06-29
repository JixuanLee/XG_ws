### Map folder

put your map file here (.pcd).

PCD地图的每个数据点的坐标系（原点）是[录制该地图时激光雷达的初始位姿]，而[map]是人为设定的永恒不变。
因此，导入PCD地图时，需要将  [录制该PCD时的激光雷达初始位姿]  对齐到  [map]，这一步在[map_loader.launch]中已经完成。