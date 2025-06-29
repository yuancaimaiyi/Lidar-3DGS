# Lidar-3DGS

激光相机设备 Gaussian Splatting，SfM 的结果会有噪点，或多或少会对 3DGS 的结果产生影响，那么在不考虑设备成本的情况下，激光点云初始化 3DGS 是一个不错的选择。

## 设备概况

![image](https://github.com/user-attachments/assets/4c0b9b36-7bc0-4b8e-8729-bee1252e1f34)

设备组合由 Mid360 + 消费级全景组合，秉着不需要时间同步的出发点，彻彻底底地降低硬件研发的成本；如果不考虑成本，那么工业相机 + Mid360 更好。

## 设备处理

假设采用 Mid360 + 消费级全景组合，算法处理后，可以输出彩色点云，那么可以得到以下信息：

1. 彩色点云 PLY 文件
2. 所有相机的 Pose 信息
3. 相机的内参
4. 每个点由哪些图像贡献，类似 COLMAP 的 `fused.ply.vis` 文件（如果你对 COLMAP 代码熟悉，你就知道我在说什么）
5. 曲线救国方法- 由于大多数 Gaussian Splatting 的代码输入都是 COLMAP Sparse Model 的格式，那么最简单最直接的方法就是将上述的 (1)、(2)、(3)、(4) 转成 `images.bin`、`points3D.bin`、`cameras.bin`。这个转换是非常容易实现的，那么为何转，有两个原因：其一是不改 GS 的代码，其二是转换后的数据可以用 COLMAP GUI 查看，可以直观查看转换的效果和点云的效果。

`cameras.bin/txt` 如下：

![image](https://github.com/user-attachments/assets/ca5d3ed2-bb4a-4770-9c27-7746c80c5e21)

`images.bin/txt` 如下：

![image](https://github.com/user-attachments/assets/c5646078-3da8-492b-8bb6-312711c6af21)

`images.txt` 只有 Pose 信息。

`points3d.bin/txt` 利用 (1) 和 (4) 这样实现：

```
const auto& ply_points = ReadPly(JoinPaths(path, "points.ply"));
std::ofstream file("/path/points3d.txt", std::ios::trunc);
CHECK(file.is_open()) << path;

const std::string vis_path = JoinPaths(path, "points.ply.vis");
std::fstream vis_file(vis_path, std::ios::in | std::ios::binary);
CHECK(vis_file.is_open()) << vis_path;

const size_t vis_num_points = ReadBinaryLittleEndian<uint64_t>(&vis_file);
CHECK_EQ(vis_num_points, ply_points.size());
points.reserve(ply_points.size());
for (const auto& ply_point : ply_points) {
  const int point_idx = points.size();
  file << point_idx << " ";
  file << ply_point.x << " ";
  file << ply_point.y << " ";
  file << ply_point.z << " ";
  file << ply_point.r << " ";
  file << ply_point.g << " ";
  file << ply_point.b << " ";
  file << 0.0 << " ";
  std::ostringstream line;
  input_point.num_visible_images =
      ReadBinaryLittleEndian<uint32_t>(&vis_file);
  for (uint32_t i = 0; i < input_point.num_visible_images; ++i) {
    const int image_idx = ReadBinaryLittleEndian<uint32_t>(&vis_file);
    line << image_idx << " ";
    line << 0 << " "; // line << track_el.point2D_idx << " ";
  }
  std::string line_string = line.str();
  line_string = line_string.substr(0, line_string.size() - 1);
  file << line_string << std::endl;
}
```
6. 当机立断方法 - 要知道gaussian splatting 输入的本质是what ？ 只需要激光点云及其对于的图像视角信息（image_id 、image pose)即可 ,那么就可以采用下面这个方法    
    i. 对points.ply 进行投影到2D ，自动分块得到box及block_points   
    ii.根据points.ply.vis 拿到 每个block_points对应的图像id   
    iii.得到若干个block_init.ply 和 一个配置所有block的文件  
    iv.对大场景的激光点云进行分块的gs ，因为激光点云的密度是非常大的，直接训练的显存和内存是无法承受的  
## GS 训练


