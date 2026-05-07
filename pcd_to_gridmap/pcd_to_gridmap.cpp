#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <chrono>
#include <sys/stat.h>
#include <unistd.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>

typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> PointCloud;

// 输出使用帮助
void print_help(const char* prog_name) {
    std::cout << "用法: " << prog_name << " <输入.pcd> [选项]\n"
              << "将三维点云投影到 XOZ 平面，生成 2D 栅格地图。\n\n"
              << "选项:\n"
              << "  -r <float>     栅格分辨率 (米/像素, 默认: 0.05)\n"
              << "  -m <1|2|3>     地面检测方法: 1=RANSAC, 2=MLESAC, 3=MSAC (默认: 1)\n"
              << "  -g <float>      地面检测距离阈值 (米, 默认: 0.05)\n"
              << "  -min <float>    保留点的最小相对高度 (米, 默认: 0.2)\n"
              << "  -max <float>    保留点的最大相对高度 (米, 默认: 2.0)\n"
              << "  -o <float>      障碍物判定阈值比例 [0,1] (默认: 0.2)\n"
              << "  -h, --help      显示本帮助\n\n"
              << "输出文件: gridmap.pgm, gridmap.yaml (保存在当前目录)\n";
}

// 地面平面检测 
void planar(const PointCloud::Ptr& cloud,
                   int m,
                   double g,
                   Eigen::Vector4d& coefficients) {
    // 降采样加速
    pcl::VoxelGrid<PointType> voxel_filter;
    PointCloud::Ptr fastcloud(new PointCloud);
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(0.1f, 0.1f, 0.1f);
    voxel_filter.filter(*fastcloud);
    // 创建分割对象
    pcl::SACSegmentation<PointType> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);  // 指定平面模型
    switch(m) {
        case 1:
            seg.setMethodType(pcl::SAC_RANSAC);
            std::cout << "  使用 RANSAC 方法" << std::endl;
            break;
        case 2:
            seg.setMethodType(pcl::SAC_MLESAC);
            std::cout << "  使用 MLESAC 方法" << std::endl;
            break;
        case 3:
            seg.setMethodType(pcl::SAC_MSAC);
            std::cout << "  使用 MSAC 方法" << std::endl;
            break;
    }
    seg.setDistanceThreshold(g);  // 设置点到平面的距离阈值
    seg.setMaxIterations(500);  // 设置最大迭代次数
    seg.setProbability(0.99);  // 设置置信度
    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    seg.setInputCloud(fastcloud);
    seg.segment(*inliers, *coeff);  // 执行分割，将内点索引存入 inliers，平面系数（a,b,c,d）存入 coefficients
    coefficients << coeff->values[0], coeff->values[1], coeff->values[2], coeff->values[3];  // 提取系数
}

// 计算点到平面的有向距离
double h(const PointType& point,
                     const Eigen::Vector4d& plane) {
    double a = plane[0], b = plane[1], c = plane[2], d = plane[3];
    double m = std::sqrt(a*a + b*b + c*c);
    if (m < 1e-6){
        return 0.0;  // 防止除数过小
    }
    return (a*point.x + b*point.y + c*point.z + d) / m;
}

int main(int argc, char** argv) {
    // 解析命令行参数
    if (argc < 2) {
        print_help(argv[0]);
        return -1;
    }

    std::string input_pcd;
    double r = 0.05;       // 栅格分辨率 (m/pixel)
    int m = 1;             // 地面检测方法
    double g = 0.05;       // 地面检测距离阈值
    double hMin = 0.2;    // 最小相对高度
    double hMax = 2.0;    // 最大相对高度
    double o = 0.2;        // 障碍物阈值比例

    int arg_idx = 1;
    // 检查第一个参数是否为帮助
    std::string first_arg = argv[1];
    if (first_arg == "-h" || first_arg == "--help") {
        print_help(argv[0]);
        return 0;
    }
    // 第一个参数是输入 PCD 文件
    input_pcd = argv[1];
    arg_idx = 2;

    // 解析可选参数
    for (int i = arg_idx; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "-r" && i + 1 < argc) {
            r = atof(argv[++i]);
        } else if (arg == "-m" && i + 1 < argc) {
            m = atoi(argv[++i]);
        } else if (arg == "-g" && i + 1 < argc) {
            g = atof(argv[++i]);
        } else if (arg == "-min" && i + 1 < argc) {
            hMin = atof(argv[++i]);
        } else if (arg == "-max" && i + 1 < argc) {
            hMax = atof(argv[++i]);
        } else if (arg == "-o" && i + 1 < argc) {
            o = atof(argv[++i]);
        } else {
            std::cerr << "警告: 未知参数 " << arg << std::endl;
        }
    }
    
    // 参数有效性检查
    if (r <= 0.0) {
        std::cerr << "错误：分辨率必须大于0" << std::endl;
        return -1;
    }
    if (o < 0.0 || o > 1.0) {
        std::cerr << "错误：障碍物比例必须在 [0,1] 之间" << std::endl;
        return -1;
    }
    if (g <= 0.0) {
        std::cerr << "错误：地面检测距离阈值必须大于0" << std::endl;
        return -1;
    }
    if (hMin >= hMax) {
        std::cerr << "错误：最小高度必须小于最大高度" << std::endl;
        return -1;
    }
    if (m < 1 || m > 3) {
        std::cerr << "错误: -s 必须为 1,2 或 3\n";
        return -1;
    }
    
    auto start = std::chrono::high_resolution_clock::now();

    std::cout << "\n========== [1/4] 加载点云 ==========" << std::endl;
    PointCloud::Ptr cloud(new PointCloud);
    if (pcl::io::loadPCDFile<PointType>(input_pcd, *cloud) == -1) {
        std::cerr << "错误: 无法读取 PCD 文件 " << input_pcd << std::endl;
        return -1;
    }
    std::cout << "  读取点云: " << cloud->size() << " 个点" << std::endl;

    std::cout << "\n========== [2/4] 地面检测与高度过滤 ==========" << std::endl;
    Eigen::Vector4d ground;
    planar(cloud, m, g, ground);
    std::cout << "  平面方程: " << ground[0] << "x + " << ground[1] << "y + "
              << ground[2] << "z + " << ground[3] << " = 0" << std::endl;

    std::vector<PointType> filter;
    filter.reserve(cloud->size());
    double min_x = std::numeric_limits<double>::max();
    double max_x = -std::numeric_limits<double>::max();
    double min_z = std::numeric_limits<double>::max();
    double max_z = -std::numeric_limits<double>::max();

    for (const auto& point : cloud->points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) continue;
        double height = h(point, ground);
        if (height >= hMin && height <= hMax) {
            filter.push_back(point);
            if (point.x < min_x) min_x = point.x;
            if (point.x > max_x) max_x = point.x;
            if (point.z < min_z) min_z = point.z;
            if (point.z > max_z) max_z = point.z;
        }
    }
    std::cout << "  相对高度范围: [" << hMin << ", " << hMax << "] 米\n";
    std::cout << "  过滤后点数: " << filter.size() << " / " << cloud->size() << std::endl;

    // 边界裕量
    double margin = r * 2.0;
    double min_x_margin = min_x - margin;
    double max_x_margin = max_x + margin;
    double min_z_margin = min_z - margin;
    double max_z_margin = max_z + margin;

    if (max_x_margin <= min_x_margin || max_z_margin <= min_z_margin) {
        std::cerr << "错误: 无效边界，可能缺少有效点\n";
        return -1;
    }

    std::cout << "\n========== [3/4] 创建栅格地图 ==========" << std::endl;
    int W = static_cast<int>((max_x_margin - min_x_margin) / r) + 1;
    int H = static_cast<int>((max_z_margin - min_z_margin) / r) + 1;

    size_t total_cells = W * H;
    std::vector<int> hit_count(total_cells, 0);
    std::vector<uint8_t> grid(total_cells, 254); // 自由区域

    std::cout << "  障碍物判定阈值比例: " << o << std::endl;
    for (const auto& point : filter) {
        int x = static_cast<int>((point.x - min_x_margin) / r);
        int z = static_cast<int>((point.z - min_z_margin) / r);
        if (x >= 0 && x < W && z >= 0 && z < H) {
            hit_count[z * W + x]++;
        }
    }

    int maxH = 0;
    for (int i = 0; i < W * H; ++i) {
        if (hit_count[i] > maxH) maxH = hit_count[i];
    }
    std::cout << "  最大击中次数: " << maxH << std::endl;

    double T = o * maxH;
    int occupied_cells = 0;
    for (int i = 0; i < W * H; ++i) {
        if (hit_count[i] > T) {
            grid[i] = 0;   // 障碍物
            occupied_cells++;
        } else if (hit_count[i] > 0) {
            grid[i] = 127; // 未知
        }
    }

    std::cout << "\n========== [4/4] 保存地图文件 ==========" << std::endl;

    // 保存 PGM
    std::string pgm_file = "gridmap.pgm";
    std::ofstream pgm(pgm_file, std::ios::binary);
    if (!pgm) {
        std::cerr << "错误: 无法创建 PGM 文件 " << pgm_file << std::endl;
        return -1;
    }
    pgm << "P5\n";
    pgm << "# Generated by PCDToGridMap\n";
    pgm << "# Projection Plane: XOZ (X-Z plane)\n";
    pgm << "# Origin X (min_x): " << min_x_margin << "\n";
    pgm << "# Origin Z (min_z): " << min_z_margin << "\n";
    pgm << "# Resolution: " << r << " m/pixel\n";
    pgm << "# Occupancy: 0=obstacle, 127=unknown, 254=free\n";
    pgm << W << " " << H << "\n";
    pgm << "255\n";

    for (int y = H - 1; y >= 0; --y) {
        for (int x = 0; x < W; ++x) {
            uint8_t pixel = grid[y * W + x];
            pgm.write(reinterpret_cast<const char*>(&pixel), 1);
        }
    }
    pgm.close();
    std::cout << "  保存栅格图: " << pgm_file << std::endl;

    // 保存 YAML
    std::string yaml_file = "gridmap.yaml";
    std::ofstream yaml(yaml_file);
    if (!yaml) {
        std::cerr << "错误: 无法创建 YAML 文件 " << yaml_file << std::endl;
        return -1;
    }
    yaml << "image: gridmap.pgm\n";
    yaml << "resolution: " << r << "\n";
    yaml << "origin: [" << min_x_margin << ", " << min_z_margin << ", 0.0]\n";
    yaml << "occupied_thresh: 0.65\n";
    yaml << "free_thresh: 0.196\n";
    yaml << "# Projection Plane: XOZ (X-Z plane)\n";
    yaml << "# Map dimensions: " << W << " x " << H << " pixels\n";
    yaml << "# Physical dimensions: " << (W * r) << " x " << (H * r) << " meters\n";
    yaml << "# Occupancy values: 0=obstacle, 127=unknown, 254=free\n";
    yaml.close();
    std::cout << "  保存 YAML: " << yaml_file << std::endl;

    auto end = std::chrono::high_resolution_clock::now();
    auto time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    std::cout << "\n========== 栅格地图信息 ==========" << std::endl;
    std::cout << "  地图尺寸: " << W << " x " << H << " 像素" << std::endl;
    std::cout << "  分辨率: " << r << " m/px" << std::endl;
    std::cout << "  占用栅格数: " << occupied_cells << std::endl;
    std::cout << "  耗时: " << time << " 毫秒" << std::endl;

    return 0;
}
