    #include<rclcpp/rclcpp.hpp> //ros2

    #include <pcl/filters/passthrough.h>
    #include<pcl_conversions/pcl_conversions.h>
    #include<pcl/io/ply_io.h>//读取文件
    #include <pcl/filters/extract_indices.h>
    #include <pcl/kdtree/kdtree_flann.h>
    #include<pcl/exceptions.h>
    #include<visualization_msgs/msg/marker_array.hpp>
    #include <sensor_msgs/point_cloud2_iterator.hpp>
    #include<tf2_geometry_msgs/tf2_geometry_msgs.hpp>
    #include<chrono>

    using namespace std::chrono_literals;

    class PointDelete:public rclcpp::Node{
    public:
        PointDelete():Node("point_delete"){
            msg_=std::make_shared<sensor_msgs::msg::PointCloud2>();
            cloud_=std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
            init_cloud();

            publisher_marker=this->create_publisher<visualization_msgs::msg::MarkerArray>("/visualization_markers",10000);
            publisher_point=this->create_publisher<sensor_msgs::msg::PointCloud2>("/pointcloud",10000);
            point_to_marker();

            subscription_=this->create_subscription<geometry_msgs::msg::PointStamped>("/clicked_point",10,
                std::bind(&PointDelete::handle_click,this,std::placeholders::_1));
        }
        void init_cloud(){
            this->declare_parameter<float>("delete_radius", 1.0f);// 默认删除半径1米
            this->declare_parameter<int>("min_points_per_grid", 3);// 生成立方体的最小点数
            this->declare_parameter<float>("grid_size", 1.0f);// 网格边长（单位：米）
            this->declare_parameter<float>("gap_ratio", 0.0f);// 间隙占总网格尺寸的比例
            this->declare_parameter<std::string>("save_filename", "");
            this->declare_parameter<std::string>("source_cloud", "");
            this->declare_parameter<std::string>("frame_id", "");

            delete_radius= this->get_parameter("delete_radius").as_double();
            min_points_per_grid = this->get_parameter("min_points_per_grid").as_int();
            grid_size = this->get_parameter("grid_size").as_double();
            gap_ratio = this->get_parameter("gap_ratio").as_double();
            frame_id = this->get_parameter("frame_id").as_string();
            save_filename = this->get_parameter("save_filename").as_string();
            source_cloud = this->get_parameter("source_cloud").as_string();

            pcl::PCDReader reader;
            reader.read(source_cloud,*cloud_);
            std::cout<<"点云个数："<<cloud_->size()<<std::endl;
            //过滤
            // pcl::PassThrough<pcl::PointXYZ> pass;
            // pass.setInputCloud(cloud_);
            // pass.setFilterFieldName("z");
            // pass.setFilterLimits(-5, 3);
            // pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            // pass.filter(*cloud_);
            // std::cout<<cloud_->size()<<std::endl;
            //放大
            // auto &points=cloud_->points;
            // for(auto &point:points){
            //     point.x*=40;
            //     point.y*=40;
            //     point.z*=40;
            // }
            // pcl::toROSMsg(*cloud_,*msg_);
            // msg_->header.frame_id="base_link_1";
            // msg_->header.stamp=this->now();
        }
       
        
        void point_to_marker() {
            // 创建网格容器（三维哈希表）
            std::unordered_map<int, 
                std::unordered_map<int, 
                    std::unordered_map<int, 
                        std::vector<geometry_msgs::msg::Point>>>> grid_map;
        
            // 遍历点云数据并填充网格
            for (const auto& point : cloud_->points) {
                const int grid_x = static_cast<int>(std::floor(point.x / grid_size));
                const int grid_y = static_cast<int>(std::floor(point.y / grid_size));
                const int grid_z = static_cast<int>(std::floor(point.z / grid_size));
        
                geometry_msgs::msg::Point p;
                p.x = point.x;
                p.y = point.y;
                p.z = point.z;
                grid_map[grid_x][grid_y][grid_z].push_back(p);
            }
           
            // 查找最大点密度
            if(if_first_max){
                if_first_max=false;
                max_density = 0;
                for (const auto& x_entry : grid_map) {
                    for (const auto& y_entry : x_entry.second) {
                        for (const auto& z_entry : y_entry.second) {
                            max_density = std::max(max_density, z_entry.second.size());
                        }
                    }
                }
            }
            
            // 清空原有MarkerArray
            marker_array.markers.clear();
            // 添加清除指令,!!!必须要添加，否则因为marker的生命周期问题，rviz中的正方体不会及时相应
            visualization_msgs::msg::Marker clear_marker;
            clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
            clear_marker.ns = "grid_markers";
            marker_array.markers.push_back(clear_marker);
        
            // 遍历所有网格
            int marker_id = 0;
            for (const auto& x_entry : grid_map) {
                for (const auto& y_entry : x_entry.second) {
                    for (const auto& z_entry : y_entry.second) {
                        const auto& points = z_entry.second;
        
                        // 筛选满足最小点数的网格
                        if (points.size() < min_points_per_grid) continue;
        
                        // 计算网格中心坐标
                        geometry_msgs::msg::Point center;
                        for (const auto& p : points) {
                            center.x += p.x;
                            center.y += p.y;
                            center.z += p.z;
                        }
                        center.x /= points.size();
                        center.y /= points.size();
                        center.z /= points.size();
        
                        // 创建网格Marker
                        visualization_msgs::msg::Marker grid_marker;
                        grid_marker.header.frame_id =frame_id;
                        grid_marker.header.stamp = this->now(); 
                        grid_marker.ns = "grid_markers";
                        grid_marker.id = marker_id++;
                        grid_marker.type = visualization_msgs::msg::Marker::CUBE;
                        grid_marker.action = visualization_msgs::msg::Marker::ADD;
                        
                        // 设置立方体属性
                        
                        grid_marker.scale.x = grid_size * (1.0-gap_ratio); // 留10%间隙
                        grid_marker.scale.y = grid_size * (1.0-gap_ratio);
                        grid_marker.scale.z = grid_size * (1.0-gap_ratio);
                        // 计算颜色渐变（白 -> 蓝）
                        const float density_ratio = static_cast<float>(points.size()) / max_density;
                        grid_marker.color.r = 1.0f - density_ratio * 0.8f;
                        grid_marker.color.g = 1.0f - density_ratio * 0.8f; 
                        grid_marker.color.b = 1.0f - density_ratio * 0.2f;
                        grid_marker.color.a =1.0f; // 半透明效果,;
                        grid_marker.pose.position = center;
                        grid_marker.pose.orientation.w = 1.0;
                        
                        // 加入Marker数组
                        marker_array.markers.push_back(grid_marker);
                    }
                }
            }
            pcl::toROSMsg(*cloud_,*msg_);
            msg_->header.frame_id=frame_id;
            msg_->header.stamp=this->now();
            // 发布MarkerArray
            if(if_first_publish){
                if_first_publish=false;
                publisher_marker->publish(marker_array);
                publisher_point->publish(*msg_);
            }
           
            publisher_marker->publish(marker_array);
            
            publisher_point->publish(*msg_);
            RCLCPP_INFO(this->get_logger(), "Generated %d grid markers", marker_id);
        }

        void handle_click(const geometry_msgs::msg::PointStamped& click_point) {
            std::lock_guard<std::mutex> lock(markers_mutex_);
            // 获取最新参数
            delete_radius = this->get_parameter("delete_radius").as_double();
            pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
            kdtree.setInputCloud(cloud_);
            // 搜索半径内的点
            pcl::PointXYZ search_point;
            search_point.x = click_point.point.x;
            search_point.y = click_point.point.y;
            search_point.z = click_point.point.z;
            std::vector<int> point_indices;
            std::vector<float> distances;
            kdtree.radiusSearch(search_point, delete_radius, point_indices, distances);
             // 提取要删除的点
            pcl::PointIndices::Ptr indices_to_remove(new pcl::PointIndices);
            indices_to_remove->indices = point_indices;

            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(cloud_);
            extract.setIndices(indices_to_remove);
            extract.setNegative(true);  // 保留未删除的点

            pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            extract.filter(*filtered_cloud);
            cloud_ = filtered_cloud;
            // 更新ROS消息并重新生成网格
            point_to_marker();
            publisher_marker->publish(marker_array);
            RCLCPP_INFO(this->get_logger(), "Deleted %zu points", point_indices.size());
            save_pcd();
        }
        void save_pcd(){
            pcl::io::savePCDFileASCII(save_filename, *cloud_);
            RCLCPP_INFO(this->get_logger(), "Saved filtered point cloud to %s", save_filename.c_str());
        }

    private:
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscription_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_marker;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_point;
        
        sensor_msgs::msg::PointCloud2::SharedPtr msg_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
        visualization_msgs::msg::MarkerArray marker_array;
        visualization_msgs::msg::Marker marker;
        size_t max_density = 0;bool if_first_max=true;bool if_first_publish=true;

        std::mutex markers_mutex_;
        double delete_radius ;  
        std::string save_filename,source_cloud,frame_id;
        float grid_size ;
        int min_points_per_grid ; 
        float gap_ratio= 0.0f;  // 间隙占总网格尺寸的比例
        
    };


    int main(int argc,char **argv){
        rclcpp::init(argc,argv);
        auto node=std::make_shared<PointDelete>();
        rclcpp::spin(node);
        rclcpp::shutdown();
        return 0;
    }