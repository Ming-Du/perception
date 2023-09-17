#include "denoise_feature_extractor.h"
namespace robosense {
void DenoiseFeatureExtractor::init() {
    //
}

bool print_nearby_numbers(double num,double int_near) {
    int int_part = static_cast<int>(num);
    double decimal_part = fabs(num - static_cast<double>(int_part));
    if (decimal_part <= int_near) {
        // std::cout << " "<<num<<" " << std::endl;
        return true;
    }
    return false;
}
void print_invalid_points(std::vector<double> &px,std::vector<double> &py){
    if(px.size()>0 && px.size()==py.size()){
        std::cout<<"invalid points deleted:";
        for(size_t i=0;i<px.size();i++){
            std::cout<<" ["<<px.at(i)<<","<<py.at(i)<<"] ";
        }
        std::cout<<std::endl;
    }
}
int DenoiseFeatureExtractor::prepare_xybin_grid(std::vector<int> &point_valid_num_inxybin,int &point_num,std::vector<int> &points_idx,double xy_sizeORI){
//     const auto &cloud_ptr = msg_ptr_->scan_ptr;
//     // double zmin = 0.2;
//     // double zmax = 1.4;
//     double x_min = floor(cloud_ptr->points[points_idx[0]].x);
//     double y_min = floor(cloud_ptr->points[points_idx[0]].y);
//     double x_range = 1.0;//1.0m
//     double y_range = 1.0;//1.0m
//     int xsize = 10;
//     int ysize = 10;
//     // int xy_size = xsize*ysize;
//     double unit_size_x = x_range/xsize;
//     double unit_size_y = y_range/ysize;
//     for (auto &pi : points_idx) {
//         const auto &p = cloud_ptr->points[pi];
//         // if(p.z<zmin || p.z>zmax){continue;}
//         point_num++;
//         int x_id = (p.x-x_min)/unit_size_x;
//         int y_id = (p.y-y_min)/unit_size_y;
//         int grid_idx = x_id*ysize+y_id;
//         point_valid_num_inxybin[grid_idx]+=1;
//     }
//     int valid_grid_num = 0;
//     for(auto pi:point_valid_num_inxybin){
//         if(pi>0){valid_grid_num++;}
//     }
//     return valid_grid_num;
// }

// int DenoiseFeatureExtractor::prepare_xybin(std::vector<int> &point_valid_num_inxybin,int &point_num,std::vector<int> &points_idx){
    const auto &cloud_ptr = msg_ptr_->scan_ptr;
    // double zmin = 0.2;
    // double zmax = 1.4;
    
    double x_min = floor(cloud_ptr->points[points_idx[0]].x);
    double y_min = floor(cloud_ptr->points[points_idx[0]].y);
    // bool is_bug = false;
    // for (auto &pi : points_idx){
    //     double px = cloud_ptr->points[pi].x;
    //     double py = cloud_ptr->points[pi].y;
    //     //TODO:
    //     if(!print_nearby_numbers(px,0.00001) && !print_nearby_numbers(py,0.00001)){
    //         x_min = floor(px);
    //         y_min = floor(py);
    //         is_bug = false;
    //         break;
    //     }
    // }
    // if(is_bug){
    //     std::cout<<"allpoints_invalid_del_frame"<<std::endl;
    //     return -1;
    // }

    double x_range = 1.0;//1.0m
    double y_range = 1.0;//1.0m
    int xsize = 10;
    int ysize = 10;
    // int xy_size = xsize*ysize;
    double unit_size_x = x_range/xsize;
    double unit_size_y = y_range/ysize;
    std::vector<double> px;
    std::vector<double> py;
    for (auto &pi : points_idx) {
        const auto &p = cloud_ptr->points[pi];
        // if(p.z<zmin || p.z>zmax){continue;}
        point_num++;
        int x_id = (p.x-x_min)/unit_size_x;
        int y_id = (p.y-y_min)/unit_size_y;
        int grid_idx = x_id*ysize+y_id;
        if(grid_idx<0 || grid_idx>=100){
            // px.push_back(p.x);
            // py.push_back(p.y);
            continue;
        }
        point_valid_num_inxybin[grid_idx]+=1;
    }
    // if(is_bug){print_invalid_points(px,py);}
    int valid_grid_num = 0;
    for(auto pi:point_valid_num_inxybin){
        if(pi>0){valid_grid_num++;}
    }
    return valid_grid_num;
}

void DenoiseFeatureExtractor::getZRangeIdx( std::vector<int> &points_idx, std::vector<int> &points_zrange_idx) {
    const auto &cloud_ptr = msg_ptr_->scan_ptr;
    double zmin = 0.4;//20230106_210549 0.2;
    double zmax = 1.4;
    for (auto pid : points_idx) {
        const auto &p = cloud_ptr->points[pid];
        if(p.z<zmin || p.z>zmax){
            continue;
        }
        points_zrange_idx.push_back(pid);
    }
}

bool is_equal(double a,double b){
    if(fabs(a-b)<0.0000001){
        return true;
    }else{
        return false;
    }
}
bool DenoiseFeatureExtractor::getFeatures_grid(const LidarFrameMsg::Ptr &msg_ptr, std::vector<int> &points_idx, std::vector<double> &feature,double xy_sizeORI) {
    // std::cout<<H_DEBUG_R<<"yinggai you~~~"<<std::endl;
    msg_ptr_ = msg_ptr;
    feature_ = &feature;
    // points_idx = &points_idx;
    std::vector<int> points_zrange_idx;
    getZRangeIdx(points_idx, points_zrange_idx);
    if(points_zrange_idx.size()<1){
        return false;
    }
    // std::cout<<H_DEBUG_R<<"yinggai you~~~"<<std::endl;
    int xy_size = 100;
    std::vector<int> point_valid_num_inxybin(xy_size,0);
    // point_valid_num_inxybin.reserve((int)xy_size*2);
    int point_num = 0;
    int valid_grid_num = prepare_xybin_grid(point_valid_num_inxybin,point_num, points_zrange_idx,xy_size);
    if(valid_grid_num==-1){
        //all points in edge near
        return false;
    }
    if(valid_grid_num<5){
        return false;
    }
    xy_bin_histogram_pareto(point_valid_num_inxybin, point_num, points_zrange_idx);//1,2,3
    intensity_std(points_zrange_idx);//4
    z_bin_ratio_std(points_idx);//5
    xy_bin_valid_ratio(point_valid_num_inxybin, xy_size,5,points_zrange_idx);//6
    double area = 1.0;
    points_num(points_zrange_idx,area);//7
    // z_range(points_idx);//8 error not same as Wiki and Haiko's s5
    z_range(points_zrange_idx);//8
    intensity_mean(points_zrange_idx);//9
    // bool is_debug = false;
    // if(is_debug){
    //     // debug z_range feature is not same between createFeature(c++) and s5feature.py(python).
    //     //     c++   :  /home/mogo/data/humengmeng/z_git/bus_lidar/xgboost/createFeatureGrid_cpp/build/createFeature
    //     //     python:  /home/mogo/data/humengmeng/z_git/bus_lidar/xgboost/Phase2_Obj/debug_s5/s5feature.py
    //     //     data  :  /home/mogo/data/humengmeng/z_git/bus_lidar/xgboost/Phase2_Obj/r1_points_grid/grid__Bus水花case__RBS-1096_0__pos.txt
    //     //        1675666865.201170 4410 13.0 0.0 13.4424 0.326336 0.323252 34.0 13.4241 0.29964 0.322725 35.0 13.4216 0.2716 0.321415 33.0 13.391 0.244754 0.321434 29.0 13.3804 0.21763 0.320538 33.0 13.3499 0.192534 0.320646 69.0 13.3311 0.166365 0.320165 70.0 13.3162 0.138529 0.319431 69.0 13.2973 0.112626 0.318971 70.0 13.2902 0.0854507 0.317912 69.0 13.2789 0.0574156 0.317 70.0 13.2677 0.0308481 0.31616 72.0 13.2485 0.00527549 0.315731 71.0 13.7543 0.34101 0.463074 39.0 13.6803 0.317741 0.463979 28.0 13.646 0.291459 0.463634 68.0 13.2086 0.0238595 0.462684 145.0 13.1378 0.00500008 0.463735 123.0 13.8966 0.179943 0.606923 36.0 13.8858 0.151204 0.605673 32.0 13.7834 0.131775 0.605814 39.0 13.669 0.114023 0.606161 31.0 13.6301 0.087846 0.605334 34.0 13.6031 0.0618652 0.604396 33.0 13.0148 0.413526 0.310576 65.0 13.0767 0.034568 0.305721 64.0 13.131 0.0191461 0.30262 65.0 13.1575 0.000752175 0.301058 86.0 13.0431 0.33403 0.450553 34.0 13.0554 0.308617 0.449925 37.0 13.0597 0.281883 0.449593 27.0 13.0284 0.249607 0.450593 27.0
    //     bool is_it = true;
    //     double feature_tocheck[8]={0.66666666666666663,1 ,1 ,0.136951048665008596 ,0.355246779504645949 ,0 ,15 ,0.305864989757537842};
    //     for(int i=0;i<7;i++){
    //         if(!is_equal((*feature_).at(i),feature_tocheck[i])){
    //             is_it = false;
    //             break;
    //         }
    //     }
    //     if(is_it){
    //         std::cout<<"FOUND feature:"<<(*feature_).at(7)<<std::endl;
    //         std::cout<<"FOUND      z:";
    //         const auto &cloud_ptr = msg_ptr_->scan_ptr;
    //         for (auto &pid : points_idx) {
    //             const auto &p = cloud_ptr->points[pid];
    //             std::cout<<p.z<<",";
    //         }
    //         std::cout<<std::endl;
    //         std::cout<<std::endl;
    //     }
    // }
    return true;
}
void DenoiseFeatureExtractor::z_range( std::vector<int> &points_idx) {
    const auto &cloud_ptr = msg_ptr_->scan_ptr;
    // double zmin = 0.2;
    // double zmax = 1.4;
    // double unit_size_z=0.2;
    double z_range_min = 999;
    double z_range_max = -999;
    for (auto &pid : points_idx) {
        const auto &p = cloud_ptr->points[pid];
        // if(p.z<zmin || p.z>zmax){continue;}
        if(p.z<z_range_min){z_range_min=p.z;}
        if(p.z>z_range_max){z_range_max=p.z;}
    }
    (*feature_).push_back(z_range_max-z_range_min);
}

void DenoiseFeatureExtractor::points_num( std::vector<int> &points_idx, double area) {
    double num = points_idx.size();
    num = num/area;
    (*feature_).push_back(num);
}
void DenoiseFeatureExtractor::z_bin_ratio_std( std::vector<int> &points_idx) {
    const auto &cloud_ptr = msg_ptr_->scan_ptr;
    double zmin = 0.4;//20230106_210549 0.2;
    double zmax = 2.2;
    // double x_min = floor(cloud_ptr->points[points_idx[0]].x);
    // double y_min = floor(cloud_ptr->points[points_idx[0]].y);
    // int zsize = 5;
    // double unit_size_z=(zmax-zmin)/zsize;
    double unit_size_z=0.2;
    int zsize = std::round((zmax-zmin)/unit_size_z);
    std::vector<double> point_valid_num_inzbin(zsize,0.0);
    for (auto &pid : points_idx) {
        const auto &p = cloud_ptr->points[pid];
        if(p.z<zmin || p.z>zmax){continue;}
        int z_id = (p.z-zmin)/unit_size_z;
        point_valid_num_inzbin[z_id]++;
    }
    double pointnum_max = 0.1;
    for(auto pi:point_valid_num_inzbin){
        if(pi>pointnum_max){
            pointnum_max=pi;
        }
    }
    for(auto &pi:point_valid_num_inzbin){
        pi = pi/pointnum_max;
    }
    double std = stdev(point_valid_num_inzbin);
    (*feature_).push_back(std);
}

void DenoiseFeatureExtractor::xy_bin_valid_ratio(std::vector<int> &point_valid_num_inxybin,double xy_size, int valid_num, std::vector<int> &points_idx) {
    double valid_cnt = 0.0;
    for(auto pi:point_valid_num_inxybin){
        if(pi>=valid_num){
            valid_cnt++;
        }
    }
    double ratio = valid_cnt/xy_size;
    (*feature_).push_back(ratio);
}
double DenoiseFeatureExtractor::stdev( std::vector<double> &data) {
    double sum = 0.0, mean, standardDeviation = 0.0;
    int i;
    int n = data.size();
    for(i = 0; i < n; ++i) {
      sum += data[i];
    }
    mean = sum / n;
    for(i = 0; i < n; ++i) {
      standardDeviation += pow(data[i] - mean, 2);
    }
    return sqrt(standardDeviation / n);
}
double DenoiseFeatureExtractor::mean( std::vector<double> &data) {
    double sum = 0.0, mean = 0.0;
    int i;
    int n = data.size();
    for(i = 0; i < n; ++i) {
      sum += data[i];
    }
    mean = sum / n;
    return mean;
}
// void DenoiseFeatureExtractor::connected_val( std::vector<int> &points_idx) {
//     const auto &cloud_ptr = msg_ptr_->scan_ptr;
//     // #include <pcl/point_cloud.h>
//     // #include <pcl/point_types.h>
//     // #include <pcl/segmentation/region_growing.h>

//     // int main()
//     // {
//     //     pcl::PointCloud<pcl::PointXYZI>::Ptr pc(new pcl::PointCloud<pcl::PointXYZI>);
//     //     // 从文件中读取点云数据
//     //     pcl::io::loadPCDFile("point_cloud.pcd", *pc);

//     //     // 定义区域生长分割器
//     //     pcl::RegionGrowing<pcl::PointXYZI, pcl::Normal> reg;
//     //     reg.setInputCloud(pc);
//     //     // 设置法线估计器
//     //     pcl::search::Search<pcl::PointXYZI>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZI> >(new pcl::search::KdTree<pcl::PointXYZI>);
//     //     pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
//     //     pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normal_estimator;
//     //     normal_estimator.setSearchMethod(tree);
//     //     normal_estimator.setInputCloud(pc);
//     //     normal_estimator.setKSearch(10);
//     //     normal_estimator.compute(*normals);
//     //     reg.setInputNormals(normals);
//     //     // 设置区域生长参数
//     //     reg.setMinClusterSize(50);
//     //     reg.setMaxClusterSize(1000000);
//     //     reg.setSearchMethod(tree);
//     //     reg.setNumberOfNeighbours(30);
//     //     reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
//     //     reg.setCurvatureThreshold(1.0);
//     //     // 执行区域生长分割
//     //     std::vector<pcl::PointIndices> clusters;
//     //     reg.extract(clusters);

//     //     // 输出连通区域数量
//     //     std::cout << "Number of clusters: " << clusters.size() << std::endl;

//     //     return 0;
//     // }



//     (*feature_).push_back(std_ev);
// }
void DenoiseFeatureExtractor::intensity_std( std::vector<int> &points_idx) {
    const auto &cloud_ptr = msg_ptr_->scan_ptr;
    std::vector<double> intensity_vec;
    intensity_vec.reserve(points_idx.size());
    for (auto &pi : points_idx) {
        const auto &p = cloud_ptr->points[pi];
        intensity_vec.push_back(p.intensity/255.0);
    }
    double std_ev = stdev(intensity_vec);
    (*feature_).push_back(std_ev);
}
void DenoiseFeatureExtractor::intensity_mean( std::vector<int> &points_idx) {
    const auto &cloud_ptr = msg_ptr_->scan_ptr;
    std::vector<double> intensity_vec;
    intensity_vec.reserve(points_idx.size());
    for (auto &pi : points_idx) {
        const auto &p = cloud_ptr->points[pi];
        intensity_vec.push_back(p.intensity/255.0);
    }
    double mean_val = mean(intensity_vec);
    (*feature_).push_back(mean_val);
}
std::vector<double>  DenoiseFeatureExtractor::calcPareto( std::vector<int> &data,int totalnum) {
    sort(data.begin(),data.end(),[](int x,int y){return x>y;});
    std::vector<double> percent{0.05, 0.10, 0.20};
    int n = data.size();
    double accumulate_num = 0;
    int j=0;
    std::vector<double> pareto_vec;
    pareto_vec.reserve(n);
    for(int i=0;i<n;i++){
        accumulate_num+=data.at(i);
        if(i+1>percent[j]*n){
            double per = accumulate_num/totalnum;
            pareto_vec.push_back(per);
            j++;
        }
        if(j>=percent.size()){
            break;
        }
    }
    return pareto_vec;
}
void DenoiseFeatureExtractor::xy_bin_histogram_pareto( std::vector<int> &point_valid_num_inxybin,int point_num, std::vector<int> &points_idx) {
    // const auto &cloud_ptr = msg_ptr_->scan_ptr;
    // double zmin = 0.2;
    // double zmax = 2.5;
    // double x_min = floor(cloud_ptr->points[points_idx[0]].x);
    // double y_min = floor(cloud_ptr->points[points_idx[0]].y);
    // double x_range = 1.0;//1.0m
    // double y_range = 1.0;//1.0m
    // int xsize = 10;
    // int ysize = 10;
    // double unit_size_x = x_range/xsize;
    // double unit_size_y = y_range/ysize;
    // std::vector<int> point_valid_num_inxybin(xsize*ysize,0);
    // int point_num = 0;
    // for (auto &pi : points_idx) {
    //     const auto &p = cloud_ptr->points[pi];
    //     if(p.z>zmin && p.z<zmax){
    //         point_num++;
    //         int x_id = (p.x-x_min)/unit_size_x;
    //         int y_id = (p.y-y_min)/unit_size_y;
    //         int grid_idx = x_id*ysize+y_id;
    //         point_valid_num_inxybin[grid_idx]+=1;
    //     }
    // }
    std::vector<double> pareto_vec = calcPareto(point_valid_num_inxybin,point_num);
    for(auto pi:pareto_vec){
        (*feature_).push_back(pi);
    }  
}

// void DenoiseFeatureExtractor::feature_height_max( std::vector<int> &points_idx) {
//     const auto &cloud_ptr = msg_ptr_->scan_ptr;
//     double z_max = cloud_ptr->points[points_idx[0]].z;
//     for (auto &pi : points_idx) {
//         const auto &p = cloud_ptr->points[pi];
//         if (p.z > z_max) {
//             z_max = p.z;
//         }
//     }
//     (*feature_).push_back(z_max);
// }
// void DenoiseFeatureExtractor::feature_height_min( std::vector<int> &points_idx) {
//     const auto &cloud_ptr = msg_ptr_->scan_ptr;
//     double z_min = cloud_ptr->points[points_idx[0]].z;
//     for (auto &pi : points_idx) {
//         const auto &p = cloud_ptr->points[pi];
//         if (p.z < z_min) {
//             z_min = p.z;
//         }
//     }
//     (*feature_).push_back(z_min);
// }
int DenoiseFeatureExtractor::prepare_xybin_obj(std::vector<int> &point_valid_num_inxybin,int &point_num,std::vector<int> &points_idx,double xy_size){
    const auto &cloud_ptr = msg_ptr_->scan_ptr;
    std::map<int,int> point_valid_num_inxybin_map;
    // double zmin = 0.2;
    // double zmax = 1.4;
    
    double x_min = floor(cloud_ptr->points[points_idx[0]].x);
    double y_min = floor(cloud_ptr->points[points_idx[0]].y);
    double x_range = 1.0;//1.0m
    double y_range = 1.0;//1.0m
    int xsize = 10;
    int ysize = 10;
    // int xy_size = xsize*ysize;
    double unit_size_x = x_range/xsize;
    double unit_size_y = y_range/ysize;
    for (auto &pi : points_idx) {
        const auto &p = cloud_ptr->points[pi];
        // if(p.z<zmin || p.z>zmax){continue;}
        point_num++;
        int x_id = (p.x-x_min)/unit_size_x;
        int y_id = (p.y-y_min)/unit_size_y;
        int grid_idx = x_id*ysize+y_id;
        if (point_valid_num_inxybin_map.find(grid_idx) == point_valid_num_inxybin_map.end()) {
            point_valid_num_inxybin_map.emplace(grid_idx,0);
        }else{
            point_valid_num_inxybin_map[grid_idx]+=1;
        }
    }
    int valid_grid_num = point_valid_num_inxybin_map.size();
    point_valid_num_inxybin.reserve(valid_grid_num);
    for (auto it = point_valid_num_inxybin_map.begin();it != point_valid_num_inxybin_map.end();it++){
        point_valid_num_inxybin.push_back(it->second);
    }
    // int xybin_num = (int)xy_size;//after add 0 for blank grid, feature weight lower.
    // for(size_t i=valid_grid_num;i<xybin_num;i++){
    //     point_valid_num_inxybin.push_back(0);
    // }
    return valid_grid_num;
}
bool DenoiseFeatureExtractor::getFeatures_obj(const LidarFrameMsg::Ptr &msg_ptr, std::vector<int> &points_idx, std::vector<double> &feature,double xy_size) {
    msg_ptr_ = msg_ptr;
    feature_ = &feature;
    // points_idx = &points_idx;
    std::vector<int> points_zrange_idx;
    getZRangeIdx(points_idx, points_zrange_idx);
    if(points_zrange_idx.size()<1){
        return false;
    }
    // int xy_size;// = 100;
    std::vector<int> point_valid_num_inxybin;//(xy_size,0);
    point_valid_num_inxybin.reserve((int)xy_size*2);
    int point_num = 0;
    
    int valid_grid_num = prepare_xybin_obj(point_valid_num_inxybin,point_num, points_zrange_idx,xy_size);
    if(valid_grid_num<5){
        return false;
    }
    xy_bin_histogram_pareto(point_valid_num_inxybin, point_num, points_zrange_idx);//1,2,3
    intensity_std(points_zrange_idx);//4
    z_bin_ratio_std(points_idx);  // 5
    xy_bin_valid_ratio(point_valid_num_inxybin, xy_size,5,points_zrange_idx);//6
    double area = xy_size/100.0;
    points_num(points_zrange_idx, area);//7
    z_range(points_zrange_idx);//8
    intensity_mean(points_zrange_idx);//9
    return true;
}
} // namespace robosense