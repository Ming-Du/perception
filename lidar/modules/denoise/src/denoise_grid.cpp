#include "denoise_grid.h"

namespace robosense {
void DenoiseGrid::init(const Rs_YAMLReader &configParse) {
    params_ = configParse.getValue<DenoiseParam>("denoise");
    denoise_enable_ = params_.enable;
    if(!denoise_enable_){
        return;
    }
    std::string xgb_model_path_ = configParse.getBasePath() + "/model/xgboost/" + params_.model;
    feature_extractor_ptr_.reset(new DenoiseFeatureExtractor);
    feature_extractor_ptr_->init();
    ROS_INFO_STREAM("xgb_model_path_:"<<xgb_model_path_);
    // ROS_INFO_STREAM("xgb_obj_model_path_:"<<xgb_obj_model_path_);
    xgb_ptr_.reset(new Xgb);
    // xgb_obj_ptr_.reset(new Xgb);
    int silent = 0; //whether print messages during loading
    xgb_ptr_->init(xgb_model_path_, silent);
    // xgb_obj_ptr_->init(xgb_obj_model_path_, silent);
}

bool DenoiseGrid::isInRange(double x,double y,double z){
    return (x>params_.range.xmin && x<params_.range.xmax &&
            y>params_.range.ymin && y<params_.range.ymax && 
            z>params_.range.zmin && z<params_.range.zmax  );
}
void DenoiseGrid::try_grid_data(LidarFrameMsg::Ptr &msg_ptr) {
    auto &cloud_ptr = msg_ptr->scan_ptr;
    auto &scan_valid_indice = msg_ptr->valid_indices;
    //const auto &points = cloud_ptr->points; // each point struct
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZI>());
    // points.clear();
    scan_valid_indice.clear();
    //test case 1
    int data_num = 114;float dummy_data[data_num][4] = {{8.37346,3.70432,0.346578,16.},{8.37321,3.68556,0.348191,181.},{8.98361,3.29501,0.351047,33.},{8.93946,3.25202,0.356916,31.},{8.93706,3.23225,0.358721,34.},{8.95851,3.22685,0.358112,36.},{8.879,3.1651,0.367343,49.},{8.89369,3.15526,0.367449,11.},{8.93594,3.16135,0.364822,36.},{8.93976,3.14625,0.365919,14.},{8.97861,3.15005,0.363657,60.},{8.47171,3.29831,0.449535,10.},{8.4865,3.29047,0.449606,37.},{8.41057,3.22444,0.45727,22.},{8.63533,3.06465,0.461579,26.},{8.7331,3.10271,0.455281,46.},{8.80856,3.30595,0.515255,41.},{8.77896,3.27006,0.51825,42.},{8.78316,3.25536,0.519153,38.},{8.83203,3.26705,0.517402,44.},{8.8018,3.23192,0.520363,46.},{8.77509,3.19856,0.523136,33.},{8.77557,3.18193,0.52425,30.},{8.75176,3.15134,0.526784,30.},{8.75932,3.13821,0.527526,10.},{8.7874,3.13739,0.527037,42.},{8.78413,3.11886,0.528359,11.},{8.80562,3.11327,0.528325,10.},{8.84424,3.11786,0.52727,10.},{8.75667,3.05334,0.533352,35.},{8.7531,3.03495,0.534677,28.},{8.76026,3.02183,0.535439,11.},{8.78118,3.01675,0.53539,21.},{8.80569,3.01332,0.535161,26.},{8.86587,3.02782,0.533028,28.},{8.9117,3.03514,0.53166,26.},{8.96478,3.04574,0.529933,13.},{8.98621,3.03915,0.529987,12.},{8.97169,3.01498,0.53193,63.},{8.61855,3.61881,0.576831,22.},{8.63382,3.61135,0.577126,13.},{8.65896,3.61061,0.576987,19.},{8.6943,3.61615,0.576435,21.},{8.77827,3.65665,0.573631,15.},{8.90214,3.72371,0.569097,22.},{8.75139,3.34393,0.591331,22.},{8.73204,3.31456,0.593117,26.},{8.71945,3.28939,0.594629,26.},{8.68305,3.24941,0.597119,32.},{8.6598,3.21848,0.599023,40.},{8.53705,3.12796,0.604871,48.},{8.56853,3.12994,0.604604,63.},{8.54814,3.10174,0.606343,60.},{8.53795,3.07975,0.607675,50.},{8.53465,3.06174,0.608744,40.},{8.528,3.04143,0.609966,37.},{8.57302,3.05148,0.609178,41.},{8.57304,3.03543,0.610125,38.},{8.92828,3.70425,0.653409,15.},{8.93414,3.68868,0.654164,161.},{8.5575,3.22904,0.745763,34.},{8.55473,3.21084,0.746402,37.},{8.35094,3.66522,0.801016,15.},{8.34112,3.63896,0.801224,15.},{8.35897,3.54389,0.803819,16.},{8.3773,3.5395,0.804539,19.},{8.43777,3.47754,0.808067,12.},{8.45604,3.47266,0.808832,19.},{8.43814,3.44265,0.808923,31.},{8.4137,3.40794,0.8089,29.},{8.38915,3.37331,0.808876,31.},{8.38718,3.35516,0.809261,26.},{8.39502,3.34385,0.809835,26.},{8.38976,3.32325,0.810173,16.},{8.42092,3.32728,0.811217,19.},{8.43861,3.32255,0.811998,12.},{8.37666,3.26418,0.811242,12.},{8.49444,3.32547,0.814006,14.},{8.39526,3.24345,0.812502,19.},{8.39954,3.22968,0.813043,23.},{8.42733,3.23103,0.814063,14.},{8.51865,3.19189,0.818727,19.},{8.57059,3.2068,0.820327,16.},{8.46177,3.1242,0.818513,19.},{8.56814,3.17257,0.821267,13.},{8.56189,3.15203,0.821649,16.},{8.59651,3.15656,0.822897,13.},{8.78654,3.06622,0.833728,19.},{8.77955,3.04559,0.834154,10.},{8.77226,3.02544,0.834556,26.},{8.75795,3.00133,0.834805,26.},{8.04091,3.45497,0.863501,20.},{8.04304,3.43954,0.863731,17.},{8.1358,3.49635,0.867486,17.},{8.12522,3.47106,0.867232,22.},{8.13678,3.46282,0.867848,24.},{8.14519,3.45205,0.868347,24.},{8.14065,3.43179,0.868341,25.},{8.16163,3.43075,0.869343,24.},{8.25293,3.38218,0.874319,20.},{8.25781,3.36867,0.874737,11.},{8.27866,3.36712,0.875789,23.},{8.28672,3.35591,0.876343,10.},{8.29495,3.34436,0.876918,10.},{8.4566,3.04385,0.89105,20.},{8.43595,3.0159,0.890546,23.},{8.20588,3.48039,0.230668,19.},{8.04715,3.34192,0.257828,22.},{8.0523,3.32877,0.258833,18.},{8.05392,3.31363,0.260324,22.},{8.70863,3.34129,0.283442,36.},{8.57147,3.23823,0.302088,41.},{8.61325,3.24658,0.29859,48.},{8.59717,3.21981,0.302166,47.}};
    //test case 2
    // int data_num = 38;float dummy_data[data_num][4] = {{0.686804,3.44889,0.751764,21.},{0.639827,3.4954,0.750141,16.},{0.575915,3.55245,0.748463,173.},{0.902791,3.13069,0.927759,35.},{0.987269,3.10232,0.922879,43.},{0.99779,3.11353,0.921257,40.},{0.882238,3.19402,0.925068,37.},{0.812922,3.25064,0.926783,31.},{0.788997,3.28186,0.926577,31.},{0.807055,3.28977,0.924581,42.},{0.797354,3.31345,0.923747,48.},{0.959555,3.02335,1.00879,37.},{0.439562,3.68524,1.0191,16.},{0.465539,3.68981,1.01636,14.},{0.553281,3.32086,1.11702,18.},{0.546923,3.34303,1.11656,17.},{0.51264,3.38055,1.11844,13.},{0.485548,3.41448,1.1197,11.},{0.497145,3.42758,1.11773,19.},{0.616901,3.48002,1.27736,14.},{0.624629,3.49367,1.27581,14.},{0.626238,3.51254,1.27496,15.},{0.620824,3.53504,1.275,14.},{0.635992,3.54534,1.27252,18.},{0.823435,3.01276,1.34596,27.},{0.837471,3.02367,1.34335,23.},{0.843754,3.03664,1.34193,21.},{0.843549,3.05429,1.34141,14.},{0.850284,3.06796,1.33991,12.},{0.37653,3.0448,0.451777,23.},{0.339945,3.07986,0.447248,28.},{0.763722,3.03964,0.457579,23.},{0.831441,3.02325,0.460365,26.},{0.884386,3.03046,0.460194,29.},{0.827707,3.92532,0.350264,18.},{0.844447,3.93474,0.349187,18.},{0.848445,3.95364,0.346815,18.},{0.867676,3.95952,0.346195,21.}};
    for(int i=0;i<data_num;i++){
        float x = dummy_data[i][0];
        float y = dummy_data[i][1];
        float z = dummy_data[i][2];
        float ist = dummy_data[i][3];
        pcl::PointXYZI p;
        p.x  = x ;
        p.y  = y ;
        p.z  = z ;
        p.intensity = ist;
        pointcloud->push_back(p);
        // pointcloud->push_back(pcl::PointXYZI(x,y,z,ist));
        scan_valid_indice.push_back(i);
    }
    cloud_ptr = pointcloud;
}
void DenoiseGrid::createGridPointsMap(const LidarFrameMsg::Ptr &msg_ptr) {
    const auto &cloud_ptr = msg_ptr->scan_ptr;
    // const auto &scan_valid_indice = msg_ptr->valid_indices;
    const auto &scan_valid_indice = msg_ptr->roifiltered_indices;
    rowcol_ptidxs_map_.clear();
    rowcol_vec_.clear();
    for (size_t i = 0; i < scan_valid_indice.size(); ++i) {
        const auto &pt_idx = scan_valid_indice[i];      // each point index
        const auto &tmp_pt = cloud_ptr->points[pt_idx]; // each point struct
        if (std::isnan(tmp_pt.x) || std::isnan(tmp_pt.y) || std::isnan(tmp_pt.z)) {
            continue;
        }
        
        if(!isInRange(tmp_pt.x,tmp_pt.y,tmp_pt.z)){
            continue;
        }
        int x_id = static_cast<int>((tmp_pt.x - params_.range.xmin) / params_.unit_size);
        int y_id = static_cast<int>((tmp_pt.y - params_.range.ymin) / params_.unit_size);
        int grid_id = x_id * params_.ysize + y_id;
        if (rowcol_ptidxs_map_.find(grid_id) == rowcol_ptidxs_map_.end()) {
            std::vector<int> ptidxs(1, 0);
            ptidxs[0] = pt_idx;
            rowcol_ptidxs_map_[grid_id] = ptidxs;
            rowcol_vec_.push_back(grid_id);
        } else {
            rowcol_ptidxs_map_[grid_id].emplace_back(pt_idx);
        }
    }
}
void DenoiseGrid::perception(const LidarFrameMsg::Ptr &msg_ptr) {
    // LidarFrameMsg::Ptr msg_ptr(new LidarFrameMsg);
    // try_grid_data(msg_ptr);
    // H: range
    TRY_CATCH
    {
    msg_ptr->denoise_grid.enable = params_.enable;
    // msg_ptr->denoise_grid.enable = ;
    createGridPointsMap(msg_ptr);


    // std::vector<std::vector<double>> feature_vec(rowcol_ptidxs_map_.size());
    // H: result output
    std::vector<int> &pt_num = msg_ptr->denoise_grid.point_num;
    std::vector<int> &pos_x = msg_ptr->denoise_grid.pos_x;
    std::vector<int> &pos_y = msg_ptr->denoise_grid.pos_y;
    std::vector<NoiseType> &noise_type = msg_ptr->denoise_grid.noise_type;
    noise_type.clear();
    msg_ptr->denoise_grid.xmin = params_.range.xmin;
    msg_ptr->denoise_grid.xmax = params_.range.xmax;
    msg_ptr->denoise_grid.ymin = params_.range.ymin;
    msg_ptr->denoise_grid.ymax = params_.range.ymax;
    msg_ptr->denoise_grid.xsize = params_.xsize;
    msg_ptr->denoise_grid.ysize = params_.ysize;
    msg_ptr->denoise_grid.unit_size = params_.unit_size;
    int grid_num = params_.grid_num;
    pt_num.resize(grid_num,0);
    pos_x.resize(grid_num,0);
    pos_y.resize(grid_num,0);
    noise_type.resize(grid_num,NoiseType::IGNORE);
    if(rowcol_vec_.size()<1){
        return;
    }

    std::vector<std::vector<double>> features(rowcol_vec_.size());
    std::vector<bool> fea_isvalid(rowcol_vec_.size(),false);
    uint64_t obj_num=rowcol_vec_.size();
    uint64_t feature_num=9;
    float obj_feature1D[obj_num*feature_num]={0};
    double xy_size = 100.0;
    for(int gi = 0;gi<rowcol_vec_.size();gi++){
        // std::vector<double> feature;
        bool is_valid = feature_extractor_ptr_->getFeatures_grid(msg_ptr,rowcol_ptidxs_map_[rowcol_vec_[gi]],features[gi],xy_size);
        if(!is_valid){
            //no need judge. set result directly.
            fea_isvalid[gi] = is_valid;
        }else{
            fea_isvalid[gi] = is_valid;
            for(int i=0;i<features[gi].size();i++){
                obj_feature1D[gi*feature_num+i] = features[gi][i];
            }
            
        }
    }

// //H: save data to check
// std::string filePath = "/home/mogo/data/humengmeng/2_vehicle/catkin_ws_2100XGB/save_data/"; 
//   std::string fileName = "noisegrid_pointcloud_t";
//   std::string fileExt = ".txt";
//   double timestamp = msg_ptr->timestamp;
//   std::string fileNameExt = fileName + std::to_string(timestamp) + fileExt;
//   std::cout<<"timestamp:"<<timestamp<<std::endl;
//   std::string filePathNameExt = filePath + fileNameExt;
//   std::ofstream ofs;
//   std::cout << "Create text file for this frame: " << fileNameExt << std::endl;
//   ofs.open(filePathNameExt, std::ios::trunc);

// std::cout<<H_B<<"featureslist--[posx,posy] label id:fea(id)"<<std::endl;
// const auto &cloud_ptr = msg_ptr->scan_ptr;
//     for(int gi = 0;gi<rowcol_vec_.size();gi++){
//         int idx0 = rowcol_ptidxs_map_[rowcol_vec_[gi]][0];
//         auto &pt = msg_ptr->scan_ptr->points[idx0];
//         double pt_num_i = rowcol_ptidxs_map_[rowcol_vec_[gi]].size();
//         double pos_x_i = std::floor(pt.x)+0.5;
//         double pos_y_i = std::floor(pt.y)+0.5;
//         int x_id = (pt.x-params_.range.xmin)/params_.unit_size;
//         int y_id = (pt.y-params_.range.ymin)/params_.unit_size;
//         int rowcol =  x_id* params_.ysize + y_id;
//         if(fea_isvalid[gi]){
//             ofs << rowcol << " " << pos_x_i << " " << pos_y_i;
//             for(auto pi:rowcol_ptidxs_map_[rowcol_vec_[gi]]){
//                 auto &pt = cloud_ptr->points[pi];
//                 ofs <<" "<< pt.x << " " << pt.y << " " << pt.z<<" "<<pt.intensity;
//             }
//             ofs<< std::endl;
//         }
//     }
// ofs.close();

    // H: objects' feature --> DMatrix
    xgb_ptr_->prepareData(obj_feature1D,obj_num,feature_num,0.0);
    // H: xgboost predict
    // uint64_t gt_test_len;
    // float const *gt_test_results;
    // xgb_ptr_->getLabel(&gt_test_len, &gt_test_results);
    // xgb_ptr_->showValue(gt_test_results,"test:",10);
    uint64_t const *out_shape; // Shape of output prediction
    uint64_t out_dim;          // Dimension of output prediction
    float const *out_results;  // Pointer to a thread local contigious array, assigned in prediction function.
    // int run_times = 1;
    // for (int i = 0; i < run_times; i++) {
    xgb_ptr_->predict(&out_shape, &out_dim, &out_results);
    // }
    // std::cout <<H_R<< "rowcol_vec_.size():" << rowcol_vec_.size() << std::endl;
    // std::cout <<H_R<< "out_shape[0]:" << out_shape[0] << H_0<<std::endl;
    // xgb_ptr_->showValue(out_results,"denoise result" ,out_shape[0]);
    // double cost_time = timePredict.toc();
    
    ROS_DEBUG_STREAM("denoise to predict grid num:"<<rowcol_vec_.size());
    msg_ptr->denoise_grid.noisemapobj.clear();
    msg_ptr->denoise_grid.noisemapobj.reserve(rowcol_vec_.size());
    msg_ptr->denoise_grid.noisemapnos.clear();
    msg_ptr->denoise_grid.noisemapnos.reserve(rowcol_vec_.size());
    //std::cout<<H_B<<"featureslist--[posx,posy] label id:fea(id)"<<std::endl;
    for(int gi = 0;gi<rowcol_vec_.size();gi++){
        int idx0 = rowcol_ptidxs_map_[rowcol_vec_[gi]][0];
        auto &pt = msg_ptr->scan_ptr->points[idx0];
        double pt_num_i = rowcol_ptidxs_map_[rowcol_vec_[gi]].size();
        double pos_x_i = std::floor(pt.x)+0.5;
        double pos_y_i = std::floor(pt.y)+0.5;
        pt_num.push_back(pt_num_i);
        pos_x.push_back(pos_x_i);
        pos_y.push_back(pos_y_i);
        if(fea_isvalid[gi]){
            // std::cout<<"denoise Grid out_results["<<gi<<"]:"<<out_results[gi]<<std::endl;
            bool label = out_results[gi]>params_.confidence;
            // printf("[%4.1f ,%4.1f ] (%5.2f) ",pos_x_i,pos_y_i,out_results[gi]);
            // std::cout<<label<<" ";
            // for(int fi=0;fi<feature_num;fi++){
            //     double fea = obj_feature1D[gi*feature_num+fi];
            //     std::cout<<fi+1<<":"<<fea<<" ";
            // }
            // std::cout<<std::endl;
            if(label){
                msg_ptr->denoise_grid.noisemapnos.emplace_back(pos_x_i,pos_y_i);
                noise_type[rowcol_vec_[gi]] = NoiseType::NOISE;
            }else{
                msg_ptr->denoise_grid.noisemapobj.emplace_back(pos_x_i,pos_y_i);
                noise_type[rowcol_vec_[gi]] = NoiseType::OBJECT;
            }
        }
    }
    }
    END_TRY_CATCH
}
} // namespace robosense
