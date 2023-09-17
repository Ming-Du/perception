#include "denoise_obj.h"

namespace robosense {

void DenoiseObj::init(const Rs_YAMLReader &configParse) {
    params_ = configParse.getValue<DenoiseParam>("denoise");
    denoise_enable_ = params_.enable;
    if (!denoise_enable_) {
        return;
    }
    std::string xgb_obj_model_path_ = configParse.getBasePath() + "/model/xgboost/" + params_.obj_model;
    feature_extractor_ptr_.reset(new DenoiseFeatureExtractor);
    feature_extractor_ptr_->init();
    ROS_INFO_STREAM("xgb_obj_model_path_:" << xgb_obj_model_path_);
    xgb_obj_ptr_.reset(new Xgb);
    int silent = 0; // whether print messages during loading
    xgb_obj_ptr_->init(xgb_obj_model_path_, silent);
}

bool DenoiseObj::isInRange(double x, double y, double z) {
    return (x > params_.range.xmin && x < params_.range.xmax &&
            y > params_.range.ymin && y < params_.range.ymax &&
            z > params_.range.zmin && z < params_.range.zmax);
}

void DenoiseObj::try_grid_data_obj(LidarFrameMsg::Ptr &msg_ptr) {
    // std::vector<Object::Ptr> objects_rule;
    auto &objects_rule = msg_ptr->objects_rule;
    auto &cloud_ptr = msg_ptr->scan_ptr;
    auto &scan_valid_indice = msg_ptr->valid_indices;
    // const auto &points = cloud_ptr->points; // each point struct
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZI>());
    // points.clear();
    scan_valid_indice.clear();

    //----------obj_id:2 ----begin------
    // idx_start:14
    // idx_end  :51
    // ------------------ c++  ------------------
    int dummy_data_num = 37;
    float dummy_data[37][4] = {
        {16.9336, -0.234807, 0.531514, 18},
        {16.9561, -0.274782, 0.533227, 12},
        {17.9235, -2.00086, 0.436169, 18},
        {17.8794, -1.94319, 0.436649, 19},
        {17.9891, -1.59983, 0.581657, 14},
        {17.9832, -1.64175, 0.583663, 12},
        {17.9729, -1.68611, 0.585837, 15},
        {17.9586, -1.72876, 0.587988, 14},
        {17.9174, -1.8422, 1.04442, 15},
        {17.5832, -0.0131624, 0.423966, 21},
        {17.4982, -0.299889, 2.18652, 19},
        {17.7623, -0.3705, 2.21869, 16},
        {17.5528, -0.00198752, 2.2712, 21},
        {17.5361, -0.0466594, 2.27098, 19},
        {17.5548, -0.087208, 2.27503, 16},
        {17.5813, -0.128084, 2.28007, 15},
        {17.5204, -0.173228, 2.2744, 18},
        {17.7334, 0.0617975, 0.41954, 20},
        {17.7181, 0.0182898, 0.420361, 21},
        {17.5543, 0.082141, 2.2679, 13},
        {17.5417, 0.0398507, 2.26807, 16},
        {17.7853, 0.467527, 0.481502, 17},
        {17.822, 0.428747, 0.483051, 20},
        {17.8585, 0.389741, 0.484606, 19},
        {17.8593, 0.344971, 0.486999, 17},
        {17.8558, 0.301214, 0.489396, 18},
        {17.8799, 0.260615, 0.491199, 17},
        {17.8325, 0.211346, 0.494536, 15},
        {17.7449, 0.160214, 0.49857, 24},
        {18.2988, -2.00257, 1.05945, 15},
        {18.3113, -2.04789, 1.06181, 14},
        {18.2392, -1.68494, 1.04334, 17},
        {18.2168, -1.72782, 1.04493, 13},
        {18.2223, -1.77207, 1.04712, 15},
        {18.2195, -1.81803, 1.04923, 17},
        {18.2416, -1.90769, 1.05388, 14},
        {18.2742, -1.95633, 1.0568, 17},
    };
    int dummy_polygon_num = 7;
    float dummy_polygon[7][2] = {
        {16.9336, -0.234807},
        {17.7853, 0.467527},
        {17.822, 0.428747},
        {17.8585, 0.389741},
        {18.3113, -2.04789},
        {17.9235, -2.00086},
        {17.8794, -1.94319},
    };

    double area = 1.65708;
    // feature:(label is unsure)
    // 0 1:0.0714286 2:0.142857 3:0.25 4:0.0112325 5:0.3305 6:0 7:28 8:0.64227
    //----------obj_id:2 ---- end ------

    Object::Ptr obj;
    obj.reset(new Object);
    obj->type = ObjectType::UNKNOW;
    obj->mode = ModeType::BSD;
    obj->cloud_indices.reserve(dummy_data_num);

    // test case 1
    //  int data_num = 114;float dummy_data[data_num][4] = {{8.37346,3.70432,0.346578,16.},{8.37321,3.68556,0.348191,181.},{8.98361,3.29501,0.351047,33.},{8.93946,3.25202,0.356916,31.},{8.93706,3.23225,0.358721,34.},{8.95851,3.22685,0.358112,36.},{8.879,3.1651,0.367343,49.},{8.89369,3.15526,0.367449,11.},{8.93594,3.16135,0.364822,36.},{8.93976,3.14625,0.365919,14.},{8.97861,3.15005,0.363657,60.},{8.47171,3.29831,0.449535,10.},{8.4865,3.29047,0.449606,37.},{8.41057,3.22444,0.45727,22.},{8.63533,3.06465,0.461579,26.},{8.7331,3.10271,0.455281,46.},{8.80856,3.30595,0.515255,41.},{8.77896,3.27006,0.51825,42.},{8.78316,3.25536,0.519153,38.},{8.83203,3.26705,0.517402,44.},{8.8018,3.23192,0.520363,46.},{8.77509,3.19856,0.523136,33.},{8.77557,3.18193,0.52425,30.},{8.75176,3.15134,0.526784,30.},{8.75932,3.13821,0.527526,10.},{8.7874,3.13739,0.527037,42.},{8.78413,3.11886,0.528359,11.},{8.80562,3.11327,0.528325,10.},{8.84424,3.11786,0.52727,10.},{8.75667,3.05334,0.533352,35.},{8.7531,3.03495,0.534677,28.},{8.76026,3.02183,0.535439,11.},{8.78118,3.01675,0.53539,21.},{8.80569,3.01332,0.535161,26.},{8.86587,3.02782,0.533028,28.},{8.9117,3.03514,0.53166,26.},{8.96478,3.04574,0.529933,13.},{8.98621,3.03915,0.529987,12.},{8.97169,3.01498,0.53193,63.},{8.61855,3.61881,0.576831,22.},{8.63382,3.61135,0.577126,13.},{8.65896,3.61061,0.576987,19.},{8.6943,3.61615,0.576435,21.},{8.77827,3.65665,0.573631,15.},{8.90214,3.72371,0.569097,22.},{8.75139,3.34393,0.591331,22.},{8.73204,3.31456,0.593117,26.},{8.71945,3.28939,0.594629,26.},{8.68305,3.24941,0.597119,32.},{8.6598,3.21848,0.599023,40.},{8.53705,3.12796,0.604871,48.},{8.56853,3.12994,0.604604,63.},{8.54814,3.10174,0.606343,60.},{8.53795,3.07975,0.607675,50.},{8.53465,3.06174,0.608744,40.},{8.528,3.04143,0.609966,37.},{8.57302,3.05148,0.609178,41.},{8.57304,3.03543,0.610125,38.},{8.92828,3.70425,0.653409,15.},{8.93414,3.68868,0.654164,161.},{8.5575,3.22904,0.745763,34.},{8.55473,3.21084,0.746402,37.},{8.35094,3.66522,0.801016,15.},{8.34112,3.63896,0.801224,15.},{8.35897,3.54389,0.803819,16.},{8.3773,3.5395,0.804539,19.},{8.43777,3.47754,0.808067,12.},{8.45604,3.47266,0.808832,19.},{8.43814,3.44265,0.808923,31.},{8.4137,3.40794,0.8089,29.},{8.38915,3.37331,0.808876,31.},{8.38718,3.35516,0.809261,26.},{8.39502,3.34385,0.809835,26.},{8.38976,3.32325,0.810173,16.},{8.42092,3.32728,0.811217,19.},{8.43861,3.32255,0.811998,12.},{8.37666,3.26418,0.811242,12.},{8.49444,3.32547,0.814006,14.},{8.39526,3.24345,0.812502,19.},{8.39954,3.22968,0.813043,23.},{8.42733,3.23103,0.814063,14.},{8.51865,3.19189,0.818727,19.},{8.57059,3.2068,0.820327,16.},{8.46177,3.1242,0.818513,19.},{8.56814,3.17257,0.821267,13.},{8.56189,3.15203,0.821649,16.},{8.59651,3.15656,0.822897,13.},{8.78654,3.06622,0.833728,19.},{8.77955,3.04559,0.834154,10.},{8.77226,3.02544,0.834556,26.},{8.75795,3.00133,0.834805,26.},{8.04091,3.45497,0.863501,20.},{8.04304,3.43954,0.863731,17.},{8.1358,3.49635,0.867486,17.},{8.12522,3.47106,0.867232,22.},{8.13678,3.46282,0.867848,24.},{8.14519,3.45205,0.868347,24.},{8.14065,3.43179,0.868341,25.},{8.16163,3.43075,0.869343,24.},{8.25293,3.38218,0.874319,20.},{8.25781,3.36867,0.874737,11.},{8.27866,3.36712,0.875789,23.},{8.28672,3.35591,0.876343,10.},{8.29495,3.34436,0.876918,10.},{8.4566,3.04385,0.89105,20.},{8.43595,3.0159,0.890546,23.},{8.20588,3.48039,0.230668,19.},{8.04715,3.34192,0.257828,22.},{8.0523,3.32877,0.258833,18.},{8.05392,3.31363,0.260324,22.},{8.70863,3.34129,0.283442,36.},{8.57147,3.23823,0.302088,41.},{8.61325,3.24658,0.29859,48.},{8.59717,3.21981,0.302166,47.}};
    // test case 2
    //  int data_num = 38;float dummy_data[data_num][4] = {{0.686804,3.44889,0.751764,21.},{0.639827,3.4954,0.750141,16.},{0.575915,3.55245,0.748463,173.},{0.902791,3.13069,0.927759,35.},{0.987269,3.10232,0.922879,43.},{0.99779,3.11353,0.921257,40.},{0.882238,3.19402,0.925068,37.},{0.812922,3.25064,0.926783,31.},{0.788997,3.28186,0.926577,31.},{0.807055,3.28977,0.924581,42.},{0.797354,3.31345,0.923747,48.},{0.959555,3.02335,1.00879,37.},{0.439562,3.68524,1.0191,16.},{0.465539,3.68981,1.01636,14.},{0.553281,3.32086,1.11702,18.},{0.546923,3.34303,1.11656,17.},{0.51264,3.38055,1.11844,13.},{0.485548,3.41448,1.1197,11.},{0.497145,3.42758,1.11773,19.},{0.616901,3.48002,1.27736,14.},{0.624629,3.49367,1.27581,14.},{0.626238,3.51254,1.27496,15.},{0.620824,3.53504,1.275,14.},{0.635992,3.54534,1.27252,18.},{0.823435,3.01276,1.34596,27.},{0.837471,3.02367,1.34335,23.},{0.843754,3.03664,1.34193,21.},{0.843549,3.05429,1.34141,14.},{0.850284,3.06796,1.33991,12.},{0.37653,3.0448,0.451777,23.},{0.339945,3.07986,0.447248,28.},{0.763722,3.03964,0.457579,23.},{0.831441,3.02325,0.460365,26.},{0.884386,3.03046,0.460194,29.},{0.827707,3.92532,0.350264,18.},{0.844447,3.93474,0.349187,18.},{0.848445,3.95364,0.346815,18.},{0.867676,3.95952,0.346195,21.}};
    for (int i = 0; i < dummy_data_num; i++) {
        float x = dummy_data[i][0];
        float y = dummy_data[i][1];
        float z = dummy_data[i][2];
        float ist = dummy_data[i][3];
        pcl::PointXYZI p;
        p.x = x;
        p.y = y;
        p.z = z;
        p.intensity = ist;
        pointcloud->push_back(p);
        // pointcloud->push_back(pcl::PointXYZI(x,y,z,ist));
        // scan_valid_indice.push_back(i);
        obj->cloud_indices.emplace_back(i);
    }
    obj->polygons.resize(dummy_polygon_num);
    for (size_t i = 0; i < dummy_polygon_num; ++i) {
        obj->polygons[i].x() = dummy_polygon[i][0];
        obj->polygons[i].y() = dummy_polygon[i][1];
        obj->polygons[i].z() = 0;
    }
    objects_rule.push_back(obj);
    cloud_ptr = pointcloud;
}

double DenoiseObj::calcAreaFromConvex(const std::vector<Eigen::Vector3d> &polygon2D) {
    std::vector<double> points_x;
    std::vector<double> points_y;
    for (Eigen::Vector3d p : polygon2D) {
        points_x.push_back(p.x());
        points_y.push_back(p.y());
    }
    int polygon_pointnum = points_x.size();
    int j = 0;
    double area = 0;
    double px0 = points_x[0];
    double py0 = points_y[0];
    for (int i = 1; i < polygon_pointnum - 1; i++) {
        j = i + 1;
        double pxi = points_x[i] - points_x[0];
        double pyi = points_y[i] - points_y[0];
        double pxj = points_x[j] - points_x[0];
        double pyj = points_y[j] - points_y[0];
        area += (pxi * pyj - pyi * pxj) / 2;
    }
    // std::cout<<"area not abs:"<<area;
    area = fabs(area);
    return area;
}

void DenoiseObj::createObjPointsMap(const LidarFrameMsg::Ptr &msg_ptr, std::vector<int> &cloud_indices, std::vector<int> &obj_ptidx) {
    const auto &cloud_ptr = msg_ptr->scan_ptr;
    obj_ptidx.reserve(cloud_indices.size());
    for (size_t i = 0; i < cloud_indices.size(); ++i) {
        const auto &pt_idx = cloud_indices.at(i);
        const auto &tmp_pt = cloud_ptr->points[pt_idx];
        if (std::isnan(tmp_pt.x) || std::isnan(tmp_pt.y) || std::isnan(tmp_pt.z)) {
            continue;
        }
        if (!isInRange(tmp_pt.x, tmp_pt.y, tmp_pt.z)) {
            continue;
        }
        obj_ptidx.emplace_back(pt_idx);
    }
}

void DenoiseObj::showObjectPoints(const LidarFrameMsg::Ptr &msg_ptr, std::vector<int> &obj_ptidx) {
    const auto &cloud_ptr = msg_ptr->scan_ptr;
    bool is_inRoi = true;
    for (size_t i = 0; i < obj_ptidx.size(); ++i) {
        const auto &pt_idx = obj_ptidx.at(i);
        const auto &tmp_pt = cloud_ptr->points[pt_idx];
        if (tmp_pt.x < 7 || tmp_pt.x > 9.3 || tmp_pt.y < -2.4 || tmp_pt.y > -0.8) {
            is_inRoi = false;
            break;
        }
    }
    if (is_inRoi) {
        std::cout << "inROI----begin---" << std::endl;
        for (size_t i = 0; i < obj_ptidx.size(); ++i) {
            const auto &pt_idx = obj_ptidx.at(i);
            const auto &tmp_pt = cloud_ptr->points[pt_idx];
            std::cout << tmp_pt.x << "," << tmp_pt.y << "," << tmp_pt.z << "," << tmp_pt.intensity << ",";
        }
        std::cout << "inROI----end---" << std::endl;
    }
}

double DenoiseObj::calcWeightObject(double conf_predict, double conf_model) {
    double obj_weight = 0.0;
    if (conf_predict < conf_model) {
        obj_weight = conf_predict / conf_model - 1;
    } else {
        obj_weight = (conf_model - conf_predict) / (conf_model - 1);
    }
    return obj_weight;
}

void DenoiseObj::perception(const LidarFrameMsg::Ptr &msg_ptr) {
    TRY_CATCH
    std::vector<int> &erase_objs = msg_ptr->refine_data.erase_objs;
    uint64_t obj_num = msg_ptr->objects_refine.size();
    uint64_t feature_num = 9;
    float obj_feature1D[obj_num * feature_num] = {0};
    int valid_obj_num = 0;
    std::vector<int> denoise_objid;
    denoise_objid.reserve(obj_num);
    // calculator object num
    {
        std::lock_guard<std::mutex> lock(mutex_);
        int obj_idx = -1;
        for (auto &obj_rule : msg_ptr->objects_refine) {
            obj_idx++;
            if (std::find(erase_objs.begin(), erase_objs.end(), obj_idx) != erase_objs.end()) {
                continue;
            }
            std::vector<double> obj_feature;
            double area = calcAreaFromConvex(obj_rule->polygons);
            double xy_size = area * 100;
            if (xy_size < 1) { // safe check & feature stabilize
                continue;
            }
            std::vector<int> obj_ptidx;
            createObjPointsMap(msg_ptr, obj_rule->cloud_indices, obj_ptidx);

            std::vector<double> features;
            bool is_valid = feature_extractor_ptr_->getFeatures_obj(msg_ptr, obj_ptidx, features, xy_size);
            if (!is_valid) {
                continue;
            } else {
                // showObjectPoints(msg_ptr,obj_ptidx);
                for (size_t i = 0; i < features.size(); i++) {
                    obj_feature1D[valid_obj_num * feature_num + i] = features[i];
                }
                valid_obj_num++;
                denoise_objid.emplace_back(obj_idx);
            }
            // file_objs_feature.push_back(features);
        }
    }
    obj_num = valid_obj_num;
    if (obj_num < 1) {
        return;
    }

    // H: objects' feature --> DMatrix
    TicToc time_prepare("perception/denoise object/prepare");
    xgb_obj_ptr_->prepareData(obj_feature1D, obj_num, feature_num, 0.0);
    time_prepare.print();

    // H: xgboost predict
    // uint64_t gt_test_len;
    // float const *gt_test_results;
    // xgb_ptr_->getLabel(&gt_test_len, &gt_test_results);
    // xgb_ptr_->showValue(gt_test_results,"test:",10);
    uint64_t const *out_shape; // Shape of output prediction
    uint64_t out_dim;          // Dimension of output prediction
    float const *out_results;  // Pointer to a thread local contigious array, assigned in prediction function.
    TicToc time_predict("perception/denoise object/predict");
    xgb_obj_ptr_->predict(&out_shape, &out_dim, &out_results);
    time_predict.print();

    // set object prop
    {
        std::lock_guard<std::mutex> lock(mutex_);
        for (int i = 0; i < obj_num; i++) {
            // auto &noise_state = msg_ptr->objects_rule.at(denoise_objid.at(i))->object_state.noise_state;
            auto &noise_state = msg_ptr->objects_refine.at(denoise_objid.at(i))->noise_state_obj;
            auto &denoise_obj_weight = msg_ptr->objects_refine.at(denoise_objid.at(i))->denoise_obj_weight;
            denoise_obj_weight = calcWeightObject(out_results[i], params_.obj_confidence);
            // std::cout<<"denoise obj out_results["<<i<<"]:"<<out_results[i]<<std::endl;
            bool label = out_results[i] > params_.obj_confidence;
            if (label) {
                // msg_ptr->denoise_grid.noisemapnos.emplace_back(pos_x_i,pos_y_i);
                noise_state = NoiseState::NOISE_NOISE;
            } else {
                // msg_ptr->denoise_grid.noisemapobj.emplace_back(pos_x_i,pos_y_i);
                noise_state = NoiseState::NOISE_OBJECT;
            }
        }
    }
    END_TRY_CATCH
}
} // namespace robosense
