#include "common/base_cuda_check.h"
#include "common/include/tic_toc.h"
#include "lidarrcnn/preprocess_cuda.h"
#include <random>       // std::default_random_engine
#include <chrono>       // std::chrono::system_clock


#define DIVUP(m, n) ((m) / (n) + ((m) % (n) > 0))
#define TEST_CRASH 0
namespace robosense {

__global__ void process_point_kernel(
    float* proposal_features,
    const pcl::PointXYZI* in_cloud,
    const int* random_pc_valid_indice,
    const int num_points,
    int *point_count,
    const float* proposal,
    const int num_points_per_proposal,
    const int start_idx_in_batch) {
    int th_i = threadIdx.x + blockIdx.x * blockDim.x;

    if (th_i >= num_points) {
        return;
    }
    float yaw = proposal[6];
    float cos_yaw = std::cos(yaw);
    float sin_yaw = std::sin(yaw);

    pcl::PointXYZI p = in_cloud[random_pc_valid_indice[th_i]];
    float px = p.x;
    float py = p.y;
    float pz = p.z;

    int resCount = (int)atomicAdd(point_count, 1);
    if (resCount < num_points_per_proposal) {
        px -= proposal[0];
        py -= proposal[1];
        pz -= proposal[2];
        float norm_px = cos_yaw * px + sin_yaw * py;
        float norm_py = -1.0 * sin_yaw * px + cos_yaw * py;
        
        proposal_features[start_idx_in_batch + resCount] = norm_px;
        proposal_features[start_idx_in_batch + resCount + num_points_per_proposal] = norm_py;
        proposal_features[start_idx_in_batch + resCount + 2*num_points_per_proposal] = pz;
        proposal_features[start_idx_in_batch + resCount + 3*num_points_per_proposal] = proposal[3] / 2 - norm_px;
        proposal_features[start_idx_in_batch + resCount + 4*num_points_per_proposal] = proposal[4] / 2 - norm_py;
        proposal_features[start_idx_in_batch + resCount + 5*num_points_per_proposal] = proposal[5] / 2 - pz;
        proposal_features[start_idx_in_batch + resCount + 6*num_points_per_proposal] = proposal[3] / 2 + norm_px;
        proposal_features[start_idx_in_batch + resCount + 7*num_points_per_proposal] = proposal[4] / 2 + norm_py;
        proposal_features[start_idx_in_batch + resCount + 8*num_points_per_proposal] = proposal[5] / 2 + pz;
    }
    
}


__global__ void random_fill(
    float* proposal_features,
    const int* point_count, 
    const int num_points_per_proposal, 
    const int start_idx_in_batch){
    for (int fill_id = *point_count + 1; fill_id < num_points_per_proposal; fill_id++) {
        int mapping_idx = (fill_id - *point_count) % (*point_count + 1);
        proposal_features[start_idx_in_batch + fill_id] = proposal_features[start_idx_in_batch + mapping_idx];
        proposal_features[start_idx_in_batch + fill_id + num_points_per_proposal] = proposal_features[start_idx_in_batch + mapping_idx + num_points_per_proposal];
        proposal_features[start_idx_in_batch + fill_id + 2*num_points_per_proposal] = proposal_features[start_idx_in_batch + mapping_idx + 2*num_points_per_proposal];
        proposal_features[start_idx_in_batch + fill_id + 3*num_points_per_proposal] = proposal_features[start_idx_in_batch + mapping_idx + 3*num_points_per_proposal];
        proposal_features[start_idx_in_batch + fill_id + 4*num_points_per_proposal] = proposal_features[start_idx_in_batch + mapping_idx + 4*num_points_per_proposal];
        proposal_features[start_idx_in_batch + fill_id + 5*num_points_per_proposal] = proposal_features[start_idx_in_batch + mapping_idx + 5*num_points_per_proposal];
        proposal_features[start_idx_in_batch + fill_id + 6*num_points_per_proposal] = proposal_features[start_idx_in_batch + mapping_idx + 6*num_points_per_proposal];
        proposal_features[start_idx_in_batch + fill_id + 7*num_points_per_proposal] = proposal_features[start_idx_in_batch + mapping_idx + 7*num_points_per_proposal];
        proposal_features[start_idx_in_batch + fill_id + 8*num_points_per_proposal] = proposal_features[start_idx_in_batch + mapping_idx + 8*num_points_per_proposal];
    }  

}

void LiDARRCNNPreprocess::allocateMemory() {
    TV_CHECK_CUDA_ERR_V3(cudaMalloc((void**)&pc_dev_, max_pc_nums_ * sizeof(pcl::PointXYZI)));
    TV_CHECK_CUDA_ERR_V3(cudaMalloc((void**)&random_pc_valid_indice_dev_, max_pc_nums_ * sizeof(int)));

    TV_CHECK_CUDA_ERR_V3(cudaStreamCreate(&cpy_stream_));
    TV_CHECK_CUDA_ERR_V3(cudaStreamCreate(&pre_stream_));
}


void LiDARRCNNPreprocess::ProcessPoints(const Object::Ptr &proposal, const int prop_id_in_batch){
    const auto& num_points_per_proposal = params_ptr_->num_points_per_proposal;
    const auto& pts_feature_num = params_ptr_->pts_feature_num;
    const auto& batch_size = params_ptr_->batch_size;

    pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_in_proposal(new pcl::PointCloud<pcl::PointXYZI>()); ;

    CellInfoPtr cell_info_ptr;
    for (auto &cell_id : proposal->cell_indices) {
        cell_info_ptr = lidar_msg_ptr_->grid_map_ptr->getCellInfo(cell_id);
        for (auto &point_id : cell_info_ptr->points_indices_) {
            pointcloud_in_proposal->push_back(lidar_msg_ptr_->scan_ptr->points[point_id]);
        }
    }

    int valid_pc_size = pointcloud_in_proposal->size();
    std::vector<int> random_pc_valid_indice(valid_pc_size);
    std::iota(random_pc_valid_indice.begin(), random_pc_valid_indice.end(), 0);
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    shuffle(random_pc_valid_indice.begin(), random_pc_valid_indice.end(), std::default_random_engine(seed));

    TV_CHECK_CUDA_ERR_V3(cudaMemcpyAsync(pc_dev_, pointcloud_in_proposal->points.data(), valid_pc_size * sizeof(pcl::PointXYZI), cudaMemcpyHostToDevice, cpy_stream_));
    TV_CHECK_CUDA_ERR_V3(cudaMemcpyAsync(random_pc_valid_indice_dev_, &random_pc_valid_indice[0], valid_pc_size * sizeof(int), cudaMemcpyHostToDevice, cpy_stream_));

    int threadsInBlock = 1024;  // 64;
    int num_block = DIVUP(valid_pc_size, threadsInBlock);

    // Synchronize data
    cudaStreamSynchronize(cpy_stream_);
    cudaStreamSynchronize(pre_stream_);

    int start_idx_in_batch = prop_id_in_batch * num_points_per_proposal * pts_feature_num;
    int *point_count;
    TV_CHECK_CUDA_ERR_V3(cudaMalloc((void **)&point_count, sizeof(int)));
    TV_CHECK_CUDA_ERR_V3(cudaMemset(point_count, -1, sizeof(int)));

    std::vector<float> proposal_vec;
    proposal_vec.emplace_back(proposal->center.x());
    proposal_vec.emplace_back(proposal->center.y());
    proposal_vec.emplace_back(proposal->center.z());
    proposal_vec.emplace_back(proposal->size.x());
    proposal_vec.emplace_back(proposal->size.y());
    proposal_vec.emplace_back(proposal->size.z());
    float cosyaw = proposal->direction[0];
    float sinyaw = proposal->direction[1];
    float angle = std::atan2(sinyaw, cosyaw);
    proposal_vec.emplace_back(angle);
    // std::cout << " prp extract in preprocess : " << proposal_vec[0] << " " << proposal_vec[1] << " " << proposal_vec[2] << " " << proposal_vec[3] << " " << proposal_vec[4] << " " << proposal_vec[5] << std::endl;
    float *proposal_gpu;
    TV_CHECK_CUDA_ERR_V3(cudaMalloc((void **)&proposal_gpu, 7 * sizeof(float)));
    TV_CHECK_CUDA_ERR_V3(cudaMemcpy(proposal_gpu, &proposal_vec[0], 7 * sizeof(float), cudaMemcpyHostToDevice));
    // TV_CHECK_CUDA_ERR_V3(cudaMemcpyAsync(proposal_gpu, &proposal_vec[0], 7 * sizeof(float), cudaMemcpyHostToDevice, cpy_stream_));

    process_point_kernel<<<num_block, threadsInBlock, 0, pre_stream_>>>(
        feature_data_ptr_,
        pc_dev_,
        random_pc_valid_indice_dev_,
        valid_pc_size,
        point_count,
        proposal_gpu,
        num_points_per_proposal,
        start_idx_in_batch);
    cudaStreamSynchronize(pre_stream_);
    TV_CHECK_CUDA_ERR_V3(cudaGetLastError())

    int point_count_cpu = -1;    
    TV_CHECK_CUDA_ERR_V3(cudaMemcpy(&point_count_cpu, point_count, sizeof(int), cudaMemcpyDeviceToHost));
    if (point_count_cpu <  num_points_per_proposal - 1){
        random_fill<<<1,1,0,pre_stream_>>>(feature_data_ptr_, point_count, num_points_per_proposal, start_idx_in_batch);
    }

}


void LiDARRCNNPreprocess::preprocess(const LidarFrameMsg::Ptr& lidar_msg_ptr, const int start_idx) {
    lidar_msg_ptr_ = lidar_msg_ptr;
    TV_CHECK_CUDA_ERR_V3(cudaSetDevice(params_ptr_->device_id));

    const auto& proposals = lidar_msg_ptr->objects_proposals;    

    const auto& num_points_per_proposal = params_ptr_->num_points_per_proposal;
    const auto& pts_feature_num = params_ptr_->pts_feature_num;
    const auto& batch_size = params_ptr_->batch_size;

    TV_CHECK_CUDA_ERR_V3(cudaMemsetAsync(feature_data_ptr_, 0, batch_size * pts_feature_num * num_points_per_proposal * sizeof(float), pre_stream_));

    TicToc extract_timer("perception/AI refine/preprocess/extract");
    for (int prop_id = start_idx; prop_id < start_idx + batch_size; prop_id ++){
        if (prop_id == proposals.size()){
            break;
        }
        int prop_id_in_batch = prop_id % batch_size;
        if (proposals.at(prop_id)->cell_indices.size() < 1) {
            continue;
        }
        ProcessPoints(proposals.at(prop_id), prop_id_in_batch);

    }
    extract_timer.print();

    // //airefinedebug
    // for (int prop_id = start_idx; prop_id < start_idx + batch_size; prop_id ++){
    //         if (prop_id == proposals.size()){
    //             break;
    //         }
    //         auto debug_feature_data_ptr_ = std::shared_ptr<float>(new float[batch_size * 9 * 512]);
    //         TV_CHECK_CUDA_ERR_V3(cudaMemcpy(debug_feature_data_ptr_.get(), feature_data_ptr_, batch_size *9*512 * sizeof(float),
    //                 cudaMemcpyDeviceToHost));
    //         int prop_id_in_batch = prop_id % batch_size;

    //         std::ofstream outFile;
    //         std::string outname = "/home/mogo/data/houjiayue/catkin_ws_3.5.0/debug_extractpoint/%f_randidx_proposal_%d_dache_%.2f_%.2f.txt";
    //         char buffer[1000];
    //         std::sprintf(buffer, outname.c_str(), lidar_msg_ptr->timestamp, prop_id, proposals.at(prop_id)->center.x(), proposals.at(prop_id)->center.y());
    //         std::string res(buffer);
    //         outFile.open(res);
    //         for (int i = 0; i < 512; i++){
    //             for (int j = 0; j < 9; j++) {
    //                 outFile << debug_feature_data_ptr_.get()[9 * 512 * prop_id_in_batch + 512 * j + i] << "\n";
    //                 // outFile << debug_feature_data_ptr_.get()[9*512*prop_id_in_batch+512+i] << "\n";
    //                 // outFile << debug_feature_data_ptr_.get()[9*512*prop_id_in_batch+512*2+i] << "\n";
    //             }
                
    //         }
    //         outFile << proposals.at(prop_id)->center.x() << "\n";
    //         outFile << proposals.at(prop_id)->center.y() << "\n";
    //         outFile << proposals.at(prop_id)->center.z() << "\n";
    //         outFile << proposals.at(prop_id)->size.x() << "\n";
    //         outFile << proposals.at(prop_id)->size.y() << "\n";
    //         outFile << proposals.at(prop_id)->size.z() << "\n";
    //         float cosyaw = proposals.at(prop_id)->direction[0];
    //         float sinyaw = proposals.at(prop_id)->direction[1];
    //         float angle = std::atan2(sinyaw, cosyaw);
    //         outFile << angle << "\n";
            
    //         outFile.close();
      
    // }
}

}  // namespace robosense
