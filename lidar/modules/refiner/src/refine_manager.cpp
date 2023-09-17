#include "refine_manager.h"

namespace robosense {

RefineManager::RefineManager()
    : bev_aggregation_ptr(new BevAggregation)
    , single_frame_refiner_ptr(new SingleFrameRefiner)
    , multi_frame_refiner_ptr(new MultiFrameRefiner)
    , denoise_obj_ptr(new DenoiseObj)
    , denoise_frame_ptr(new DenoiseFrame)
    , ai_refine_ptr(new LiDARRCNNAIRefine)
    , exit_flag_(false)
    , denoise_obj_processed(false)
    , ai_refiner_processed(false)
    , ai_refiner_enable_(false)
{
    
}

RefineManager::~RefineManager() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        exit_flag_ = true;
    }
    if (denoise_obj_th_ != nullptr) {
        denoise_obj_cv_.notify_one();
        denoise_obj_th_->join();
    }

    if (ai_refiner_th_ != nullptr) {
        ai_refiner_cv_.notify_one();
        ai_refiner_th_->join();
    }
}

void RefineManager::init(const Rs_YAMLReader &configParse) {
    params_ = configParse.getValue<RefinerParam>("refiner");
    ai_refiner_enable_ = configParse.getValue<bool>("ai_refine.enable");

    bev_aggregation_ptr->init(params_);

    single_frame_refiner_ptr->init(params_);

    multi_frame_refiner_ptr->init(params_);
    multi_frame_refiner_ptr->set_cv(&ai_refiner_cv_, ai_refiner_enable_);

    denoise_obj_ptr->init(configParse);
    denoise_frame_ptr->init(configParse);

    if (denoise_obj_ptr->denoise_enable_)
        denoise_obj_th_ = std::make_shared<std::thread>(std::bind(&RefineManager::denoise_obj_process, this));

    if (ai_refiner_enable_) {
        ai_refine_ptr->init(configParse);
        ai_refiner_th_ = std::make_shared<std::thread>(std::bind(&RefineManager::ai_refiner_process, this));
    }
}

void RefineManager::perception(const LidarFrameMsg::Ptr &msg_ptr) {
    msg_ptr_ = msg_ptr;
    TRY_CATCH 
    {
        msg_ptr_->denoise_enable = denoise_obj_ptr->denoise_enable_;
        TicToc timer("perception/refiner/bev aggregation");
        bev_aggregation_ptr->perception(msg_ptr_);
    }
    {
        TicToc timer("perception/refiner/multi frame refiner");
        multi_frame_refiner_ptr->perception(msg_ptr_);
    }

    if (denoise_obj_ptr->denoise_enable_) {
        denoise_obj_cv_.notify_one();
    }

    {
        TicToc timer("perception/refiner/single frame refiner");
        single_frame_refiner_ptr->perception(msg_ptr_);
    }

    // wait for ai refiner finshed.
    if (ai_refiner_enable_) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (!ai_refiner_processed)
            ai_refiner_cv_.wait(lock, [&]() { return ai_refiner_processed; });
    }

    // wait for denoise finshed.
    if (denoise_obj_ptr->denoise_enable_) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (!denoise_obj_processed)
            denoise_obj_cv_.wait(lock, [&]() { return denoise_obj_processed; });
    }

    update_refiner_obj(msg_ptr_);

    END_TRY_CATCH
    
    {
        std::lock_guard<std::mutex> lock(mutex_);
        ai_refiner_processed = false;
        denoise_obj_processed = false;
    }
}

void RefineManager::update_refiner_obj(const LidarFrameMsg::Ptr &msg_ptr) {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<int> &erase_objs = msg_ptr->refine_data.erase_objs;
    auto &objs_measure = msg_ptr->objects_refine;
    // delete measurement
    if (erase_objs.size() != 0) {
        std::sort(erase_objs.begin(), erase_objs.end(), std::greater<int>());
        for (const auto &i : erase_objs) {
            if(objs_measure.size() > 0 && i < objs_measure.size())
                objs_measure.erase(objs_measure.begin() + i);
        }
    }
    // add ai refine result
    if (ai_refiner_enable_)
        objs_measure.insert(objs_measure.end(), msg_ptr->objects_proposals.begin(), msg_ptr->objects_proposals.end());
}

void RefineManager::denoise_obj_process() {
    while (ros::ok()) {
        {
            std::unique_lock<std::mutex> lock(mutex_);
            denoise_obj_cv_.wait(lock);
            if (exit_flag_)
                break;

            if (denoise_obj_processed) continue;

        }
        {
            TicToc timer("perception/refiner/denoise obj");
            TRY_CATCH
            denoise_obj_ptr->perception(msg_ptr_);
            END_TRY_CATCH
        }
        {
            TicToc timer("perception/refiner/denoise frame");
            TRY_CATCH
            denoise_frame_ptr->perception(msg_ptr_);
            END_TRY_CATCH
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            denoise_obj_processed = true;
        }
        denoise_obj_cv_.notify_one();
    }
}

void RefineManager::ai_refiner_process() {
    while (ros::ok()) {
        {
            std::unique_lock<std::mutex> lock(mutex_);
            ai_refiner_cv_.wait(lock);
            if (exit_flag_)
                break;

            if (ai_refiner_processed) continue;

        }
        {
            TicToc timer("perception/refiner/ai refiner");
            TRY_CATCH
            ai_refine_ptr->perception(msg_ptr_);
            END_TRY_CATCH
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            ai_refiner_processed = true;
        }
        ai_refiner_cv_.notify_one();
    }
}

} // namespace robosense