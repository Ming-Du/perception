#include <xgb.h>
//public
void Xgb::init(std::string model_path,int silent){
  // int silent = 0;//whether print messages during loading
  safe_xgboost(XGBoosterCreate(NULL, 0, &booster_));
  silent_ = silent;
  // showXgboostVersion();
  loadModel(model_path);
}
void Xgb::showXgboostVersion(){
    int major;
    int minor;
    int patch;
    XGBoostVersion(&major, &minor, &patch);
    std::cout<< "xgboost version:"<<major<<"."<<minor<<"."<<patch<<std::endl;
}
void Xgb::prepareData(const float *obj_feature, int obj_num, int feature_num,float miss_value){
  safe_xgboost(XGDMatrixCreateFromMat(obj_feature, obj_num, feature_num, miss_value, &test_data_));
}
void Xgb::loadData(std::string data_path){
    safe_xgboost(XGDMatrixCreateFromFile(data_path.c_str(), silent_, &test_data_));
}
void Xgb::loadModel(std::string model_path){
    safe_xgboost(XGBoosterLoadModel(booster_, model_path.c_str()));
}
void Xgb::predict(uint64_t const **out_shape,uint64_t *out_dim,float const **out_results){
    /* Run prediction with DMatrix object. */
    char const config[] =
        "{\"training\": false, \"type\": 0, "
        "\"iteration_begin\": 0, \"iteration_end\": 0, \"strict_shape\": false}";
    // safe_xgboost(XGBoosterPredictFromDMatrix(booster, test_data, config, &out_shape,&out_dim, &out_results));
    safe_xgboost(XGBoosterPredictFromDMatrix(booster_, test_data_, config, out_shape,out_dim, out_results));
}
void Xgb::showValue(float const *out_results,std::string title,unsigned int num){
    std::cout<<title<<":";//    printf("y_pred: ");
    // for (int i = 0; i < out_dim; ++i) {
    // for (unsigned int i = 0; i < out_shape[0]; ++i) {
    for (unsigned int i = 0; i < 10; ++i) {
      printf("%1.2f, ", out_results[i]);
    }
    printf("\n");
}
void Xgb::getLabel(uint64_t *out_len,float const **out_results){
    // safe_xgboost(XGDMatrixGetFloatInfo(test_data, "label", &out_len, &out_results));
    safe_xgboost(XGDMatrixGetFloatInfo(test_data_, "label", out_len, out_results));
}

void Xgb::showPrecision(const float *gt,const float *pr,uint64_t len){
  float pr_cur=0.0;
  int num_cor = 0;
  int num_tot = 0;
  int num_err = 0;
  for(uint64_t i=0;i<len;i++){
    if(pr[i]>0.5){pr_cur=1.0;}else{pr_cur=0.0;}
    if(pr_cur==gt[i]){num_cor++;}else{num_err++;}
    num_tot++;
  }
  std::cout<<"total:"<<num_tot<<"\tcorrect:"<<num_cor<<"\terror:"<<num_err<<"\tprecision:"<<num_cor*100./num_tot <<"%"<<std::endl;
}

void Xgb::showLabelCnt(const float *y,float val,uint64_t len){
  int num_cor = 0;
  int num_tot = 0;
  int num_err = 0;
  float cur;
  for(uint64_t i=0;i<len;i++){
    if(y[i]>0.5){cur=1.0;}else{cur=0.0;}
    if(val==cur){num_cor++;}else{num_err++;}
    num_tot++;
  }
  std::cout<<"total:"<<num_tot<<"\tnumis_"<<val<<":"<<num_cor<<"\telse:"<<num_err<<"\tratio:"<<num_cor*100./num_tot <<"%"<<std::endl;
}
//private
