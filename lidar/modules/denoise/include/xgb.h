#ifndef XGB_H_
#define XGB_H_

#include <assert.h>
#include <iostream>
#include <memory>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include "xgboost_lib/c_api.h"


#define safe_xgboost(err)                                                     \
    if ((err) != 0) {                                                         \
        fprintf(stderr, "%s:%d: error in %s: %s\n", __FILE__, __LINE__, #err, \
                XGBGetLastError());                                           \
        exit(1);                                                              \
    }

#define safe_malloc(ptr)                                                 \
    if ((ptr) == NULL) {                                                 \
        fprintf(stderr, "%s:%d: Failed to allocate memory.\n", __FILE__, \
                __LINE__);                                               \
        exit(1);                                                         \
    }

class Xgb {
public:
    using Ptr = std::shared_ptr<Xgb>;
    Xgb() {}
    void init(std::string model_path, int silent);
    void showXgboostVersion();
    void prepareData(const float *obj_feature, int obj_num, int feature_num,float miss_value);
    void loadData(std::string data_path);
    void loadModel(std::string model_path);
    void predict(uint64_t const **out_shape, uint64_t *out_dim, float const **out_results);
    void showValue(float const *out_results, std::string title, unsigned int num);
    void getLabel(uint64_t *out_len, float const **out_results);

    void showLabel(std::string str, int num);
    void showPrecision(const float *gt, const float *pr, uint64_t len);
    void showLabelCnt(const float *y, float val, uint64_t len);

private:
public:
private:
    BoosterHandle booster_;
    DMatrixHandle test_data_;
    int silent_;
};

#endif
