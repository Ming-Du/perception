#pragma once

#include <deque>
#include <list>
#include <memory>
#include <vector>

namespace perception {
namespace fusion {
// @brief general object pool interface
template <class ObjectType>
class BaseObjectPool {
 public:
  // TODO(All): remove
  // typedef std::shared_ptr<ObjectType> ObjectTypePtr;

  // @brief default constructor
  BaseObjectPool() = default;
  // @brief default destructor
  virtual ~BaseObjectPool() = default;
  // @brief pure virtual function to get object smart pointer
  virtual std::shared_ptr<ObjectType> Get() = 0;
  // @brief pure virtual function to get batch of smart pointers
  // @params[IN] num: batch number
  // @params[OUT] data: vector container to store the pointers
  virtual void BatchGet(size_t num, std::vector<std::shared_ptr<ObjectType>>* data) = 0;
  // @brief pure virtual function to get batch of smart pointers
  // @params[IN] num: batch number
  // @params[IN] is_front: indicating insert to front or back of the list
  // @params[OUT] data: list container to store the pointers
  virtual void BatchGet(size_t num,
                        bool is_front,
                        std::list<std::shared_ptr<ObjectType>>* data) = 0;
  // @brief pure virtual function to get batch of smart pointers
  // @params[IN] num: batch number
  // @params[IN] is_front: indicating insert to front or back of the deque
  // @params[OUT] data: deque container to store the pointers
  virtual void BatchGet(size_t num,
                        bool is_front,
                        std::deque<std::shared_ptr<ObjectType>>* data) = 0;
  // @brief virtual function to set capacity
  virtual void set_capacity(size_t capacity) {}
  // @brief capacity getter
  size_t get_capacity() { return capacity_; }
  // @brief get remained object number
  virtual size_t RemainedNum() { return 0; }

 protected:
  BaseObjectPool(const BaseObjectPool& rhs) = delete;
  BaseObjectPool& operator=(const BaseObjectPool& rhs) = delete;
  size_t capacity_ = 0;
};  // class BaseObjectPool

// @brief dummy object pool implementation, not managing memory
template <class ObjectType>
class DummyObjectPool : public BaseObjectPool<ObjectType> {
 public:
  // @brief Only allow accessing from global instance
  static DummyObjectPool& Instance() {
    static DummyObjectPool pool;
    return pool;
  }
  // @brief overrided function to get object smart pointer
  std::shared_ptr<ObjectType> Get() override { return std::shared_ptr<ObjectType>(new ObjectType); }
  // @brief overrided function to get batch of smart pointers
  // @params[IN] num: batch number
  // @params[OUT] data: vector container to store the pointers
  void BatchGet(size_t num, std::vector<std::shared_ptr<ObjectType>>* data) override {
    for (size_t i = 0; i < num; ++i) {
      data->emplace_back(std::shared_ptr<ObjectType>(new ObjectType));
    }
  }
  // @brief overrided function to get batch of smart pointers
  // @params[IN] num: batch number
  // @params[IN] is_front: indicating insert to front or back of the list
  // @params[OUT] data: list container to store the pointers
  void BatchGet(size_t num, bool is_front, std::list<std::shared_ptr<ObjectType>>* data) override {
    for (size_t i = 0; i < num; ++i) {
      is_front ? data->emplace_front(std::shared_ptr<ObjectType>(new ObjectType))
               : data->emplace_back(std::shared_ptr<ObjectType>(new ObjectType));
    }
  }
  // @brief overrided function to get batch of smart pointers
  // @params[IN] num: batch number
  // @params[IN] is_front: indicating insert to front or back of the deque
  // @params[OUT] data: deque container to store the pointers
  void BatchGet(size_t num, bool is_front, std::deque<std::shared_ptr<ObjectType>>* data) override {
    for (size_t i = 0; i < num; ++i) {
      is_front ? data->emplace_front(std::shared_ptr<ObjectType>(new ObjectType))
               : data->emplace_back(std::shared_ptr<ObjectType>(new ObjectType));
    }
  }

 protected:
  // @brief default constructor
  DummyObjectPool() = default;
};  // class DummyObjectPool

}  // namespace fusion
}  // namespace perception
