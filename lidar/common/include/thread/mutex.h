#pragma once

#include <pthread.h>

namespace robosense {

class Mutex {
public:
    Mutex() { pthread_mutex_init(&mu_, nullptr); }

    ~Mutex() { pthread_mutex_destroy(&mu_); }

    inline void Lock() { pthread_mutex_lock(&mu_); }

    inline void Unlock() { pthread_mutex_unlock(&mu_); }

    inline bool TryLock() { return pthread_mutex_trylock(&mu_) == 0; }

    Mutex(const Mutex &) = delete;
    Mutex &operator=(const Mutex &) = delete;

private:
    friend class CondVar;
    pthread_mutex_t mu_;
};

class MutexLock {
public:
    explicit MutexLock(Mutex *mu) : mu_(mu) { mu_->Lock(); }
    ~MutexLock() { mu_->Unlock(); }

    MutexLock(const MutexLock &) = delete;
    MutexLock &operator=(const MutexLock &) = delete;

private:
    Mutex *const mu_;
};

// Wrapper for pthread_cond_t
class CondVar {
public:
    CondVar() { pthread_cond_init(&cv_, nullptr); }
    ~CondVar() { pthread_cond_destroy(&cv_); }

    void Wait(Mutex *mu) { pthread_cond_wait(&cv_, &mu->mu_); }

    void Signal() { pthread_cond_signal(&cv_); }

    void Signalall() { pthread_cond_broadcast(&cv_); }

    CondVar(const CondVar &) = delete;
    CondVar &operator=(const CondVar &) = delete;

private:
    pthread_cond_t cv_;
};

class BlockingCounter {
public:
    explicit BlockingCounter(size_t cnt) : counter_(cnt) {}

    bool Decrement() {
        MutexLock lock(&mutex_);
        --counter_;

        if (counter_ == 0u) {
            cond_.Signalall();
        }

        return counter_ == 0u;
    }

    void Reset(size_t cnt) {
        MutexLock lock(&mutex_);
        counter_ = cnt;
    }

    void Wait() {
        MutexLock lock(&mutex_);

        while (counter_ != 0u) {
            cond_.Wait(&mutex_);
        }
    }

    BlockingCounter(const BlockingCounter &) = delete;
    BlockingCounter &operator=(const BlockingCounter &) = delete;

private:
    Mutex mutex_;
    CondVar cond_;
    size_t counter_;
};

class RwMutex {
public:
    RwMutex() { pthread_rwlock_init(&mu_, nullptr); }
    ~RwMutex() { pthread_rwlock_destroy(&mu_); }

    inline void ReaderLock() { pthread_rwlock_rdlock(&mu_); }
    inline void WriterLock() { pthread_rwlock_wrlock(&mu_); }

    inline void Unlock() { pthread_rwlock_unlock(&mu_); }

    RwMutex(const RwMutex &) = delete;
    RwMutex &operator=(const RwMutex &) = delete;

private:
    pthread_rwlock_t mu_;
};

class ReaderMutexLock {
public:
    explicit ReaderMutexLock(RwMutex *mu) : mu_(mu) { mu_->ReaderLock(); }
    ~ReaderMutexLock() { mu_->Unlock(); }

    ReaderMutexLock(const ReaderMutexLock &) = delete;
    ReaderMutexLock &operator=(const ReaderMutexLock &) = delete;

private:
    RwMutex *mu_ = nullptr;
};

class WriterMutexLock {
public:
    explicit WriterMutexLock(RwMutex *mu) : mu_(mu) { mu_->WriterLock(); }
    ~WriterMutexLock() { mu_->Unlock(); }

    WriterMutexLock(const WriterMutexLock &) = delete;
    WriterMutexLock &operator=(const WriterMutexLock &) = delete;

private:
    RwMutex *mu_ = nullptr;
};

} // namespace robosense
