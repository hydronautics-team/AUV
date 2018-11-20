#ifndef AUV_VISION_CONCURRENTQUEUE_H
#define AUV_VISION_CONCURRENTQUEUE_H

#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>

/**
 * A concurrent wrap above STL Queue.
 * Implemented in .h file due to limitations of
 * CMake and templates.
 */
template <typename T>
class ConcurrentQueue {

private:

    std::queue<T> queue;
    std::mutex mutex;
    std::condition_variable cond;

public:

    ConcurrentQueue() = default;
    ConcurrentQueue(const ConcurrentQueue&) = delete;
    ConcurrentQueue& operator=(const ConcurrentQueue&) = delete;

    T pop();
    void pop(T& item);
    void push(const T& item);
    void clear();

};

template <typename T>
T ConcurrentQueue<T>::pop() {
    std::unique_lock<std::mutex> mlock(mutex);
    while (queue.empty()) {
        cond.wait(mlock);
    }
    auto val = queue.front();
    queue.pop();
    return val;
}

template <typename T>
void ConcurrentQueue<T>::pop(T &item) {
    std::unique_lock<std::mutex> mlock(mutex);
    while (queue.empty()) {
        cond.wait(mlock);
    }
    item = queue.front();
    queue.pop();
}

template <typename T>
void ConcurrentQueue<T>::push(const T &item) {
    std::unique_lock<std::mutex> mlock(mutex);
    queue.push(item);
    mlock.unlock();
    cond.notify_one();
}

template <typename T>
void ConcurrentQueue<T>::clear() {
    std::unique_lock<std::mutex> mlock(mutex);
    std::queue<T> emptyQueue;
    std::swap(queue, emptyQueue);
    mlock.unlock();
}


#endif //AUV_VISION_CONCURRENTQUEUE_H
