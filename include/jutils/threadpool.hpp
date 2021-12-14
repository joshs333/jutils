#ifndef JUTILS_MULTIPROCESSING_HPP_
#define JUTILS_MULTIPROCESSING_HPP_

#include <queue>
#include <atomic>
#include <mutex>
#include <thread>
#include <memory>
#include <functional>

namespace jutils {

namespace _internal {

//! Operator to 
template<typename T>
struct QueueTop {
    T& operator()(const std::queue<T>& q) {
        return q.front();
    }
    T& operator()(const std::priority_queue<T>& q) {
        return q.top();
    }
};

} /* namespace _internal */

/**
 * @brief creates a threadpool to execute tasks
 * @tparam TaskArgs the argumets that go to executor functions
 * @tparam Queue the queue used internal to process tasks (can be a normal queue or priority)
 * @tparam QueueTop the operator to use to get the next task to process
 **/
template<
    typename Task,
    typename Queue = std::queue<Task>,
    typename TopOperator = _internal::QueueTop<Task>
>
class ThreadPool {
public:
    //! Executor for simple short tasks that don't need a kill signal
    typedef std::function<void(const Task&)> TaskExecutor;
    //! Executor for long tasks that can access the kill signal
    typedef std::function<void(const Task&, const std::atomic_bool&)> LongTaskExecutor;

    /**
     * @brief constructs a threadpool
     **/
    ThreadPool(int max_num_threads):
        max_num_threads_(max_num_threads)
    {}

    /**
     * @brief allows direct access to the internal queue object
     * @return the internal queue
     **/
    Queue& queue() {
        return queue_;
    }

    void execute(const std::vector<Task>& tasks) {
        for(const auto& t : tasks)
            execute(t);
    }

    void execute(const Task& task) {

    }


private:
    //! The number of worker threads 
    int max_num_threads_;
    //! The internal queue tracking tasks
    Queue queue_;
    //! 
    TopOperator queue_top_;
};



} /* namespace jutils */


#endif /* JUTILS_MULTIPROCESSING_HPP_ */