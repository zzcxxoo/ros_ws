#pragma once
/******************************************************************************
 @copyright Copyright <JT-Innovation> (c) 2022
******************************************************************************
File Name     : JtcxSecurityQueue.hpp
Version       : v1.0.0
Author        : Jianqiang 
Created       : 2022/10/11
Last Modified :
Description   : Security Queue
Function List :
History       :  first edition ----20221011
******************************************************************************/

#include <thread>
#include <mutex>
#include <atomic>
#include <unistd.h>
#include <condition_variable>
#include <deque>

template <typename Dtype>
class SecurityQueue{
    public:
    
        explicit SecurityQueue(int size):size_(size){};;

        SecurityQueue(const SecurityQueue & other);

        SecurityQueue(SecurityQueue &&) = delete;

        SecurityQueue & operator= (const SecurityQueue &) = delete;

        ~SecurityQueue(){};

        /// @brief Populate queue
        /// @param element 
        /// @return 
        bool tryPush(const Dtype& element);

        /// @brief push with mutex_ and cv_
        /// @param element 
        /// @return 
        bool push(const Dtype& element);

        /// @brief pop from back with mutex_ and cv_
        /// @return 
        Dtype popBack();

        /// @brief pop from front with mutex_ and cv_
        /// @return 
        Dtype popFront();

        /// @return queue is empty or not
        bool empty() const;

        /// @brief check up the back element
        /// @return 
        Dtype checkUpBack();

        /// @brief check up the back element
        /// @return 
        Dtype checkUpFront();

    private:
        const int size_;
        std::mutex mutex_;
        std::deque<Dtype> queue_;
        std::condition_variable cv_;
};

template <typename Dtype>
bool SecurityQueue<Dtype>::tryPush(const Dtype& element){
        bool success= true;
        std::unique_lock<std::mutex> lock(mutex_);
        if(queue_.size()<size_)
        {
            queue_.push_front(element);
            cv_.notify_one();
        }
        else
        {
            success=false;
        }
        return success;
}
template <typename Dtype>
bool SecurityQueue<Dtype>::push(const Dtype& element)
{
        std::unique_lock<std::mutex>lock(mutex_);
        if(queue_.size()==size_)
        {
            queue_.pop_back();
        }
        queue_.push_front(std::move(element));
        cv_.notify_one();
        return true;
}
template <typename Dtype>
Dtype SecurityQueue<Dtype>::popBack()
{
        std::unique_lock<std::mutex>lock(mutex_);
        cv_.wait(lock,[&]{return !this->queue_.empty();});
     
        Dtype element(std::move(queue_.back()));
        queue_.pop_back();
        return element;
}

template <typename Dtype>
Dtype SecurityQueue<Dtype>::popFront()
{
        std::unique_lock<std::mutex>lock(mutex_);
        cv_.wait(lock,[&]{return !this->queue_.empty();});
     
        Dtype element(std::move(queue_.front()));
        queue_.pop_front();
        return element;
}

template <typename Dtype>
bool SecurityQueue<Dtype>::empty() const
{
        return queue_.empty();
}

/// @brief Judge whether it is empty before use

template <typename Dtype>
Dtype SecurityQueue<Dtype>::checkUpBack()
{       
        std::unique_lock<std::mutex>lock(mutex_);
        return this->queue_.back();    
}

template <typename Dtype>
Dtype SecurityQueue<Dtype>::checkUpFront()
{       
        std::unique_lock<std::mutex>lock(mutex_);
        return this->queue_.front();    
}