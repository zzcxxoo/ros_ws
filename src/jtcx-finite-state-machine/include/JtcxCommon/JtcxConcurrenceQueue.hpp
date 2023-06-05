#pragma once
/******************************************************************************
 Copyright (C), 2022-2032, JTCX Inc.
******************************************************************************
File Name     : ExternalEventsFactory
Version       : v1.0.0
Author        : wang jian qiang 
Created       : 2022/10/11
Last Modified :
Description   : Factory
Function List :
History       : 20221011_v1.0.0
******************************************************************************/

#include <thread>
#include <mutex>
#include <atomic>
#include <unistd.h>
#include <condition_variable>
#include <deque>

template <typename Dtype>
class ConcurrenceQueue{
    public:
    
        explicit ConcurrenceQueue(int size):size_(size){};;

        ConcurrenceQueue(const ConcurrenceQueue & other);

        ConcurrenceQueue(ConcurrenceQueue &&) = delete;

        ConcurrenceQueue & operator= (const ConcurrenceQueue &) = delete;

        ~ConcurrenceQueue();
        /********************************
         * Populate queue
         *******************************/
        bool try_push(const Dtype& element);
        /*******************************
         * push with mutex_ and cv_
         *******************************/
        bool push(const Dtype& element);
        /*******************************
         * pop with mutex_ and cv_
         *******************************/
        Dtype pop();
        /*******************************
         * whether the queue is empty
         *******************************/
        bool empty() const;
        /*******************************
         * check up the first element
         *******************************/
        Dtype check_up();

    private:
        const int size_;
        std::mutex mutex_;
        std::deque<Dtype> queue_;
        std::condition_variable cv_;
};

template <typename Dtype>
bool ConcurrenceQueue<Dtype>::try_push(const Dtype& element){
        bool success= true;
        std::unique_lock<std::mutex> lock(mutex_);
        if(queue_.size()<size_)
        {
            queue_.push_front(element);
            lock.unlock();
            cv_.notify_one();
        }
        else
        {
            success=false;
        }
        return success;
}
template <typename Dtype>
bool ConcurrenceQueue<Dtype>::push(const Dtype& element)
{
        std::unique_lock<std::mutex>lock(mutex_);
        if(queue_.size()==size_)
        {
            queue_.pop_back();
        }
        queue_.push_front(std::move(element));
        lock.unlock();
        cv_.notify_one();
        return true;
}
template <typename Dtype>
Dtype ConcurrenceQueue<Dtype>::pop()
{
        std::unique_lock<std::mutex>lock(mutex_);
        cv_.wait(lock,[&]{return !this->queue_.empty();});
     
        Dtype element(std::move(queue_.back()));
        queue_.pop_back();
        return element;
}

template <typename Dtype>
bool ConcurrenceQueue<Dtype>::empty() const
{
        return queue_.empty();
}

template <typename Dtype>
Dtype ConcurrenceQueue<Dtype>::check_up()
{
        std::unique_lock<std::mutex>lock(mutex_);
        cv_.wait(lock,[&]{return !this->queue_.empty();});
        return this->queue_.back();
}