/**
 * @brief wraps std::chrono to provide a few more convenient functions
 *  also provides a timer class
 * @date 12/05/2021
 * @author Joshua Spisak <jspisak@andrew.cmu.edu>
 * 
 * Copyright 2021 Joshua Spisak
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 **/
#ifndef JUTILS_TIME_HPP_
#define JUTILS_TIME_HPP_
#define JUTILS_TIME_VERSION_MAJOR 0
#define JUTILS_TIME_VERSION_MINOR 1

#include <chrono>
#include <iomanip>
#include <ostream>
#include <thread>
#include <mutex>
#include <functional>

//! Automatically enable the hour / millisecond / etc... literals
using namespace std::chrono_literals;

namespace jutils {

/**
 * @brief extends the behavior of a chrono time_point
 *  to add convenient functions
 * @tparam clock the base chrono clock to use
 **/
template<typename clock = std::chrono::high_resolution_clock>
class extended_time : public std::chrono::time_point<clock> {
public:
    //! The type this is extended from
    typedef std::chrono::time_point<clock> time_point;

    //! Construct an extended_time from a time_point
    extended_time(const time_point& time):
        time_point(time)
    {};

    // //! Converts from a time point to a double
    double seconds() const {
        return static_cast<double>(this->time_since_epoch().count())
                * clock::period::num 
                / clock::period::den;
    }

    //! Converts from a time point to a long double
    long double lseconds() const {
        return static_cast<long double>(this->time_since_epoch().count())
                * clock::period::num 
                / clock::period::den;
    }
    
    //! Allows addition to durations, returning an extended_time
    template<typename duration = std::chrono::milliseconds>
    extended_time operator+(const duration& d) {
        return extended_time(static_cast<time_point>(*this) + d);
    }

    //! Format this time_point into a string
    std::string to_string() const {
        auto ls = this->time_since_epoch().count();
        auto secs = ls / clock::period::den;
        auto nsecs = ls - secs * clock::period::den;
        int size_s = std::snprintf( nullptr, 0, "%ld.%09ld", secs, nsecs) + 1;
        auto buf = std::make_unique<char[]>(size_s);
        std::snprintf( buf.get(), size_s, "%ld.%09ld", secs, nsecs);
        return std::string( buf.get(), buf.get() + size_s - 1 );
    }

    ///! Allows sending it into an ostream
    friend std::ostream& operator<<(
        std::ostream& out,
        const extended_time& t
    ) {
        auto ls = t.time_since_epoch().count();
        auto secs = ls / clock::period::den;
        auto nsecs = ls - secs * clock::period::den;

        if(std::abs(secs) > 0) {
            out << secs;
        } else {
            if(nsecs < 0) {
                out << "-0";
            } else {
                out << "0";
            }
        }
        out << "." << std::setfill('0') << std::setw(9) << std::abs(nsecs);
        return out;
    }
};

/**
 * @brief extends the behavior of a chrono clock
 *  to simplify usage and to return extended_time points
 * @tparam clock the base chrono clock to extend
 **/
template<typename clock = std::chrono::high_resolution_clock>
class extended_clock : public clock {
public:
    //! The extended_time point equivalent to this clock
    typedef extended_time<clock> Time;

    //! Returns the current time as an extended point
    static Time now() {
        return Time(clock::now());
    } 
};

//! The extended_clock equivelent of the system clock
typedef extended_clock<std::chrono::system_clock> Clock;
//! The extended_clock equivelent of the steady clock (read: monotonic)
typedef extended_clock<std::chrono::steady_clock> MonotonicClock;

/**
 * @brief sleeps for some duration
 * @param duration a chrono duration to sleep for
 **/
template<typename UT, typename RatioT>
void sleep(std::chrono::duration<UT, RatioT> duration) {
    std::this_thread::sleep_for(duration);
}

/**
 * @brief sleeps for some duration
 * @param seconds the duration to sleep for specified in seconds
 **/
void sleep(double seconds) {
    sleep(std::chrono::microseconds(static_cast<long int>(seconds * 1e6)));
}

/**
 * @brief a simple templated timer
 * @tparam clock_t the underlying clock to use to check sleep lengths
 * @tparam duration_t the duration type to use (determines precision)
 **/
template<
    typename clock_t = std::chrono::high_resolution_clock,
    typename duration_t = std::chrono::microseconds
>
class Timer {
public:
    //! The extend_clock type for the underlying chrono clock
    typedef extended_clock<clock_t> Clock;
    //! The extend_time type for the underlying clock
    typedef extended_time<clock_t> Time;
    //! The duration type used internally (determines precision)
    typedef duration_t Duration;

    //! Create a timer with no period defined yet
    Timer():
        period_set_(false),
        align_time_set_(false),
        thread_running_(false)
    {}

    //! Create a timer with a period defined by a chrono duration
    template<typename UT, typename RatioT>
    Timer(std::chrono::duration<UT, RatioT> period):
        thread_running_(false)
    {
        setPeriod(period);
    }

    //! Create a timer with a period defined in seconds as a double
    Timer(double period):
        thread_running_(false)
    {
        setPeriod(period); 
    }

    //! Set the period from a double
    void setPeriod(double seconds) {
        period_ = duration_t(static_cast<long int>(seconds * duration_t::period::den));
        align_time_set_ = false;
        period_set_ = true;
    }

    //! Set the period from a chrono duration
    template<typename UT, typename RatioT>
    void setPeriod(std::chrono::duration<UT, RatioT> period) {
        period_ = std::chrono::duration_cast<duration_t>(period);
        align_time_set_ = false;
        period_set_ = true;
    }

    /**
     * @brief sleeps the period of the timer
     * @param[in] reset whether or not to reset the timer to begin now
     *  otherwise it will try to remain period-aligned with the first time
     *  it's called
     * @note if the clock jumps or a loop takes too long it will reset the
     *  alignment time.
     **/
    void sleep(bool reset = false) {
        if(!period_set_) {
            throw std::runtime_error("running timer without a period being set");
        }

        if(reset || !align_time_set_ 
            || Clock::now() > align_time_ + 10*period_ // Clock jumps forward
            || Clock::now() < align_time_ - 2*period_ // Clock jumps backward
        ) {
            align_time_ = Clock::now() + period_;
            align_time_set_ = true;
        }
        auto sleep_duration = align_time_ - Clock::now();
        // If the sleep duration is greater than 0 that means
        // we still have time to go 
        if(sleep_duration > 0ns) {
            if(sleep_duration > period_) {
                jutils::sleep(period_);
            } else {
                jutils::sleep(sleep_duration);
                align_time_ += period_;
            }
        } else {
            while(Clock::now() + 5us > align_time_) {
                align_time_ += period_;
            }
        }
    }

    /**
     * @brief spawns a thread to execute a function on this timer
     * @param[in] f the function to execute until it returns fals
     *  or the thread receives a kill signal
     **/
    void spawn_executor(const std::function<bool()>& f) {
        thread_running_mutex_.lock();
        if(thread_running_) {
            thread_running_mutex_.unlock();
            return;
        }
        kill_signal_mutex_.lock();
        kill_signal_ = false;
        thread_running_ = true;

        executor_ = std::make_shared<std::thread>([this, f](void) {
            while(true) {
                kill_signal_mutex_.lock();
                if(kill_signal_) {
                    kill_signal_mutex_.unlock();
                    break;
                }
                kill_signal_mutex_.unlock();
                if(!f()) {
                    break;
                }
                sleep();
            }
        });

        kill_signal_mutex_.unlock();
        thread_running_mutex_.unlock();
    }

    void stop_executor() {
        // make sure there is a running thread
        thread_running_mutex_.lock();
        if(!thread_running_) {
            thread_running_mutex_.unlock();
            return;
        }

        // Send the kill signal
        kill_signal_mutex_.lock();
        kill_signal_ = true;
        kill_signal_mutex_.unlock();

        executor_->join();
        thread_running_ = false;
        thread_running_mutex_.unlock();
    }



private:
    //! Whether or not the period is set
    bool period_set_;
    //! Whether or not the alignment time is set
    bool align_time_set_;
    //! The period of the timer (should sleep for this amount)
    std::chrono::microseconds period_;
    //! Each wake should be period aligned to this time
    std::chrono::time_point<clock_t> align_time_;
    
    //! Whether or not we are running a thread
    bool thread_running_;
    //! Protect the signal
    std::mutex thread_running_mutex_; 
    //! If we are spawning an executor we use this to signal to stop
    bool kill_signal_;
    //! Protect the signal
    std::mutex kill_signal_mutex_;
    //! The thread itself
    std::shared_ptr<std::thread> executor_;
}; /* class Timer */

//! A timer on the system clock
typedef Timer<std::chrono::system_clock> SystemTimer;
//! A timer on the monotonic clock (even if system time jumps)
//! This timer should remain monotonic (maintain period)
typedef Timer<std::chrono::steady_clock> MonotonicTimer;


} /* namespace jutils */


#endif /* JUTILS_TIME_HPP_ */
