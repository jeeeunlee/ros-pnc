#pragma once

// #include <chrono>

// class Clock {
//     public:
//         Clock(){}
//         ~Clock(){}

//         void start() { ini_time_ = std::chrono::high_resolution_clock::now(); }
//         // return in milliseconds
//         double stop() {
//             end_time_ = std::chrono::high_resolution_clock::now();
//             duration_ = std::chrono::duration_cast<std::chrono::microseconds>(end_time_-ini_time_);
//             return double(duration_.count())*1e-3;
//         }

//     private:
//         std::chrono::microseconds duration_;
//         std::chrono::high_resolution_clock::time_point ini_time_, end_time_;
// };

#include <time.h>
class Timer {
    public:
        Timer() {start_= clock();}
        ~Timer(){}

        void start(){start_= clock();}
        double stop(){ 
            temp_= clock(); 
            double dt = ((double) (temp_ - start_)) / CLOCKS_PER_SEC; 
            start_ = temp_;
            return dt;
        }
        double check(){ 
            temp_= clock(); 
            double dt = ((double) (temp_ - start_)) / CLOCKS_PER_SEC; 
            return dt;
        }
    private:
        clock_t start_;
        clock_t temp_;

};