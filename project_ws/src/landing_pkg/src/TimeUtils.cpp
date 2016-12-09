//
// Created by francesco on 07/12/16.
//

#include "headers/TimeUtils.h"

class TimeUtils {
public:
    int hello;
    static long timediff(clock_t t1, clock_t t2){
        long elapsed;
        elapsed = ((double)t2 - t1) / CLOCKS_PER_SEC * 1000;
        return elapsed;
    }
};