//
// Created by Sebastian on 15.01.2019.
//

#ifndef HEAP_BVH_BOOST_H
#define HEAP_BVH_BOOST_H

#include <cmath>
#include <cstdint>

const uint8_t tab64[64] = {
        63, 0, 58, 1, 59, 47, 53, 2,
        60, 39, 48, 27, 54, 33, 42, 3,
        61, 51, 37, 40, 49, 18, 28, 20,
        55, 30, 34, 11, 43, 14, 22, 4,
        62, 57, 46, 52, 38, 26, 32, 41,
        50, 36, 17, 19, 29, 10, 13, 21,
        56, 45, 25, 31, 35, 16, 9, 12,
        44, 24, 15, 8, 23, 7, 6, 5};

// A faster implementation than the one found in the standard libraries
uint8_t log2_64(uint64_t value) {
    value |= value >> 1;
    value |= value >> 2;
    value |= value >> 4;
    value |= value >> 8;
    value |= value >> 16;
    value |= value >> 32;
    return tab64[((uint64_t) ((value - (value >> 1)) * 0x07EDD5E59A4E28C2)) >> 58];
}

// A utility function to swap two elements
void swap(double_t *a, double_t *b) {
    double_t t = *a;
    *a = *b;
    *b = t;
}

void swap(uint64_t *a, uint64_t *b) {
    uint64_t t = *a;
    *a = *b;
    *b = t;
}

#endif //HEAP_BVH_BOOST_H
