#include <stdint.h>
#include <string.h>

#define FREE 0
#define FULL 1

using namespace std;

class DataFilter {
   public:
    // Construtor
    DataFilter(uint8_t size) {
        size_ = size;
        it_ini_ = 0;
        it_end_ = 0;
        q_status_ = FREE;
    }

    void queue_insert_data(int data) {
        if (q_status_ == FULL) {
            it_ini_++;
            if (it_ini_ == size_) it_ini_ = 0;
        }
        data_[it_end_] = data;
        it_end_++;

        if (it_end_ == size_) it_end_ = 0;

        if (it_end_ == it_ini_) {
            q_status_ = FULL;
        }
    };

    void swap(int *p, int *q) {
        uint16_t t;
        t = *p;
        *p = *q;
        *q = t;
    }

    void sort_array(int array[], uint8_t n) {
        int i, j;
        for (i = 0; i < n - 1; i++) {
            for (j = 0; j < n - i - 1; j++) {
                if (array[j] > array[j + 1]) swap(&array[j], &array[j + 1]);
            }
        }
    }

    int queue_median(uint8_t discard) {
        uint32_t mean = 0;
        int data[size_];
        uint8_t i;
        if (q_status_ == FULL) {
            memcpy(&data, &data_, sizeof(data));
            sort_array(data, size_);
            for (i = discard; i < (size_ - discard);
                 i++) {  // discard the first two and the last two
                mean += data[i];
            }
            return (uint16_t)(mean / (size_ - (discard * 2)));
        }
        for (i = it_ini_; i < it_end_; i++) {
            mean += data_[i];
        }
        return (uint16_t)(mean / i);
    }

    int data_[16];
    uint8_t size_;
    uint8_t it_ini_;
    uint8_t it_end_;
    uint8_t q_status_;
};