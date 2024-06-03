#pragma once
#include <iostream>

template <typename T>
class Queue {
    T* queue;
    int capacity;
    int front;
    int last;
    int size;

public:
    Queue(int cap = 1) {
        capacity = cap;
        queue = new T[capacity];
        front = 0;
        last = -1;
        size = 0;
    }

    void push(T& x) {
        if (size == capacity) {
            capacity *= 2;
            T* temp = new T[capacity];
            for (int i = 0; i < size; i++) {
                temp[i] = queue[(front + i) % size];
            }
            delete[] queue;
            queue = temp;
            front = 0;
            last = size - 1;
        }
        last = (last + 1) % capacity;
        queue[last] = x;
        size++;
    }

    T& frontElement(){
        if (!empty())
            return queue[front];
        throw std::runtime_error("Queue is empty");
    }

    const T& frontElement() const {
        if (!empty())
            return queue[front];
        throw std::runtime_error("Queue is empty");
    }

    void pop() {
        if (empty()) {
            return;
        }

        front = (front + 1) % capacity;
        size--;
    }

    bool empty() const {
        return size == 0;
    }

    ~Queue() {
        delete[] queue;
    }
};
