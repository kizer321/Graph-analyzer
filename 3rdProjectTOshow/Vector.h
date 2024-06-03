#pragma once
#include <stdexcept> 

template <typename T>
class Vector {
private:
    T* wektor;
    int capacity; // total storage capacity of the vector
    int current; // number of current elements in vector

public:
    explicit Vector(int initializationSize = 0){
        capacity = 1;
        wektor = new T[capacity];
        current = initializationSize;
    }

    Vector(const Vector& other) {
        capacity = other.getCapacity();
        current = other.size();
        wektor = new T[capacity];
        for (int i = 0; i < current; i++) {
            wektor[i] = other.wektor[i];
        }
    }

    Vector(const Vector&& other) noexcept : wektor(other.wektor), current(other.current), capacity(other.capacity){
        other.current = 0;
        other.capacity = 0;
        other.wektor = nullptr;
    }

    void reserve(int newCapacity) {
        if (newCapacity > capacity) {
            T* temp = new T[newCapacity];
            for (int i = 0; i < current; i++) {
                temp[i] = wektor[i];
            }
            delete[] wektor;
            wektor = temp;
            capacity = newCapacity;
        }
    }

    void fill(const T& value, int count) {
        if (count > capacity) {
            reserve(count);
        }
        for (int i = 0; i < count; i++) {
            wektor[i] = value;
        }
        current = count;
    }

    void push(const T& data) {
        if (current >= capacity) {
            reserve(capacity > 0 ? 2 * capacity : 1);
        }
        wektor[current] = data;
        current++;
    }

    void push(const T& data, int index) {
        if (index >= capacity) {
            reserve(index + 1);
        }
        wektor[index] = data;
        if (index >= current) {
            current = index + 1;
        }
    }

    T &get(int index)  {
        if (index < 0 || index >= current)
            throw std::out_of_range("Index out of range");
        return wektor[index];
        
    }

    const T& get(int index) const {
        if (index < 0 || index >= current)
            throw std::out_of_range("Index out of range");
        return wektor[index];
    }

    void pop() {
        current--;
    }

    int size() const {
        return current;
    }

    int getCapacity() const {
        return capacity;
    }

    ~Vector() {
        if (wektor != nullptr)
            delete[] wektor;
    }

    bool empty() const {
        return current == 0;
    }

    Vector& operator=(const Vector& other) {
        if (this == &other)
            return *this;
        delete[] wektor;
        current = other.current;
        capacity = other.capacity;
        wektor = new T[capacity];
        for (int i = 0; i < current; i++) {
            wektor[i] = other.wektor[i];
        }
        return *this;
    }

    T& operator[](int index) {
        if (index < 0 || index >= current) 
            throw std::out_of_range("Index out of range");
        return wektor[index];
    }

    const T& operator[](int index) const {
        if (index < 0 || index >= current) 
            throw std::out_of_range("Index out of range");
        return wektor[index];
    }
};
