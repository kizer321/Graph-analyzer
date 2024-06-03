#pragma once
#include <iostream>

template <typename T>
class Node {
public:
    Node* link;
    T data;

    Node(T value) {
        data = value;
        link = nullptr;
    }
};

template <typename T>
class Stack {
private:
    Node<T>* top;

public:
    Stack() { top = nullptr; }

    void push(T value) {
        Node<T>* newNode = new Node<T>(value);
        newNode->link = top;
        top = newNode;
    }

    bool const isEmpty() const {
        return top == nullptr;
    }

    T peek() const {
        if (!isEmpty()) return top->data;
        else throw std::runtime_error("Stack is empty");
    }

    T pop() {
        if (isEmpty()) {
            std::cerr << "Stack Underflow" << std::endl;
            exit(1);
        }
        Node<T>* temp = top;
        T pop = temp->data;
        top = top->link;
        delete temp;
        return pop;
    }
};
