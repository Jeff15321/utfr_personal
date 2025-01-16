#include <iostream>

int t1() {
    const int b = 2;
    int a = 1;
    std::cout << a + b<< std::endl;
    return 0;
}

int main() {
    t1();
    return 0;
}

