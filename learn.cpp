#include <iostream>

class Student {
    public:
        int age;
        std::string name;
        Student(int age, std::string name) {
            this->age = age;
            this->name = name;
        }
        void myMethod() {
            std::cout << this->age << std::endl;
        }
        void outsideMethod();

};

void Student::outsideMethod() {
    std::cout << this->name << std::endl;
};

int t2() {
    Student s1(10, "John");
    s1.myMethod();
    s1.outsideMethod();

    Student s2(11, "Jane");
    s2.myMethod();
    s2.outsideMethod();

    std::cout << s1.age << std::endl;
    std::cout << s2.age << std::endl;
}

int t1() {
    int a = 5; //*b finds the value stored at the location stored
    int* b = &a; //int* tells it is a pointer, pointer stores the address of &a
    int &c = a; //& used as variabled assigned to creates an alias
                //& used when assigned to creates the variable address

    *b = 10; //* is required when updating the pointer address/value
    std::cout << b << std::endl;
    std::cout << c << std::endl;
    

    struct Pair{
        int a;
        std::string b;
        
    } myPair;

    myPair.a = 1;
    myPair.b = "hello";

    // std:: cout << myPair.a << std::endl;

    for (int i = 0; i < 10; ++i) {
        // std::cout << i << std::endl;
    }

    int list[] = {1, 2, 3, 4, 5};

}

int main() {
    t2();
    return 0;
}

