#include <iostream>

class Base {
 public:
  virtual void Init() = 0;

 protected:
  bool is_init_ = false;
};

class Derived : public Base {
    public:
    virtual void Init() override {
        is_init_ = true;
    }

    int num = 10;
};

int main() {
    Base* p = new Derived();
    Derived* p1 = dynamic_cast<Derived*>(p);
    p1->num;


}