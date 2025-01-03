#include <exception>
#include <iostream>
#include <memory>

class pr;
class cus;

class interface {
 public:
  virtual ~interface() = default;

  virtual void work(int i) = 0;
};

class interface_imp : public interface {
 private:
  std::weak_ptr<pr> pr_;

 public:
  ~interface_imp() {
    std::cout << "接口实例被销毁了" << std::endl;
    return;
  }

  interface_imp(std::shared_ptr<pr> pr) : pr_(pr) {
    std::cout << "接口实例被生产出来了" << std::endl;
    return;
  }

  void work(int i) override {
    std::cout << "工作在 " << i << std::endl;
    return;
  }
};

class pr : public std::enable_shared_from_this<pr> {
 public:
  pr(int x) : x_(x) { std::cout << "生产者被创建\n"; }

  ~pr() { std::cout << "生产者被销毁\n"; }

  std::weak_ptr<interface> produce() {
    interface_ = std::make_shared<interface_imp>(shared_from_this());  // 使用 shared_from_this()
    return std::weak_ptr<interface>(interface_);
  }

 private:
  int x_;

  std::shared_ptr<interface_imp> interface_;
};

class cus {
 private:
  std::weak_ptr<interface> interface_;

 public:
  cus(std::weak_ptr<interface> interface) : interface_(interface) {
    std::cout << "消费者被创建了" << std::endl;
    return;
  }

  ~cus() {
    std::cout << "消费者被销毁了" << std::endl;
    return;
  }

 public:
  void work(int i) {
    if (auto shared = interface_.lock()) {
      shared->work(i);
    } else {
      std::cout << "接口不可用" << std::endl;
    }
  }
};

int main() {
  auto p = std::make_shared<pr>(3);

  auto i = p->produce();
  auto c = std::make_shared<cus>(i);

  c->work(1);
  p.reset();
  c->work(2);

  // c.reset();

  c->work(3);

  c->work(4);

  return 0;
}
