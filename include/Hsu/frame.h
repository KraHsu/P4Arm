#pragma once

#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <fmt/ostream.h>
#include <array>
#include <memory>
#include <mutex>
#include <string>
#include <units.h>

namespace Hsu {
namespace Types {
struct HomogeneousM;
struct RotationM;
struct TranslationM;

struct HomogeneousM {
  Eigen::Matrix4d value;

  HomogeneousM();

  HomogeneousM(Eigen::Matrix4d homogeneous);

  HomogeneousM(const RotationM& rotation, const TranslationM& translation);

  HomogeneousM(const RotationM& rotation);

  HomogeneousM(const TranslationM& translation);

  HomogeneousM inv();

  HomogeneousM operator*(HomogeneousM const& R);
};

struct RotationM {
  Eigen::Matrix3d value;

  RotationM();

  RotationM(Eigen::Matrix3d value);

  RotationM(const HomogeneousM& homogeneous);

  RotationM inv();

  RotationM operator*(RotationM const& R);

  Eigen::Vector3d to_euler();
};

struct TranslationM {
  Eigen::Vector3d value;

  TranslationM();

  TranslationM(Eigen::Vector3d value);

  TranslationM(const HomogeneousM& homogeneous);
};
}  // namespace Types

namespace Types {
struct PointV;

struct PointV {
  Eigen::Vector4d value;

  PointV(Eigen::Vector3d coordinate);
};
}  // namespace Types

class Frame;
class Point;

class Frame : public std::enable_shared_from_this<Frame> {
  using Sprt = std::shared_ptr<Frame>;

 private:
  struct PassKey {
    explicit PassKey() {}
  };

  // std::mutex mu_;
  std::recursive_mutex mu_;

  std::shared_ptr<Frame> ref_coor_;

  Hsu::Types::HomogeneousM homogeneous_;

 public:
  std::string name;

 private:
  static std::shared_ptr<Frame> create(std::string const& name, Hsu::Types::HomogeneousM const& homogeneous) {
    return std::make_shared<Frame>(PassKey(), name, homogeneous);
  }

 public:
  Frame(const Frame&) = delete;
  Frame& operator=(const Frame&) = delete;

  Frame(PassKey _, std::string const& name, Hsu::Types::HomogeneousM const& homogeneous);

  static std::shared_ptr<Frame> const WORLD_FRAME() {
    static auto res = create("World", Hsu::Types::HomogeneousM());

    res->ref_coor_ = res;

    return res;
  }

  Sprt define_frame(std::string const& name, Hsu::Types::HomogeneousM const& homogeneous);

  Sprt copy(std::string const& name);

  Sprt set_rotation(Hsu::Types::RotationM const& rotation);

  Sprt set_translation(Hsu::Types::TranslationM const& translation);

  Sprt set_homegeneous(Hsu::Types::HomogeneousM const& homegeneous);

  Hsu::Types::HomogeneousM get_homegeneous_relative_to(Sprt target);

  Sprt set_reference_coor(Sprt ref_coor);

  Sprt change_reference_coor(Sprt ref_coor);

  Sprt transform(Hsu::Types::HomogeneousM const& H, Sprt target);
};

class Point : public std::enable_shared_from_this<Point> {
  using Sprt = std::shared_ptr<Point>;

 private:
  struct PassKey {
    explicit PassKey() {}
  };

  // std::mutex mu_;
  std::recursive_mutex mu_;

  std::shared_ptr<Frame> ref_coor_;

  Hsu::Types::HomogeneousM homogeneous_;

 public:
  std::string name;

 private:
  // static std::shared_ptr<Frame> create(std::string const& name, Hsu::Types::HomogeneousM const& homogeneous) {
  //   return std::make_shared<Frame>(PassKey(), name, );
  // }

 public:
  // Frame(const Frame&) = delete;
  // Frame& operator=(const Frame&) = delete;

  // Frame(PassKey _, std::string const& name, Hsu::Types::HomogeneousM const& homogeneous);

  // static std::shared_ptr<Frame> const WORLD_FRAME() {
  //   static auto res = create("World", Hsu::Types::HomogeneousM());

  //   res->ref_coor_ = res;

  //   return res;
  // }

  // Sprt define_frame(std::string const& name, Hsu::Types::HomogeneousM const& homogeneous);

  // Sprt copy(std::string const& name);

  // Sprt set_rotation(Hsu::Types::RotationM const& rotation);

  // Sprt set_translation(Hsu::Types::TranslationM const& translation);

  // Hsu::Types::HomogeneousM get_homegeneous_relative_to(Sprt target);

  // Sprt set_reference_coor(Sprt ref_coor);

  // Sprt change_reference_coor(Sprt ref_coor);

  // Sprt transform(Hsu::Types::HomogeneousM const& H, Sprt target);
};

class Vector {};

class DirectedSegment {};
}  // namespace Hsu

#if defined(HSU_FRAME_VISUAL)
#include <pybind11/attr.h>
#include <pybind11/embed.h>  // 引入pybind11嵌入头文件
#include <pybind11/numpy.h>
#include <pybind11/pytypes.h>
#include <thread>

namespace Hsu {
template <typename T, int Rows, int Cols>
pybind11::array_t<T> eigen_to_numpy(const Eigen::Matrix<T, Rows, Cols>& mat) {
  return pybind11::array_t<T>({Rows, Cols}, {sizeof(T), sizeof(T) * Rows}, mat.data());
}

std::string get_current_executor_path();

class Frame3DScene {
 private:
  struct Passkey {
    explicit Passkey() {}
  };

  enum Status { STOP, READY, START, RUNNING };

  Status state_{STOP};

  // std::vector<Frame> frame_list;

  std::array<Types::HomogeneousM, 8> arm_left_;
  std::array<Types::HomogeneousM, 8> arm_right_;

  std::thread* main_;

 public:
  Frame3DScene(Frame3DScene const&) = delete;
  Frame3DScene& operator=(Frame3DScene const&) = delete;

  Frame3DScene(Passkey);

  static Frame3DScene& instance() {
    static Frame3DScene ins(Passkey{});
    return ins;
  }

  Frame3DScene& begin();

  Frame3DScene& set_arm_l_data(std::array<Types::HomogeneousM, 8> const& data);
  Frame3DScene& set_arm_r_data(std::array<Types::HomogeneousM, 8> const& data);

  Frame3DScene& start();

  Frame3DScene& stop();
};
}  // namespace Hsu
#endif