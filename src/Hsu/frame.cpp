#include <Eigen/Dense>
#include <Hsu/frame.h>
#include <fmt/core.h>
#include <fmt/ostream.h>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>

using namespace Hsu;
using namespace Hsu::Types;

// ---- private ----
Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd& mat, double tol = 1e-9) {
  // SVD 分解
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(mat, Eigen::ComputeThinU | Eigen::ComputeThinV);

  // 提取奇异值
  Eigen::VectorXd singularValues = svd.singularValues();  // 奇异值是向量
  Eigen::MatrixXd singularValuesInv =
      Eigen::MatrixXd::Zero(svd.matrixV().cols(), svd.matrixU().rows());  // 确定伪逆奇异值矩阵的维度

  // 计算伪逆的奇异值矩阵
  for (int i = 0; i < singularValues.size(); ++i) {
    if (singularValues(i) > tol) {  // 对非零奇异值取倒数
      singularValuesInv(i, i) = 1.0 / singularValues(i);
    }
  }

  // 计算伪逆矩阵 A^+ = V * Σ^+ * U^T
  return svd.matrixV() * singularValuesInv * svd.matrixU().transpose();
}

// ---- Hsu::Types RTH ----

HomogeneousM::HomogeneousM() {
  value.setIdentity();
  return;
}

HomogeneousM::HomogeneousM(Eigen::Matrix4d homogeneous) : value(homogeneous) {}

HomogeneousM::HomogeneousM(const RotationM& rotation, const TranslationM& translation) {
  value.setIdentity();
  value.block<3, 3>(0, 0) = rotation.value;
  value.block<3, 1>(0, 3) = translation.value;
  return;
}

HomogeneousM::HomogeneousM(const RotationM& rotation) {
  value.setIdentity();
  value.block<3, 3>(0, 0) = rotation.value;
  return;
}

HomogeneousM::HomogeneousM(const TranslationM& translation) {
  value.setIdentity();
  value.block<3, 1>(0, 3) = translation.value;
  return;
}

HomogeneousM HomogeneousM::inv() {
  HomogeneousM result;
  result.value = pseudoInverse(value);  // 调用伪逆计算函数
  return result;
}

HomogeneousM HomogeneousM::operator*(HomogeneousM const& R) {
  auto res = HomogeneousM(value * R.value);
  return res;
}

RotationM::RotationM() {
  value.setIdentity();
  return;
}
RotationM::RotationM(Eigen::Matrix3d value) : value(value) {}

RotationM::RotationM(const HomogeneousM& homogeneous) {
  value = homogeneous.value.block<3, 3>(0, 0);
  return;
}

RotationM RotationM::inv() {
  RotationM result;
  result.value = pseudoInverse(value);
  return result;
}

RotationM RotationM::operator*(RotationM const& R) {
  auto res = value * R.value;
  return RotationM(res);
}

TranslationM::TranslationM() {
  value.setZero();
  return;
}
TranslationM::TranslationM(Eigen::Vector3d value) : value(value) {}

TranslationM::TranslationM(const HomogeneousM& homogeneous) {
  value = homogeneous.value.block<3, 1>(0, 3);
  return;
}

// ---- Hsu Frame ----

using Sprt = std::shared_ptr<Frame>;

Frame::Frame(Frame::PassKey _, std::string const& name, Hsu::Types::HomogeneousM const& homogeneous)
    : name(name), homogeneous_(homogeneous) {}

Sprt Frame::define_frame(std::string const& name, Hsu::Types::HomogeneousM const& homogeneous) {
  std::lock_guard<std::recursive_mutex> _{mu_};
  auto res = create(name, homogeneous);
  res->ref_coor_ = shared_from_this();

  return res;
}

Sprt Frame::copy(std::string const& name) { return ref_coor_->define_frame(name, homogeneous_); }

Sprt Frame::set_rotation(Hsu::Types::RotationM const& rotation) {
  std::lock_guard<std::recursive_mutex> _{mu_};
  homogeneous_.value.block<3, 3>(0, 0) = rotation.value;

  return shared_from_this();
}
Sprt Frame::set_translation(Hsu::Types::TranslationM const& translation) {
  std::lock_guard<std::recursive_mutex> _{mu_};
  homogeneous_.value.block<3, 1>(0, 3) = translation.value;

  return shared_from_this();
}

Sprt Frame::set_homegeneous(Hsu::Types::HomogeneousM const& homegeneous) {
  std::lock_guard<std::recursive_mutex> _{mu_};
  homogeneous_.value = homegeneous.value;

  return shared_from_this();
}

Hsu::Types::HomogeneousM Frame::get_homegeneous_relative_to(Sprt target) {
  std::lock_guard<std::recursive_mutex> _{mu_};
  if (target == shared_from_this()) {
    return Hsu::Types::HomogeneousM();
  }
  // TODO: 公共父节点优化
  auto world = WORLD_FRAME();

  auto&& H_TargetInWorld = target->get_homegeneous_relative_to(world);
  auto&& H_RefInWorld = ref_coor_->get_homegeneous_relative_to(world);

  return H_TargetInWorld.inv() * H_RefInWorld * homogeneous_;
}

Sprt Frame::set_reference_coor(Sprt ref_coor) {
  std::lock_guard<std::recursive_mutex> _{mu_};
  ref_coor_ = ref_coor;

  return shared_from_this();
}

Sprt Frame::change_reference_coor(Sprt ref_coor) {
  std::lock_guard<std::recursive_mutex> _{mu_};
  homogeneous_ = get_homegeneous_relative_to(ref_coor);

  set_reference_coor(ref_coor);

  return shared_from_this();
}

Sprt Frame::transform(Hsu::Types::HomogeneousM const& H, Sprt target) {
  std::lock_guard<std::recursive_mutex> _{mu_};
  auto&& H_target = target->get_homegeneous_relative_to(ref_coor_);
  homogeneous_ = H_target * H * H_target.inv() * homogeneous_;

  return shared_from_this();
}

// ---- Hsu::Types PVS ----

// PointV::PointV(Eigen::Vector3d coordinate) : value(coordinate.head<3>()) {}

// ---- VISUAL ----
#if defined(HSU_FRAME_VISUAL)
#include <pybind11/gil.h>
#include <pybind11/pytypes.h>
#include <pyconfig.h>
std::string Hsu::get_current_executor_path() {
  const std::size_t MAXBUFSIZE = 2048;
  char buf[MAXBUFSIZE] = {'\0'};
  auto res = readlink("/proc/self/exe", buf, MAXBUFSIZE);
  auto path = std::string(buf);
  std::size_t pos = path.find_last_of('/');
  if (pos != std::string::npos) {
    path = path.substr(0, pos);
  }
  return path;
}

pybind11::object frame_c2p(pybind11::object const& Frame, std::shared_ptr<Hsu::Frame> frame) {
  auto H = frame->get_homegeneous_relative_to(frame->WORLD_FRAME()).value;
  return Frame(frame->name, eigen_to_numpy(H));
}

Frame3DScene::Frame3DScene(Frame3DScene::Passkey) {}

Frame3DScene& Frame3DScene::begin() {
  if (state_ != STOP) return *this;

  main_ = new std::thread([&]() {
    pybind11::scoped_interpreter guard;

    pybind11::gil_scoped_acquire acquire;

    auto exec_dir = get_current_executor_path();

    auto sys = pybind11::module::import("sys");
    sys.attr("path").attr("append")(exec_dir);

    auto Core_ = pybind11::module::import("scripts.vispy_scene");
    auto VispyScene = Core_.attr("VispyScene");
    auto scene = VispyScene();

    auto Frame = Core_.attr("Frame");

    pybind11::dict data;
    pybind11::list list_l;
    pybind11::list list_r;

    for (int i = 0; i < 8; i++) {
      list_l.append(pybind11::none{});
      list_r.append(pybind11::none{});
    }

    pybind11::gil_scoped_release release;

    state_ = READY;

    while (state_ != START and state_ != STOP) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    state_ = RUNNING;

    while (state_ == RUNNING and state_ != STOP) {
      {
        pybind11::gil_scoped_acquire acquire;

        for (int i = 0; i < 8; i++) {
          list_l[i] = eigen_to_numpy(arm_left_[i].value);
          list_r[i] = eigen_to_numpy(arm_right_[i].value);
        }

        data["ArmL"] = list_l;
        data["ArmR"] = list_r;

        scene.attr("update")(data);
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(15));
    }

    {
      std::cout << "close" << std::endl;
      pybind11::gil_scoped_acquire acquire;
      scene.attr("close")();
    }
  });

  while (state_ != READY) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return *this;
}

Frame3DScene& Frame3DScene::set_arm_r_data(std::array<Types::HomogeneousM, 8> const& data) {
  arm_right_ = data;
  return *this;
}

Frame3DScene& Frame3DScene::set_arm_l_data(std::array<Types::HomogeneousM, 8> const& data) {
  arm_left_ = data;
  return *this;
}

// Frame3DScene& Frame3DScene::set_data(std::string const& name, pybind11::object const& obj) {
//   std::lock_guard<std::mutex> _{mu_};
//   if (data_) {
//     (*data_)[name.c_str()] = obj;
//   }
//   return *this;
// }

Frame3DScene& Frame3DScene::start() {
  if (state_ != READY) return *this;

  state_ = START;

  while (state_ != RUNNING) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return *this;
}

Frame3DScene& Frame3DScene::stop() {
  state_ = STOP;

  if (main_) {
    main_->join();
    delete main_;
  }

  return *this;
}
#endif