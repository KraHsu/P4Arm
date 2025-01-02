#pragma once

#include <Hsu/arm.h>
#include <Hsu/matrix_ops.h>
#include <memory>
#include <mutex>
#include <units.h>

namespace Hsu::detail {
template <typename T1, typename T2>
T2 map_range(T1 value, T1 a, T1 b, T2 x, T2 y) {
  if (a == b) {
    throw std::invalid_argument("输入范围 [a, b] 中 a = b 是非法的.");
  }
  if (value < a || value > b) {
    throw std::invalid_argument("输入值 value 应该属于区间 [a, b].");
  }
  return x + (y - x) * (static_cast<double>(value - a) / static_cast<double>(b - a));
}
}  // namespace Hsu::detail

namespace Hsu {
class Hand {
 public:
  struct AnglesData;
  struct Angles {
    static constexpr units::angle::degree_t Retention{-1024};
    units::angle::degree_t little;
    units::angle::degree_t ring;
    units::angle::degree_t middle;
    units::angle::degree_t index;
    units::angle::degree_t thumb;
    units::angle::degree_t thumb_r;

    Angles();

    Angles(units::angle::degree_t little, units::angle::degree_t ring, units::angle::degree_t middle,
           units::angle::degree_t index, units::angle::degree_t thumb, units::angle::degree_t thumb_r);

    Angles(AnglesData const& data);

    operator AnglesData() const;
  };

  struct AnglesData {
    int data[6];

    AnglesData();

    AnglesData(const Angles& angles);

    operator Angles() const;
  };

  struct Forces {
    units::mass::gram_t little;
    units::mass::gram_t ring;
    units::mass::gram_t middle;
    units::mass::gram_t index;
    units::mass::gram_t thumb;
    units::mass::gram_t thumb_r;
  };

 private:
  std::mutex mutex_;
  std::shared_ptr<ModbusActorBase> ma_;

  Angles angles_;

 public:
  Hand(std::shared_ptr<ModbusActorBase> ma);

  ~Hand();

 public:
  bool clear_err();

  Angles read_angles();

  bool set_angles(Angles const& angles);

  void test();

  // bool set_forces();

 public:
  enum class TactileType {
    LittleTip,
    LittleDistal,
    LittlePad,
    RingTip,
    RingDistal,
    RingPad,
    MiddleTip,
    MiddleDistal,
    MiddlePad,
    IndexTip,
    IndexDistal,
    IndexPad,
    ThumbTip,
    ThumbDistal,
    ThumbMiddle,
    ThumbPad,
    Palm
  };

  template <TactileType Type>
  struct TactileMatrix;

  // Little 指触觉数据
  template <>
  struct TactileMatrix<TactileType::LittleTip> {
    using MatrixType = Eigen::Matrix<int, 3, 3>;
    static constexpr int address = 3000;
  };

  template <>
  struct TactileMatrix<TactileType::LittleDistal> {
    using MatrixType = Eigen::Matrix<int, 12, 8>;
    static constexpr int address = 3018;
  };

  template <>
  struct TactileMatrix<TactileType::LittlePad> {
    using MatrixType = Eigen::Matrix<int, 10, 8>;
    static constexpr int address = 3210;
  };

  // Ring 指触觉数据
  template <>
  struct TactileMatrix<TactileType::RingTip> {
    using MatrixType = Eigen::Matrix<int, 3, 3>;
    static constexpr int address = 3370;
  };

  template <>
  struct TactileMatrix<TactileType::RingDistal> {
    using MatrixType = Eigen::Matrix<int, 12, 8>;
    static constexpr int address = 3388;
  };

  template <>
  struct TactileMatrix<TactileType::RingPad> {
    using MatrixType = Eigen::Matrix<int, 10, 8>;
    static constexpr int address = 3580;
  };

  // Middle 指触觉数据
  template <>
  struct TactileMatrix<TactileType::MiddleTip> {
    using MatrixType = Eigen::Matrix<int, 3, 3>;
    static constexpr int address = 3740;
  };

  template <>
  struct TactileMatrix<TactileType::MiddleDistal> {
    using MatrixType = Eigen::Matrix<int, 12, 8>;
    static constexpr int address = 3758;
  };

  template <>
  struct TactileMatrix<TactileType::MiddlePad> {
    using MatrixType = Eigen::Matrix<int, 10, 8>;
    static constexpr int address = 3950;
  };

  // Index 指触觉数据
  template <>
  struct TactileMatrix<TactileType::IndexTip> {
    using MatrixType = Eigen::Matrix<int, 3, 3>;
    static constexpr int address = 4110;
  };

  template <>
  struct TactileMatrix<TactileType::IndexDistal> {
    using MatrixType = Eigen::Matrix<int, 12, 8>;
    static constexpr int address = 4128;
  };

  template <>
  struct TactileMatrix<TactileType::IndexPad> {
    using MatrixType = Eigen::Matrix<int, 10, 8>;
    static constexpr int address = 4320;
  };

  // Thumb 指触觉数据
  template <>
  struct TactileMatrix<TactileType::ThumbTip> {
    using MatrixType = Eigen::Matrix<int, 3, 3>;
    static constexpr int address = 4480;
  };

  template <>
  struct TactileMatrix<TactileType::ThumbDistal> {
    using MatrixType = Eigen::Matrix<int, 12, 8>;
    static constexpr int address = 4498;
  };

  template <>
  struct TactileMatrix<TactileType::ThumbMiddle> {
    using MatrixType = Eigen::Matrix<int, 3, 3>;
    static constexpr int address = 4690;
  };

  template <>
  struct TactileMatrix<TactileType::ThumbPad> {
    using MatrixType = Eigen::Matrix<int, 12, 8>;
    static constexpr int address = 4708;
  };

  // Palm 触觉数据
  template <>
  struct TactileMatrix<TactileType::Palm> {
    using MatrixType = Eigen::Matrix<int, 8, 14>;
    static constexpr int address = 4900;
  };

  template <TactileType Type>
  typename TactileMatrix<Type>::MatrixType read_tactile() {
    using MatrixType = typename TactileMatrix<Type>::MatrixType;

    MatrixType res;

    try {
      std::lock_guard<std::mutex> lock(mutex_);

      constexpr int address = TactileMatrix<Type>::address;

      auto vec = ma_->read_multiple_holding_registers(address, 1,
                                                      MatrixType::RowsAtCompileTime * MatrixType::ColsAtCompileTime);

      res = Eigen::Map<const MatrixType>(vec.data());
    } catch (const std::exception& e) {
      read_tactile_(e);
    }

    return res;
  }

 private:
  void read_tactile_(const std::exception& e);
};
}  // namespace Hsu

template <>
struct fmt::formatter<Hsu::Hand::Angles> {
  constexpr auto parse(fmt::format_parse_context& ctx) -> decltype(ctx.begin()) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(const Hsu::Hand::Angles& angles, FormatContext& ctx) const -> decltype(ctx.out()) {
    return fmt::format_to(ctx.out(),
                          "角度: {{ 小拇指: {:.2f}°, 无名指: {:.2f}°, 中指: {:.2f}°, 食指: {:.2f}°, 拇指: {:.2f}°, "
                          "拇指转角: {:.2f}° }}",
                          angles.little.value(), angles.ring.value(), angles.middle.value(), angles.index.value(),
                          angles.thumb.value(), angles.thumb_r.value());
  }
};

template <>
struct fmt::formatter<Hsu::Hand::AnglesData> {
  constexpr auto parse(fmt::format_parse_context& ctx) -> decltype(ctx.begin()) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(const Hsu::Hand::AnglesData& data, FormatContext& ctx) const -> decltype(ctx.out()) {
    return fmt::format_to(ctx.out(),
                          "角度数据: {{ 小拇指: {}, 无名指: {}, 中指: {}, 食指: {}, 拇指: {}, "
                          "拇指转角: {} }}",
                          data.data[0], data.data[1], data.data[2], data.data[3], data.data[4], data.data[5]);
  }
};

template <typename T, int Rows, int Cols>
struct fmt::formatter<Eigen::Matrix<T, Rows, Cols>> {
  constexpr auto parse(format_parse_context& ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(const Eigen::Matrix<T, Rows, Cols>& mat, FormatContext& ctx) const {
    std::string result = "[[";
    for (int i = 0; i < mat.rows(); ++i) {
      if (i) result += " [";
      for (int j = 0; j < mat.cols(); ++j) {
        result += fmt::format("{:>4}", mat(i, j));
        if (j != mat.cols() - 1) result += ", ";
      }
      result += "]";
      if (i != mat.rows() - 1) result += ",\n";
    }
    result += "]";
    return fmt::format_to(ctx.out(), "{}", result);
  }
};
