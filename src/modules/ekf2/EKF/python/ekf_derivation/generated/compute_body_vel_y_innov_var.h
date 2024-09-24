// -----------------------------------------------------------------------------
// This file was autogenerated by symforce from template:
//     function/FUNCTION.h.jinja
// Do NOT modify by hand.
// -----------------------------------------------------------------------------

#pragma once

#include <matrix/math.hpp>

namespace sym {

/**
 * This function was autogenerated from a symbolic function. Do not modify by hand.
 *
 * Symbolic function: compute_body_vel_y_innov_var
 *
 * Args:
 *     state: Matrix25_1
 *     P: Matrix24_24
 *     R: Scalar
 *
 * Outputs:
 *     innov_var: Scalar
 */
template <typename Scalar>
void ComputeBodyVelYInnovVar(const matrix::Matrix<Scalar, 25, 1>& state,
                             const matrix::Matrix<Scalar, 24, 24>& P, const Scalar R,
                             Scalar* const innov_var = nullptr) {
  // Total ops: 138

  // Input arrays

  // Intermediate terms (19)
  const Scalar _tmp0 = 2 * state(3, 0);
  const Scalar _tmp1 = 2 * state(1, 0);
  const Scalar _tmp2 = -_tmp0 * state(0, 0) + _tmp1 * state(2, 0);
  const Scalar _tmp3 = _tmp0 * state(2, 0) + _tmp1 * state(0, 0);
  const Scalar _tmp4 =
      -2 * std::pow(state(1, 0), Scalar(2)) - 2 * std::pow(state(3, 0), Scalar(2)) + 1;
  const Scalar _tmp5 = 2 * state(4, 0);
  const Scalar _tmp6 = _tmp1 * state(6, 0) - _tmp5 * state(3, 0);
  const Scalar _tmp7 = (Scalar(1) / Scalar(2)) * _tmp6;
  const Scalar _tmp8 =
      (Scalar(1) / Scalar(2)) * _tmp0 * state(6, 0) + (Scalar(1) / Scalar(2)) * _tmp1 * state(4, 0);
  const Scalar _tmp9 = 4 * state(5, 0);
  const Scalar _tmp10 = 2 * state(6, 0);
  const Scalar _tmp11 = _tmp10 * state(2, 0) - _tmp5 * state(0, 0) - _tmp9 * state(3, 0);
  const Scalar _tmp12 = (Scalar(1) / Scalar(2)) * _tmp11;
  const Scalar _tmp13 = _tmp10 * state(0, 0) + _tmp5 * state(2, 0) - _tmp9 * state(1, 0);
  const Scalar _tmp14 = (Scalar(1) / Scalar(2)) * state(2, 0);
  const Scalar _tmp15 =
      _tmp12 * state(0, 0) - _tmp13 * _tmp14 - _tmp7 * state(3, 0) + _tmp8 * state(1, 0);
  const Scalar _tmp16 = (Scalar(1) / Scalar(2)) * _tmp13;
  const Scalar _tmp17 =
      -_tmp12 * state(1, 0) - _tmp14 * _tmp6 + _tmp16 * state(3, 0) + _tmp8 * state(0, 0);
  const Scalar _tmp18 =
      _tmp11 * _tmp14 + _tmp16 * state(0, 0) - _tmp7 * state(1, 0) - _tmp8 * state(3, 0);

  // Output terms (1)
  if (innov_var != nullptr) {
    Scalar& _innov_var = (*innov_var);

    _innov_var = R +
                 _tmp15 * (P(0, 2) * _tmp18 + P(1, 2) * _tmp17 + P(2, 2) * _tmp15 +
                           P(3, 2) * _tmp2 + P(4, 2) * _tmp4 + P(5, 2) * _tmp3) +
                 _tmp17 * (P(0, 1) * _tmp18 + P(1, 1) * _tmp17 + P(2, 1) * _tmp15 +
                           P(3, 1) * _tmp2 + P(4, 1) * _tmp4 + P(5, 1) * _tmp3) +
                 _tmp18 * (P(0, 0) * _tmp18 + P(1, 0) * _tmp17 + P(2, 0) * _tmp15 +
                           P(3, 0) * _tmp2 + P(4, 0) * _tmp4 + P(5, 0) * _tmp3) +
                 _tmp2 * (P(0, 3) * _tmp18 + P(1, 3) * _tmp17 + P(2, 3) * _tmp15 + P(3, 3) * _tmp2 +
                          P(4, 3) * _tmp4 + P(5, 3) * _tmp3) +
                 _tmp3 * (P(0, 5) * _tmp18 + P(1, 5) * _tmp17 + P(2, 5) * _tmp15 + P(3, 5) * _tmp2 +
                          P(4, 5) * _tmp4 + P(5, 5) * _tmp3) +
                 _tmp4 * (P(0, 4) * _tmp18 + P(1, 4) * _tmp17 + P(2, 4) * _tmp15 + P(3, 4) * _tmp2 +
                          P(4, 4) * _tmp4 + P(5, 4) * _tmp3);
  }
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
