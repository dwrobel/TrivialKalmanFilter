/*
 * Unidimensional Kalman filter.
 *
 * Copyright (C) 2018  Damian Wrobel <dwrobel@ertelnet.rybnik.pl>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

 #pragma once
 #include <math.h>

 /**
 * @mainpage Unidimensional trivial Kalman filter (header only, Arduino compatible) library
 *
 * @image html DS18B20Test.svg
 * - @ref TrivialKalmanFilter - API
 * - Examples
 *   - @ref Example             - How to use it
 *   - @ref DS18B20Test.ino     - Arduino example
 *   - @ref DS18B20Test.csv     - Sample data
 *   - @ref covariance.m        - Octave based covariance calculator
 * - <a href="https://github.com/dwrobel/TrivialKalmanFilter">Project page</a>
 */

/**
 * Unidimensional Kalman filter
 *
 * Useful for reducing input signal noise (e.g. from the temperature sensor).
 *
 * Memory efficient, consumes only 4 x sizeof(float)) [bytes] of RAM memory.
 * On (Arduino Nano 16MHz) execution of update() method takes about 100[us].
 *
 * @class TrivialKalmanFilter
 * @tparam D type to be used for arithmetic operations (e.g. float or double).
 * @example DS18B20Test.ino
 * @see https://en.wikipedia.org/wiki/Kalman_filter#Details
 */
template <typename D = float> class TrivialKalmanFilter {
public:
  /**
   * Instantiates Kalman filter
   *
   * @param Rk Estimation of the noise covariances (process)
   * @param Qk Estimation of the noise covariances (observation)
   *
   * @see covariance.m - Covariance calculator for Rk parameter
   * @see https://en.wikipedia.org/wiki/Kalman_filter#Estimation_of_the_noise_covariances_Qk_and_Rk
   *
   * Usage:
   * @anchor Example
   * @code
   * TrivialKalmanFilter tkf<float>(4.7e-3, 1e-5);
   *
   * while (true) {
   *     const auto filteredValue = tkf.update(getRawValueFromSensor());
   *     // further processing of filteredValue
   * }
   * @endcode
   */
  TrivialKalmanFilter(const D Rk, const D Qk)
      : Rk(Rk)
      , Qk(Qk) {
    reset();
  }

  /**
   * Updates Kalman filter
   *
   * @param zk measured value
   *
   * @return estimated value
   *
   * @see https://en.wikipedia.org/wiki/Kalman_filter#Predict
   */
  D update(const D zk) {
    D xk       = (Fk * xk_last) + (Bk * uk); // Predicted (a priori) state estimate
    D Pk       = (Fk * Pk_last * Fk_T) + Qk; // Predicted (a priori) error covariance
    D yk       = zk - (Hk * xk);             // Innovation or measurement pre-fit residual
    const D Sk = Rk + (Hk * Pk * Hk_T);      // Innovation (or pre-fit residual) covariance
    const D Kk = (Pk * Hk_T) / Sk;           // Optimal Kalman gain
    xk         = xk + (Kk * yk);             // Updated (a posteriori) state estimate
    Pk         = (I - (Kk * Hk)) * Pk;       // Updated (a posteriori) estimate covariance (a.k.a Joseph form)

#if 0 // unused part
    D yk       = zk - (Hk_T * xk);           // Measurement post-fit residual
#endif

    xk_last = xk;
    Pk_last = Pk;

    return xk;
  }

  /**
   * Returns last estimated value
   *
   * @return last estimated value
   */
  D get() const {
    return xk_last;
  }

  /**
   * Resets filter to its initial state
   */
  void reset(const D xk = 0, const D Pk = 1) {
    xk_last = xk;
    Pk_last = Pk;
  }

private:
  // Assumes simplified model
  static constexpr D k    = 1;
  static constexpr D Bk   = 0;
  static constexpr D uk   = 0;
  static constexpr D Fk   = 1;
  static constexpr D T    = 1;
  static constexpr D Fk_T = pow(Fk, T);
  static constexpr D Hk   = 1;
  static constexpr D Hk_T = pow(Hk, T);
  static constexpr D I    = 1;

  const D Rk;
  const D Qk;

  D xk_last;
  D Pk_last;
};
