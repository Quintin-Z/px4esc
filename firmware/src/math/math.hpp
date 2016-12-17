/**
 * Copyright (c) 2016  Zubax Robotics OU  <info@zubax.com>
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
 *    disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *    following disclaimer in the documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <Eigen/Eigen>
#include <algorithm>
#include <utility>
#include <cassert>
#include <type_traits>
#include <cmath>
#include <cstdint>
#include <zubax_chibios/util/heapless.hpp>


namespace math
{

using Scalar = float;
using Const = const Scalar;

template <int Rows, int Cols>
using Matrix = Eigen::Matrix<Scalar, Rows, Cols>;

template <int Rows>
using DiagonalMatrix = Eigen::DiagonalMatrix<Scalar, Rows>;

template <int Size>
using Vector = Matrix<Size, 1>;


constexpr inline Scalar convertKelvinToCelsius(Const kelvin)
{
    return kelvin - 273.15F;
}

constexpr inline Scalar convertCelsiusToKelvin(Const kelvin)
{
    return kelvin + 273.15F;
}

/**
 * Constants
 */
constexpr auto Pi  = Scalar(3.141592653589793);
constexpr auto Pi2 = Scalar(6.283185307179586);

/**
 * Constrains the angle within [0, Pi*2]
 */
inline Scalar normalizeAngle(Scalar x)
{
    if (x >= Pi2)
    {
        return x - Pi2;
    }
    else if (x < 0)
    {
        return x + Pi2;
    }
    else
    {
        return x;
    }
}

/**
 * Inclusive range of the form [min, max].
 */
template <typename T = Scalar>
struct Range
{
    T min = T(0);
    T max = T(0);

    constexpr Range() { }

    constexpr Range(const T min, const T max) :
        min(min),
        max(max)
    {
        assert(min < max);
    }

    constexpr T constrain(const T value) const
    {
        return std::min(std::max(value, min), max);
    }

    constexpr bool contains(const T value) const
    {
        return (value >= min) && (value <= max);
    }

    auto toString() const
    {
        return os::heapless::concatenate("[", min, ", ", max, "]");
    }
};

/**
 * Simple moving average of arbitrary depth.
 */
template <unsigned Depth, typename T>
class SimpleMovingAverageFilter
{
    T history_[Depth] = {};
    T sum_;
    unsigned next_index_ = 0;

public:
    SimpleMovingAverageFilter() :
        sum_(T() * Depth)
    {
        std::fill_n(std::begin(history_), Depth, T());
    }

    explicit SimpleMovingAverageFilter(const T& initial_value) :
        sum_(initial_value * Depth)
    {
        std::fill_n(std::begin(history_), Depth, initial_value);
    }

    void update(const T& value)
    {
        sum_ -= history_[next_index_];
        history_[next_index_] = value;
        sum_ += history_[next_index_];

        next_index_++;
        if (next_index_ >= Depth)
        {
            next_index_ = 0;
        }
    }

    void reset(const T& initial_value)
    {
        sum_ = initial_value * Depth;
        std::fill_n(std::begin(history_), Depth, initial_value);
    }

    T getValue() const
    {
        return sum_ / Depth;
    }
};

/**
 * Copmutes average over the entire collected dataset.
 * Useful for calibration and other batch measurements, e.g. motor identification.
 */
template <typename T = double>
class CumulativeAverageComputer
{
    std::uint32_t num_samples_ = 0;
    T accumulator_{};

    using FloatingPointScalar = typename std::conditional<std::is_floating_point<T>::value, T, Scalar>::type;

public:
    CumulativeAverageComputer() :
        accumulator_()
    { }

    explicit CumulativeAverageComputer(const T& init) :
        accumulator_(init)
    { }

    void addSample(const T& x)
    {
        num_samples_++;
        accumulator_ += x;
    }

    T getAverage() const
    {
        if (num_samples_ > 0)
        {
            return T(accumulator_ / FloatingPointScalar(num_samples_));
        }
        else
        {
            assert(false);
            return T();
        }
    }

    auto getNumSamples() const { return num_samples_; }
};

/**
 * Simple rolling sample variance algorithm.
 * This implementation may experience poor numerical stability for large samples, due to squaring.
 * Useful resources:
 *  - https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance
 *  - http://math.stackexchange.com/questions/595541/rolling-standard-deviations
 *
 * @tparam WindowLength     maximum window length
 * @tparam T                target data type, scalar (vectors/matrices are NOT supported)
 */
template <unsigned WindowLength, typename T>
class RollingSampleVariance
{
    T window_[WindowLength];

    T mean_sum_ = T(0);
    T squared_mean_sum_ = T(0);

    unsigned current_depth_ = 0;
    unsigned next_index_ = 0;

public:
    void update(const T x_new)
    {
        if (current_depth_ < WindowLength)
        {
            current_depth_++;
        }
        else
        {
            const T x_old = window_[next_index_];
            mean_sum_ -= x_old;
            squared_mean_sum_ -= x_old * x_old;
        }

        mean_sum_ += x_new;
        squared_mean_sum_ += x_new * x_new;
        window_[next_index_] = x_new;

        next_index_++;
        if (next_index_ >= WindowLength)
        {
            next_index_ = 0;
        }
    }

    void reset()
    {
        mean_sum_ = T(0);
        squared_mean_sum_ = T(0);

        current_depth_ = 0;
        next_index_ = 0;
    }

    T getMean() const
    {
        if (current_depth_ > 0)
        {
            return mean_sum_ / T(current_depth_);
        }

        assert(false);
        return T(0);
    }

    T getVariance() const
    {
        if (current_depth_ > 1)
        {
            const T mean = mean_sum_ / T(current_depth_);
            const T squared_mean = squared_mean_sum_ / T(current_depth_);
            return (squared_mean - (mean * mean)) * (T(current_depth_) / T(current_depth_ - 1U));
        }

        assert(false);
        return T(0);
    }

    T getStandardDeviation() const
    {
        return std::sqrt(getVariance());
    }

    bool areEstimatesAvailable() const
    {
        return current_depth_ > 1;
    }
};

/**
 * Standard one dimensional five point stencil numerical differentiation algorithm.
 */
template <typename T>
class FivePointDifferentiator
{
    T p_[5]{};
    unsigned n_ = 0;

public:
    T update(const T x)
    {
        // This block ensures correct approximation during initialization
        if (n_ < 5)
        {
            p_[n_++] = x;

            if (n_ < 2)
            {
                return T(0);
            }

            if (n_ < 5)
            {
                return p_[n_ - 1] - p_[n_ - 2];
            }
        }
        else
        {
            p_[0] = p_[1];
            p_[1] = p_[2];
            p_[2] = p_[3];
            p_[3] = p_[4];
            p_[4] = x;
        }

        return (p_[0] -
                p_[1] * T(8) +
                p_[3] * T(8) -
                p_[4]) / T(12);
    }

    void reset()
    {
        n_ = 0;
    }
};


inline Vector<2> sincos(Scalar x)
{
    // Normally this should be replaced with a call to sincos(), but this is not a part of C++ standard library.
    // However, the compiler should be able to replace the two separate but localized calls to
    // sin()/cos() with one sincos().
    return { std::sin(x), std::cos(x) };
}

/**
 * Implementation details, do not use directly.
 */
namespace impl_
{

template <typename Scalar, int Size>
using RowVector = Eigen::Matrix<Scalar, 1, Size, Eigen::RowMajor>;

template <typename Scalar, int Size, typename Head>
inline void fillVector(RowVector<Scalar, Size>& vector,
                       int next_index,
                       Head head)
{
    vector[next_index] = static_cast<Scalar>(head);
}

template <typename Scalar, int Size, typename Head, typename... Tail>
inline void fillVector(RowVector<Scalar, Size>& vector,
                       int next_index,
                       Head head,
                       Tail... tail)
{
    vector[next_index] = static_cast<Scalar>(head);
    fillVector(vector, next_index + 1, tail...);
}

template <typename Scalar, int Rows, int Columns>
inline void fillMatrix(Eigen::Matrix<Scalar, Rows, Columns>& matrix,
                       int next_row,
                       const impl_::RowVector<Scalar, Columns>& head)
{
    matrix.row(next_row) = head;
}

template <typename Scalar, int Rows, int Columns, typename... Tail>
inline void fillMatrix(Eigen::Matrix<Scalar, Rows, Columns>& matrix,
                       int next_row,
                       const impl_::RowVector<Scalar, Columns>& head,
                       Tail... tail)
{
    matrix.row(next_row) = head;
    fillMatrix(matrix, next_row + 1, tail...);
}

template <typename Scalar, int DiagonalSize>
inline void fillDiagonalMatrix(Eigen::DiagonalMatrix<Scalar, DiagonalSize>& matrix,
                               int next_position,
                               Scalar head)
{
    matrix.diagonal()[next_position] = static_cast<Scalar>(head);
}

template <typename Scalar, int DiagonalSize, typename... Tail>
inline void fillDiagonalMatrix(Eigen::DiagonalMatrix<Scalar, DiagonalSize>& matrix,
                               int next_position,
                               Scalar head,
                               Tail... tail)
{
    matrix.diagonal()[next_position] = static_cast<Scalar>(head);
    fillDiagonalMatrix<Scalar, DiagonalSize>(matrix, next_position + 1, tail...);
}

} // namespace impl_

/**
 * Creates a row vector of arbitrary length, represented as a static row vector Eigen::Matrix<>.
 * Scalar type can be overriden.
 * See also @ref makeMatrix().
 */
template <typename Scalar = Scalar, typename... Tail>
inline impl_::RowVector<Scalar, sizeof...(Tail)>
makeRow(Tail... tail)
{
    impl_::RowVector<Scalar, sizeof...(Tail)> vector;
    impl_::fillVector(vector, 0, tail...);
    return vector;
}

/**
 * Creates a static matrix (Eigen::Matrix<>) of arbitrary size from a set of row vectors.
 * The matrix will use default layout, which is columnn-major for Eigen.
 * Note that row vectors of unequal size trigger a compile-time error.
 * Scalar type will be deduced automatically from the first row.
 * This function is totally type safe.
 * Usage:
 *      makeMatrix(makeRow(1, 2),
 *                 makeRow(3, 4))  // Produces a 2x2 matrix
 */
template <typename Scalar, int Columns, typename... Tail>
inline Eigen::Matrix<Scalar, sizeof...(Tail) + 1, Columns>
makeMatrix(const impl_::RowVector<Scalar, Columns>& head,
           Tail... tail)
{
    Eigen::Matrix<Scalar, sizeof...(Tail) + 1, Columns> matrix;
    matrix.setZero();
    impl_::fillMatrix(matrix, 0, head, tail...);
    return matrix;
}

/**
 * A helper like @ref makeMatrix() that creates diagonal matrix.
 * Size is dereved from the argument list.
 * Scalar type can be overriden.
 */
template <typename Scalar = Scalar, typename... Diagonal>
inline Eigen::DiagonalMatrix<Scalar, sizeof...(Diagonal)>
makeDiagonalMatrix(Diagonal... diag)
{
    Eigen::DiagonalMatrix<Scalar, sizeof...(Diagonal)> matrix;
    matrix.setZero();
    impl_::fillDiagonalMatrix<Scalar, sizeof...(Diagonal)>(matrix, 0, diag...);
    return matrix;
}

/**
 * Printing helpers
 * @{
 */
enum class StringRepresentation
{
    SingleLine,
    MultiLine
};

template <typename Scalar, int Rows, int Columns>
inline auto toString(const Eigen::Matrix<Scalar, Rows, Columns>& matrix,
                     const StringRepresentation representation = StringRepresentation::SingleLine)
{
    os::heapless::String<Rows * Columns * 20> s;

    for (int row = 0; row < Rows; row++)
    {
        if (row > 0)
        {
            s.append((representation == StringRepresentation::MultiLine) ? "\n" : "; ");
        }
        for (int column = 0; column < Columns; column++)
        {
            if (column > 0)
            {
                s.append(", ");
            }
            s.append(matrix(row, column));
        }
    }

    return s;
}
/**
 * @}
 */

}
