#pragma once

#include "foundation/PxTransform.h"
#include <algorithm>
#include <cmath>
#include <limits>

#ifdef _MSC_VER
#pragma float_control(precise, on, push)
#endif

namespace PARS
{
    namespace FootstepInterval
    {
        // float 반올림 오차를 각 연산의 범위에 포함한다. 빠른 수학 최적화로 재결합하면 안 된다.
        struct Range
        {
        public:
            Range() = default;
            Range(double value) : lower(value), upper(value), center(value)
            {
            }
            Range(double minimum, double maximum) : lower(minimum), upper(maximum),
                center((minimum + maximum) * .5), error((maximum - minimum) * .5)
            {
            }

        public:
            bool IsFinite() const
            {
                return std::isfinite(lower) && std::isfinite(upper) && lower <= upper;
            }

        public:
            double lower = 0;
            double upper = 0;
            double center = 0;
            double slope = 0;
            double error = 0;
        };

        inline Range Unknown()
        {
            return Range(-std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
        }

        inline Range Rounded(double lower, double upper)
        {
            const float infinity = std::numeric_limits<float>::infinity();
            return Range(std::nextafter(static_cast<float>(lower), -infinity),
                std::nextafter(static_cast<float>(upper), infinity));
        }

        // 같은 시간 변수의 반복 사용을 보존하는 일차 아핀 포락선이다.
        // 비선형 나머지와 매 연산의 float 반올림 오차는 독립 오차 반경에 누적한다.
        inline Range Affine(double center, double slope, double error)
        {
            const double magnitude = std::abs(center) + std::abs(slope) + error;
            if (!std::isfinite(magnitude))
            {
                return Unknown();
            }
            const double rounding = magnitude * std::numeric_limits<float>::epsilon() +
                std::numeric_limits<float>::denorm_min();
            error += rounding + 32 * std::numeric_limits<double>::epsilon() * magnitude;
            Range result;
            result.center = center;
            result.slope = slope;
            result.error = error;
            result.lower = center - std::abs(slope) - error;
            result.upper = center + std::abs(slope) + error;
            return result;
        }

        inline Range Time(double begin, double end)
        {
            return Affine((begin + end) * .5, (end - begin) * .5, 0);
        }

        inline Range Join(const Range& first, const Range& second)
        {
            const double center = (first.center + second.center) * .5;
            const double slope = (first.slope + second.slope) * .5;
            const double firstError = first.error + std::abs(first.center - center) + std::abs(first.slope - slope);
            const double secondError = second.error + std::abs(second.center - center) + std::abs(second.slope - slope);
            return Affine(center, slope, (std::max)(firstError, secondError));
        }

        inline Range operator+(const Range& first, const Range& second)
        {
            return Affine(first.center + second.center, first.slope + second.slope, first.error + second.error);
        }

        inline Range operator-(const Range& first, const Range& second)
        {
            return Affine(first.center - second.center, first.slope - second.slope, first.error + second.error);
        }

        inline Range operator-(const Range& value)
        {
            Range result = value;
            result.lower = -value.upper;
            result.upper = -value.lower;
            result.center = -value.center;
            result.slope = -value.slope;
            return result;
        }

        inline Range operator*(const Range& first, const Range& second)
        {
            const double error = std::abs(first.center) * second.error + std::abs(second.center) * first.error +
                (std::abs(first.slope) + first.error) * (std::abs(second.slope) + second.error);
            return Affine(first.center * second.center,
                first.center * second.slope + second.center * first.slope, error);
        }

        inline Range operator/(const Range& first, const Range& second)
        {
            if (!second.IsFinite() || (second.lower <= 0 && second.upper >= 0))
            {
                return Unknown();
            }
            const double centerSquared = second.center * second.center;
            const double halfWidth = std::abs(second.slope) + second.error;
            const double closest = (std::min)(std::abs(second.lower), std::abs(second.upper));
            const double error = second.error / centerSquared + halfWidth * halfWidth / (centerSquared * closest);
            return first * Affine(1.0 / second.center, -second.slope / centerSquared, error);
        }

        inline Range Square(const Range& value)
        {
            if (value.lower > 0 || value.upper < 0)
            {
                return value * value;
            }
            const double maximum = (std::max)(value.lower * value.lower, value.upper * value.upper);
            const double minimum = value.lower <= 0 && value.upper >= 0 ? 0 :
                (std::min)(value.lower * value.lower, value.upper * value.upper);
            const Range rounded = Rounded(minimum, maximum);
            return Range((std::max)(0.0, rounded.lower), rounded.upper);
        }

        inline Range Sqrt(const Range& value)
        {
            if (!value.IsFinite() || value.upper < 0)
            {
                return Unknown();
            }
            if (value.lower > 0)
            {
                const double center = std::sqrt(value.center);
                const double halfWidth = std::abs(value.slope) + value.error;
                const double error = value.error / (2 * center) +
                    halfWidth * halfWidth / (8 * value.lower * std::sqrt(value.lower));
                return Affine(center, value.slope / (2 * center), error);
            }
            const Range rounded = Rounded(std::sqrt((std::max)(0.0, value.lower)), std::sqrt(value.upper));
            return Range((std::max)(0.0, rounded.lower), rounded.upper);
        }

        inline Range Clamp(const Range& value, double minimum, double maximum)
        {
            if (value.lower >= minimum && value.upper <= maximum)
            {
                return value;
            }
            return Range((std::max)(minimum, (std::min)(maximum, value.lower)),
                (std::max)(minimum, (std::min)(maximum, value.upper)));
        }

        inline Range Minimum(const Range& value, double maximum)
        {
            if (value.upper <= maximum)
            {
                return value;
            }
            return Range((std::min)(maximum, value.lower), (std::min)(maximum, value.upper));
        }

        inline Range Maximum(const Range& value, double minimum)
        {
            if (value.lower >= minimum)
            {
                return value;
            }
            return Range((std::max)(minimum, value.lower), (std::max)(minimum, value.upper));
        }

        inline double MagnitudeUpper(const Range& value)
        {
            return (std::max)(std::abs(value.lower), std::abs(value.upper));
        }

        inline Range CosPoint(double value)
        {
            const double square = value * value;
            double term = 1;
            double sum = 1;
            for (int index = 1; index <= 20; ++index)
            {
                const double divisor = static_cast<double>((2 * index - 1) * (2 * index));
                term *= -square / divisor;
                sum += term;
            }
            // |x| <= 7: 절댓값 항의 합은 exp(7) 미만이다.
            // 256 * double epsilon * exp(7) 및 42차 나머지를 1e-10으로 함께 감싼다.
            return Rounded(sum - 1e-10, sum + 1e-10);
        }

        inline Range Cos(const Range& angle)
        {
            if (!angle.IsFinite() || angle.lower < -1e-6 || angle.upper > 7)
            {
                return Range(-1, 1);
            }
            const Range center = CosPoint(angle.center);
            // 사인 중심값의 라이브러리 오차에는 별도 여유를 더한다.
            const double sine = std::sin(angle.center);
            const double halfWidth = std::abs(angle.slope) + angle.error;
            const double error = std::abs(sine) * angle.error + .5 * halfWidth * halfWidth +
                (center.upper - center.lower) + 1e-14 * (1 + halfWidth);
            const Range affine = Affine((center.lower + center.upper) * .5, -sine * angle.slope, error);
            if (affine.lower >= -1 && affine.upper <= 1)
            {
                return affine;
            }
            Range result = Join(CosPoint(angle.lower), CosPoint(angle.upper));
            constexpr double piLower = 3.141592653589792;
            constexpr double piUpper = 3.141592653589795;
            if (angle.lower <= piUpper && angle.upper >= piLower)
            {
                result.lower = -1;
            }
            if (angle.lower <= 0 || (angle.lower <= 2 * piUpper && angle.upper >= 2 * piLower))
            {
                result.upper = 1;
            }
            return Range((std::max)(-1.0, result.lower), (std::min)(1.0, result.upper));
        }

        struct Vector
        {
        public:
            Vector() = default;
            explicit Vector(const physx::PxVec3& value) : x(value.x), y(value.y), z(value.z)
            {
            }
            Vector(const Range& first, const Range& second, const Range& third) : x(first), y(second), z(third)
            {
            }

        public:
            bool IsFinite() const
            {
                return x.IsFinite() && y.IsFinite() && z.IsFinite();
            }

        public:
            Range x;
            Range y;
            Range z;
        };

        inline Vector operator+(const Vector& first, const Vector& second)
        {
            return Vector(first.x + second.x, first.y + second.y, first.z + second.z);
        }

        inline Vector operator-(const Vector& first, const Vector& second)
        {
            return Vector(first.x - second.x, first.y - second.y, first.z - second.z);
        }

        inline Vector operator*(const Vector& vector, const Range& scale)
        {
            return Vector(vector.x * scale, vector.y * scale, vector.z * scale);
        }

        inline Vector operator/(const Vector& vector, const Range& scale)
        {
            return Vector(vector.x / scale, vector.y / scale, vector.z / scale);
        }

        inline Range Dot(const Vector& first, const Vector& second)
        {
            return first.x * second.x + first.y * second.y + first.z * second.z;
        }

        inline Range Length(const Vector& value)
        {
            return Sqrt(Square(value.x) + Square(value.y) + Square(value.z));
        }

        inline Vector Cross(const Vector& first, const Vector& second)
        {
            return Vector(first.y * second.z - first.z * second.y,
                first.z * second.x - first.x * second.z, first.x * second.y - first.y * second.x);
        }

        inline Vector Join(const Vector& first, const Vector& second)
        {
            return Vector(Join(first.x, second.x), Join(first.y, second.y), Join(first.z, second.z));
        }

        inline bool Normalize(Vector& value)
        {
            const Range length = Length(value);
            if (!length.IsFinite() || length.lower <= 1e-6)
            {
                return false;
            }
            value = value / length;
            return value.IsFinite();
        }

        struct Quaternion
        {
        public:
            Quaternion() = default;
            explicit Quaternion(const physx::PxQuat& value) : x(value.x), y(value.y), z(value.z), w(value.w)
            {
            }
            Quaternion(const Range& first, const Range& second, const Range& third, const Range& fourth)
                : x(first), y(second), z(third), w(fourth)
            {
            }

        public:
            bool IsFinite() const
            {
                return x.IsFinite() && y.IsFinite() && z.IsFinite() && w.IsFinite();
            }

        public:
            Range x;
            Range y;
            Range z;
            Range w{1};
        };

        inline Quaternion operator*(const Quaternion& first, const Quaternion& second)
        {
            return Quaternion(first.w * second.x + second.w * first.x + first.y * second.z - second.y * first.z,
                first.w * second.y + second.w * first.y + first.z * second.x - second.z * first.x,
                first.w * second.z + second.w * first.z + first.x * second.y - second.x * first.y,
                first.w * second.w - first.x * second.x - first.y * second.y - second.z * first.z);
        }

        inline Quaternion operator*(const Quaternion& value, const Range& scale)
        {
            return Quaternion(value.x * scale, value.y * scale, value.z * scale, value.w * scale);
        }

        inline Quaternion Join(const Quaternion& first, const Quaternion& second)
        {
            return Quaternion(Join(first.x, second.x), Join(first.y, second.y),
                Join(first.z, second.z), Join(first.w, second.w));
        }

        inline bool Normalize(Quaternion& value)
        {
            const Range length = Sqrt(Square(value.x) + Square(value.y) + Square(value.z) + Square(value.w));
            if (!length.IsFinite() || length.lower <= 1e-6)
            {
                return false;
            }
            value = value * (Range(1) / length);
            return value.IsFinite();
        }

        inline Vector Rotate(const Quaternion& rotation, const Vector& value)
        {
            // PxQuat::rotate와 같은 연산 순서로 범위를 전파한다.
            const Range vx = Range(2) * value.x;
            const Range vy = Range(2) * value.y;
            const Range vz = Range(2) * value.z;
            const Range w2 = rotation.w * rotation.w - Range(.5);
            const Range dot2 = rotation.x * vx + rotation.y * vy + rotation.z * vz;
            return Vector(vx * w2 + (rotation.y * vz - rotation.z * vy) * rotation.w + rotation.x * dot2,
                vy * w2 + (rotation.z * vx - rotation.x * vz) * rotation.w + rotation.y * dot2,
                vz * w2 + (rotation.x * vy - rotation.y * vx) * rotation.w + rotation.z * dot2);
        }

        struct Transform
        {
        public:
            Transform() = default;
            explicit Transform(const physx::PxTransform& value) : position(value.p), rotation(value.q)
            {
            }

        public:
            bool IsFinite() const
            {
                return position.IsFinite() && rotation.IsFinite();
            }

        public:
            Vector position;
            Quaternion rotation;
        };

        inline Transform operator*(const Transform& first, const physx::PxTransform& second)
        {
            Transform result;
            result.position = first.position + Rotate(first.rotation, Vector(second.p));
            result.rotation = first.rotation * Quaternion(second.q);
            return result;
        }
    }
}

#ifdef _MSC_VER
#pragma float_control(pop)
#endif
