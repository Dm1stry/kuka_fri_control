#include <eigen3/Eigen/Eigen>
#include <algorithm>

template<class T, size_t x>
Eigen::Array<T, x, 1> stdArrayToEigenArray(const std::array<T, x>& source_array)
{
    Eigen::Array<T, x, 1> result;
    std::copy(source_array.cbegin(), source_array.cend(), result.begin());
    return result;
}

template<class T, int x>
std::array<T, x> eigenArrayToStdArray(const Eigen::Array<T, x, 1>& source_array)
{
    std::array<T, x> result;
    std::copy(source_array.cbegin(), source_array.cend(), result.begin());
    return result;
}

template<class T, int x>
std::array<T, x> eigenArrayToStdArray(const Eigen::Array<T, 1, x>& source_array)
{
    std::array<T, x> result;
    std::copy(source_array.cbegin(), source_array.cend(), result.begin());
    return result;
}