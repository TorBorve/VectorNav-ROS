#pragma once

#include <vn/vector.h>
#include <vn/matrix.h>

using namespace vn::math;
// matrix vector multiplication
template<size_t m_dim, size_t n_dim, typename T>
vec<m_dim, T> operator*(const mat<m_dim, n_dim, T>& matrix, const vec<n_dim, T>& vector){
    vec<m_dim, T> res = vec<m_dim, T>::zero();
    for (int i = 0; i < m_dim; i++){
        for(int j = 0; j < n_dim; j++){
            res[i] += matrix(j, i)*vector[j];
        }
    }
    return res;
}

// scalar product between vectors

