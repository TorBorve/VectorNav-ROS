#pragma once

#include <vn/vector.h>
#include <vn/matrix.h>

// matrix vector multiplication
template<size_t m_dim, size_t n_dim, typename T>
vn::math::vec<m_dim, T> operator*(const vn::math::mat<m_dim, n_dim, T>& matrix, const vn::math::vec<n_dim, T>& vector){
    vn::math::vec<m_dim, T> res = vn::math::vec<m_dim, T>::zero();
    for (int i = 0; i < m_dim; i++){
        for(int j = 0; j < n_dim; j++){
            res[i] += matrix(j, i)*vector[j];
        }
    }
    return res;
}

// scalar product between vectors

