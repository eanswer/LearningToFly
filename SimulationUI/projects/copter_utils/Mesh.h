#pragma once
#include <vector>
#include <Eigen/Dense>

namespace copter_utils {

class BoundingBox {
public:
    BoundingBox() {}
    BoundingBox(Eigen::Vector3f left_corner, Eigen::Vector3f right_corner)
        : _left_corner(left_corner), _right_corner(right_corner) {}

    Eigen::Vector3f _left_corner, _right_corner;
};

class SurfaceMesh {
public:
    std::vector<Eigen::Vector3f>& vertices() { return _vertices; }
    std::vector<Eigen::Vector3i>& elements() { return _elements; }
    Eigen::Vector3f& vertices(int idx) { return _vertices[idx]; }
    Eigen::Vector3i& elements(int idx) { return _elements[idx]; }

    void Scale(float scale) {
        for (int i = 0;i < _vertices.size();++i)
            _vertices[i] *= scale;
    }

    void Translate(Eigen::Vector3f v) {
        for (int i = 0;i < _vertices.size();++i)
            _vertices[i] += v;
    }

    Eigen::Vector3f Center() {
        Eigen::Vector3f center(0, 0, 0);
        for (int i = 0;i < _vertices.size();++i) {
            center += _vertices[i];
        }
        return center / _vertices.size();
    }

private:
    std::vector<Eigen::Vector3f> _vertices;
    std::vector<Eigen::Vector3i> _elements;
};

}