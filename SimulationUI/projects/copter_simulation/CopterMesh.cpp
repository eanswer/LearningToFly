#include "CopterMesh.h"

namespace copter_simulation {

CopterMesh::CopterMesh() {
    _body_mesh.clear();
    _body_mesh_mass.clear();
}


CopterMesh::~CopterMesh()
{
}

void CopterMesh::Scale(float scale_factor) {
    for (int i = 0;i < _body_mesh.size();++i)
        _body_mesh[i].Scale(scale_factor);
}

void CopterMesh::Translate(Eigen::Vector3f v) {
    for (int i = 0;i < _body_mesh.size();++i)
        _body_mesh[i].Translate(v);
}

Eigen::Vector3f CopterMesh::Center() {
    float total_mass = 0;
    Eigen::Vector3f center(0, 0, 0);
    for (int i = 0;i < _body_mesh.size();++i) {
        center += _body_mesh[i].Center() * _body_mesh_mass[i];
        total_mass += _body_mesh_mass[i];
    }

    center /= total_mass;

    return center;
}

float CopterMesh::Mass() {
    float total_mass = 0;
    for (float mass : _body_mesh_mass)
        total_mass += mass;
    return total_mass;
}

copter_utils::BoundingBox CopterMesh::BoundingBox() {
    if (_body_mesh.size() == 0) {
        return copter_utils::BoundingBox(Eigen::Vector3f(0.0f, 0.0f, 0.0f), Eigen::Vector3f(0.0f, 0.0f, 0.0f));
    }
    Eigen::Vector3f left_corner = _body_mesh[0].vertices(0);
    Eigen::Vector3f right_corner = left_corner;
    for (int i = 0;i < _body_mesh.size();++i)
        for (int j = 0;j < _body_mesh[i].vertices().size();++j) {
            for (int axis = 0;axis < 3;++axis) {
                if (_body_mesh[i].vertices(j)(axis) < left_corner(axis))
                    left_corner(axis) = _body_mesh[i].vertices(j)(axis);
                if (_body_mesh[i].vertices(j)(axis) > right_corner(axis))
                    right_corner(axis) = _body_mesh[i].vertices(j)(axis);
            }
        }
    return copter_utils::BoundingBox(left_corner, right_corner);
}

}