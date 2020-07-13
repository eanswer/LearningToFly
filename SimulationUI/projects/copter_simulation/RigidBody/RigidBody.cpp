#include "RigidBody.h"
#include <iostream>
namespace copter_simulation {

    RigidBody::RigidBody() 
    : _mass(1.0),
    _inertia_body(Eigen::Matrix3d::Identity()), 
    _inertia_body_inv(Eigen::Matrix3d::Identity()),
    _position(Eigen::Vector3d::Zero()),
    _orientation(Eigen::Quaterniond::Identity()), 
    _linear_momentum(Eigen::Vector3d::Zero()),
    _angular_momentum(Eigen::Vector3d::Zero()),
    _inertia_inv(Eigen::Matrix3d::Identity()),
    _R(Eigen::Matrix3d::Identity()),
    _velocity(Eigen::Vector3d::Zero()),
    _acceleration(Eigen::Vector3d::Zero()),
    _omega(Eigen::Vector3d::Zero()),
    _external_force(Eigen::Vector3d::Zero()),
    _external_torque(Eigen::Vector3d::Zero()) {}
    
    void RigidBody::Initialize(const double mass, const Eigen::Matrix3d& inertia_body,
        const Eigen::Vector3d& position, const Eigen::Vector3d& velocity,
        const Eigen::Quaterniond& orientation,
        const Eigen::Vector3d& angular_velocity) {
        _mass = mass;
        _inertia_body = Eigen::Matrix3d(inertia_body);
        _inertia_body_inv = _inertia_body.inverse();
        _position = Eigen::Vector3d(position);
        _orientation = Eigen::Quaterniond(orientation);
        _linear_momentum = velocity * mass;
        _R = _orientation.matrix();
        _omega = Eigen::Vector3d(angular_velocity);
        _angular_momentum = _R * _inertia_body * _R.transpose() * _omega;
        _inertia_inv = _R * _inertia_body_inv * _R.transpose();
        _velocity = Eigen::Vector3d(velocity);
        _acceleration.setZero();
        _external_force.setZero();
        _external_torque.setZero();
    }

    void RigidBody::Initialize(const Eigen::Vector3d& position,
        const Eigen::Vector3d& velocity, const Eigen::Quaterniond& orientation,
        const Eigen::Vector3d& angular_velocity) {
        _position = Eigen::Vector3d(position);
        _orientation = Eigen::Quaterniond(orientation);
        _linear_momentum = velocity * _mass;
        _R = _orientation.matrix();
        _omega = Eigen::Vector3d(angular_velocity);
        _angular_momentum = _R * _inertia_body * _R.transpose() * _omega;
        _inertia_inv = _R * _inertia_body_inv * _R.transpose();
        _velocity = Eigen::Vector3d(velocity);
        _acceleration.setZero();
        _external_force.setZero();
        _external_torque.setZero();
    }

    void RigidBody::ApplyForceInWorldFrame(const Eigen::Vector3d& force, const Eigen::Vector3d& position) {
        _external_force += force;
        _external_torque += (position - _position).cross(force);
    }

    void RigidBody::ApplyForceInBodyFrame(const Eigen::Vector3d& force, const Eigen::Vector3d& position) {
        ApplyForceInWorldFrame(VectorBodyToWorld(force), PointBodyToWorld(position));
    }

    void RigidBody::ApplyTorque(const Eigen::Vector3d& torque) {
        _external_torque += torque;
    }

    void RigidBody::Clear() {
        _external_force.setZero();
        _external_torque.setZero();
    }

    void RigidBody::Advance(const double dt) {
        // update states variables first
        _position = _position + _velocity * dt;
        
        Eigen::Quaterniond omega(0.0, _omega(0), _omega(1), _omega(2));
        _orientation = Add(_orientation, Multiply(omega * _orientation, 0.5 * dt));
        _orientation.normalize();

        _linear_momentum += _external_force * dt;

        _angular_momentum += _external_torque * dt;
      
        // update derived quantities
        _R = _orientation.matrix();
        _inertia_inv = _R * _inertia_body_inv * _R.transpose();
        _velocity = _linear_momentum / _mass;
        _acceleration = _external_force / _mass;
        _omega = _inertia_inv * _angular_momentum;

        // clear external input
        Clear();
    }

    const Eigen::Vector3d RigidBody::PointWorldToBody(
        const Eigen::Vector3d& world_point) const {
        return _R.transpose() * (world_point - _position);
    }

    const Eigen::Matrix3Xd RigidBody::PointsWorldToBody(
        const Eigen::Matrix3Xd& world_points) const {
        return _R.transpose() * (world_points.colwise() - _position);
    }

    const Eigen::Vector3d RigidBody::PointBodyToWorld(
        const Eigen::Vector3d& body_point) const {
        return _R * body_point + _position;
    }

    const Eigen::Matrix3Xd RigidBody::PointsBodyToWorld(
        const Eigen::Matrix3Xd& body_points) const {
        return (_R * body_points).colwise() + _position;
    }

    const Eigen::Vector3d RigidBody::VectorWorldToBody(
        const Eigen::Vector3d& world_vector) const {
        return _R.transpose() * world_vector;
    }

    const Eigen::Matrix3Xd RigidBody::VectorsWorldToBody(
        const Eigen::Matrix3Xd& world_vectors) const {
        return _R.transpose() * world_vectors;
    }

    const Eigen::Vector3d RigidBody::VectorBodyToWorld(
        const Eigen::Vector3d& body_vector) const {
        return _R * body_vector;
    }

    const Eigen::Matrix3Xd RigidBody::VectorsBodyToWolrd(
        const Eigen::Matrix3Xd& body_vectors) const {
        return _R * body_vectors;
    }

    const Eigen::Matrix4d RigidBody::BodyToWorldTransform() const {
        Eigen::Matrix4d transform;
        transform.block(0, 0, 3, 3) = _R;
        transform.block(0, 3, 3, 1) = _position;
        transform.row(3) << 0, 0, 0, 1.0;
        return transform;
    }

    const Eigen::Matrix4d RigidBody::WorldToBodyTransform() const {
        Eigen::Matrix4d transform;
        transform.block(0, 0, 3, 3) = _R.transpose();
        transform.block(0, 3, 3, 1) = -_R.transpose() * _position;
        transform.row(3) << 0, 0, 0, 1.0;
        return transform;
    }

    Eigen::Quaterniond RigidBody::Add(const Eigen::Quaterniond& q1,
        const Eigen::Quaterniond& q2) {
        return Eigen::Quaterniond(q1.w() + q2.w(), q1.x() + q2.x(),
            q1.y() + q2.y(), q1.z() + q2.z());
    }

    Eigen::Quaterniond RigidBody::Multiply(const Eigen::Quaterniond& q,
        const double scale) {
        return Eigen::Quaterniond(q.w() * scale, q.x() * scale,
            q.y() * scale, q.z() * scale);
    }
}