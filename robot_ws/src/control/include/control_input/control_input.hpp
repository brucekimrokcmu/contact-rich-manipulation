#pragma once
#include <Eigen/Dense>

namespace control::control_input
{

enum class ControlInputType
{
	DiffDrive,
	Ackermann,
};

class ControlInput
{
public:
	virtual ~ControlInput() = default;
	virtual Eigen::VectorXd asVector() const = 0;
	virtual void fromVector(const Eigen::VectorXd &vec) = 0;
	virtual ControlInputType type() const = 0;
	virtual std::unique_ptr<ControlInput> clone() const = 0;
	virtual std::size_t size() const = 0;
};

} // namespace control::control_input
