#pragma once
#include <Eigen/Dense>
#include <memory>

namespace control::state
{

	enum class StateType
	{
		DiffDrive,
		Ackermann,
	};

	class State
	{
	public:
		virtual ~State() = default;
		// make state copyable
		State(const State &) = default;
		State &operator=(const State &) = default;
		State(State &&) = default;
		State &operator=(State &&) = default;
		// make state movable
		virtual Eigen::VectorXd operator+(const Eigen::VectorXd &other) const = 0;
		virtual Eigen::VectorXd operator-(const Eigen::VectorXd &other) const = 0;
		virtual Eigen::VectorXd operator*(double scalar) const = 0;
		virtual Eigen::VectorXd operator/(double scalar) const = 0;
		virtual Eigen::VectorXd operator+(const State &other) const = 0;
		virtual Eigen::VectorXd operator-(const State &other) const = 0;
		virtual Eigen::VectorXd operator*(const Eigen::VectorXd &other) const = 0;
		virtual Eigen::VectorXd operator/(const Eigen::VectorXd &other) const = 0;
		virtual std::size_t size() const = 0;
		virtual StateType type() const = 0;
		virtual Eigen::VectorXd asVector() const = 0;
		virtual void fromVector(const Eigen::VectorXd &vec) = 0;
		virtual std::unique_ptr<State> clone() const = 0;
	};

} // namespace control::state
