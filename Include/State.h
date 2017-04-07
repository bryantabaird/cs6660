#pragma once

#include <Eigen/Dense>

struct State
{
	State()
	{
		Position = Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>::Zero(3,3);
		Velocity = Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>::Zero(3,3);
	}

	State(	Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> Position_, 
		Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> Velocity_)
	{
		Position = Position_;
		Velocity = Velocity_;
	}

	State(	Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> Position_, 
			Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> Velocity_,
			Eigen::Matrix<unsigned int, Eigen::Dynamic, 2, Eigen::RowMajor> SpringIndices_,
			Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> SpringParams_
		)
	{
		Position = Position_;
		Velocity = Velocity_;
		SpringIndices = SpringIndices_;
		SpringParams = SpringParams_;
	}

	State(	int rows, int cols)
	{
		Position.resize(rows, cols);
		Velocity.resize(rows, cols);
	}

	State operator*(float scalar)
	{
		State result;

		result.Position = Position * scalar;
		result.Velocity = Velocity * scalar;

		return result;
	}

	State operator+(State rhsState)
	{
		State result;

		result.Position = Position + rhsState.Position;
		result.Velocity = Velocity + rhsState.Velocity;

		return result;
	}

	void operator=(State rhsState)
	{
		Position.resize(rhsState.Position.rows(), rhsState.Position.cols());
		Position = rhsState.Position;

		Velocity.resize(rhsState.Velocity.rows(), rhsState.Velocity.cols());
		Velocity = rhsState.Velocity;

		SpringIndices.resize(rhsState.SpringIndices.rows(), rhsState.SpringIndices.cols());
		SpringIndices = rhsState.SpringIndices;

		SpringParams.resize(rhsState.SpringParams.rows(), rhsState.SpringParams.cols());
		SpringParams = rhsState.SpringParams;
	}

	void ResizeState(int rows, int cols)
	{
		Position.resize(rows, cols);
		Velocity.resize(rows, cols);
	}

	Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> Position;	// x, y, z
	Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> Velocity;	// Vx, Vy, Vz

	Eigen::Matrix<unsigned int, Eigen::Dynamic, 2, Eigen::RowMajor> SpringIndices; // Connections
	Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> SpringParams; // k-value, Lengths
};

struct Derivative
{
	Derivative()
	{
		dPosition = Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>::Zero(3,3);
		dVelocity = Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>::Zero(3,3);
	}

	Derivative(	Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> dPosition_, 
				Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> dVelocity_)
	{
		dPosition = dPosition_;
		dVelocity = dVelocity_;
	}

	Derivative(	int rows, int cols)
	{
		dPosition.resize(rows, cols);
		dVelocity.resize(rows, cols);
	}

	Derivative operator*(float scalar)
	{
		Derivative result;

		result.dPosition = dPosition * scalar;
		result.dVelocity = dVelocity * scalar;

		return result;
	}

	Derivative operator+(Derivative rhsState)
	{
		Derivative result;

		result.dPosition = dPosition + rhsState.dPosition;
		result.dVelocity = dVelocity + rhsState.dVelocity;

		return result;
	}

	void operator=(Derivative rhsState)
	{
		dPosition.resize(rhsState.dPosition.rows(), rhsState.dPosition.cols());
		dPosition = rhsState.dPosition;
		dVelocity.resize(rhsState.dVelocity.rows(), rhsState.dVelocity.cols());
		dVelocity = rhsState.dVelocity;
	}

	void ResizeState(int rows, int cols)
	{
		dPosition.resize(rows, cols);
		dVelocity.resize(rows, cols);
	}

	Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> dPosition;	// x/dt, y/dt, z/dt
	Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> dVelocity;	// Vx/dt, Vy/dt, Vz/dt
};

Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> Force(const State &state, float time);

Derivative Evaluate(const State &initial,
					float t,
					float dt,
					Derivative d);