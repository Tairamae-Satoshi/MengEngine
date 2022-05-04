#pragma once

#include "../pch.h"
#include "..//Spring.h"

using namespace DirectX::SimpleMath;

namespace Animation
{
	class CharacterController
	{
	public:
		CharacterController();

		Vector3 position = Vector3::Zero;

		Vector3 velocity = Vector3::Zero;

		Vector3 acceleration = Vector3::Zero;

		Vector3 target_direction = Vector3::Zero;

		void Update(const Vector3& _direction, float _t);

	//private:
		void UpdateFacingDirection(const Vector3& direction, float t);

		void UpdateVelocity(const Vector3& direction, float t);

		void UpdateSpringCharacter(float& x, float& v, float& a, float v_goal, float halflife, float dt);

		Vector3 UpdateDesiredVelocity(
			const Vector3& gamepadstick_left,
			const float camera_azimuth,
			const Quaternion& simulation_rotation,
			const float fwrd_speed,
			const float side_speed,
			const float back_speed)
		{
			// Find stick position in world space by rotating using camera azimuth
			// TODO
			Vector3 global_stick_direction = gamepadstick_left;

			// Find stick position local to current facing direction
			// TODO: Maybe I can just use the quaternion to rotate v?
			Vector3 local_stick_direction = quat_inv_mul_vec3(
				simulation_rotation, global_stick_direction);

			// Scale stick by forward, sideways and backwards speeds
			Vector3 local_desired_velocity = local_stick_direction.z > 0.0f ? 
				Vector3(side_speed, 0.0f, fwrd_speed) * local_stick_direction :
				Vector3(side_speed, 0.0f, back_speed) * local_stick_direction;

			// Re-orientate into the world space
			return quat_mul_vec3(simulation_rotation, local_desired_velocity);
		}

		Quaternion UpdateDesiredRotation(
			const Quaternion& desired_rotation,
			const Vector3& gamepadstick_left,
			const Vector3& gamepadstick_right,
			const float camera_azimuth,
			const bool desired_strafe,
			const Vector3& desired_velocity)
		{
			Quaternion desired_rotaion_curr = desired_rotation;

			// If strafe is active then desired direction is coming from right
			// stick as long as that stick is being used, otherwise we assume
			// forward facing
			if (desired_strafe)
			{
				// TODO
			}
			// If strafe is not active the desired direction comes from the left 
			// stick as long as that stick is being used
			else 
			{
				Vector3 desired_direction = desired_velocity.Normalized();
				return Quaternion::CreateFromAxisAngle(Vector3(0, 1, 0), atan2f(desired_direction.x, desired_direction.z));
			}


			return desired_rotaion_curr;
		}
		

		void UpdateSimulationPositions(
			Vector3& position,
			Vector3& velocity,
			Vector3& acceleration,
			const Vector3& desired_velocity,
			const float halflife,
			const float dt);

		void UpdateSimulationRotations(
			Quaternion& rotation,
			Vector3& angular_velocity,
			const Quaternion& desired_rotation,
			const float halflife,
			const float dt) {
			simple_spring_damper_implicit(
				rotation,
				angular_velocity,
				desired_rotation,
				halflife,
				dt);
		}
			
		// Predict what the desired velocity will be in the 
		// future. Here we need to use the future trajectory 
		// rotation as well as predicted future camera 
		// position to find an accurate desired velocity in 
		// the world space

		void PredictTrajectoryDesiredVelocities(
			std::vector<Vector3>& desired_velocities,
			const std::vector<Quaternion> trajectory_rotations,
			const Vector3& desired_velocity,
			const float camera_azimuth,
			const Vector3& gamepadstick_left,
			const Vector3& gamepadstick_right,
			const bool desired_strafe,
			const float fwrd_speed,
			const float side_speed,
			const float back_speed,
			const float dt)
		{
			desired_velocities[0] = desired_velocity;

			for (int i = 1; i < desired_velocities.size(); i++)
			{
				desired_velocities[i] = UpdateDesiredVelocity(
					gamepadstick_left,
					0, // TODO
					trajectory_rotations[i],
					fwrd_speed,
					side_speed,
					back_speed);
			}
		}

		void PredictTrajectoryPosition(
			std::vector<Vector3>& positions,
			std::vector<Vector3>& velocities,
			std::vector<Vector3>& accelerations,
			const Vector3& position,
			const Vector3& velocity,
			const Vector3& acceleration,
			const std::vector<Vector3>& desired_velocities,
			const float halflife,
			const float dt) {
			positions[0] = position;
			velocities[0] = velocity;
			accelerations[0] = acceleration;

			for (int i = 1; i < positions.size(); i++)
			{
				positions[i] = positions[i - 1];
				velocities[i] = velocities[i - 1];
				accelerations[i] = accelerations[i - 1];

				UpdateSimulationPositions(
					positions[i],
					velocities[i],
					accelerations[i],
					desired_velocities[i],
					halflife,
					dt);
			}
		}

		// Predict desired rotations given the estimated future 
		// camera rotation and other parameters
		void PredictTrajectoryDesiredRotations(
			std::vector<Quaternion>& desired_rotations,
			const std::vector<Vector3>& desired_velocities,
			const Quaternion& desired_rotation,
			const float camera_azimuth,
			const Vector3& gamepadstick_left,
			const Vector3& gamepadstick_right,
			const bool desired_strafe,
			const float dt) {
			desired_rotations[0] = desired_rotation;

			for (int i = 1; i < desired_rotations.size(); i++)
			{
				desired_rotations[i] = UpdateDesiredRotation(
					desired_rotations[i - 1],
					gamepadstick_left,
					gamepadstick_right,
					0,//TODO
					desired_strafe,
					desired_velocities[i]);
			}
		}

		void PredictTrajectoryRotations(
			std::vector<Quaternion>& rotations,
			std::vector<Vector3>& angular_velocities,
			const Quaternion& rotation,
			const Vector3& angular_velocity,
			const std::vector<Quaternion>& desired_rotations,
			const float halflife,
			const float dt)
		{
			rotations[0] = rotation;
			angular_velocities[0] = angular_velocity;

			for (int i = 1; i < rotations.size(); i++)
			{
				UpdateSimulationRotations(
					rotations[i],
					angular_velocities[i],
					desired_rotations[i],
					halflife,
					i * dt);
			}
		}

		void Update(float t)
		{
			float camera_azimuth = 0.0f;

			bool desired_strafe = false;

			// TODO
			float simulation_fwrd_speed = simulation_run_fwrd_speed;
			float simulation_side_speed = simulation_run_side_speed;
			float simulation_back_speed = simulation_run_back_speed;
		
			// Get the desired velocity
			Vector3 desired_velocity_curr = UpdateDesiredVelocity(
				gamepadstick_left,
				camera_azimuth,
				simulation_rotation,
				simulation_fwrd_speed,
				simulation_side_speed,
				simulation_back_speed);

			// Get the desired rotation/direction
			Quaternion desired_rotation_curr = UpdateDesiredRotation(
				desired_rotation,
				gamepadstick_left,
				gamepadstick_right,
				camera_azimuth,
				desired_strafe,
				desired_velocity_curr);

			// Check if we should force a search because input changed quickly
			desired_velocity_change_prev = desired_velocity_change_curr;
			desired_velocity_change_curr = (desired_velocity_curr - desired_velocity) / dt;
			desired_velocity = desired_velocity_curr;

			desired_rotation = desired_rotation_curr;

			// Predict Future Trajectory
			PredictTrajectoryDesiredRotations(
				trajectory_desired_rotations,
				trajectory_desired_velocities,
				desired_rotation,
				camera_azimuth,
				gamepadstick_left,
				gamepadstick_right,
				desired_strafe,
				20.0f * dt);

			PredictTrajectoryRotations(
				trajectory_rotations,
				trajectory_angular_velocities,
				simulation_rotation,
				simulation_angular_velocity,
				trajectory_desired_rotations,
				simulation_rotation_halflife,
				20.0f * dt);

			PredictTrajectoryDesiredVelocities(
				trajectory_desired_velocities,
				trajectory_rotations,
				desired_velocity,
				camera_azimuth,
				gamepadstick_left,
				gamepadstick_right,
				desired_strafe,
				simulation_fwrd_speed,
				simulation_side_speed,
				simulation_back_speed,
				20.0f * dt);

			PredictTrajectoryPosition(
				trajectory_positions,
				trajectory_velocities,
				trajectory_accelerations,
				simulation_position,
				simulation_velocity,
				simulation_acceleration,
				trajectory_desired_velocities,
				simulation_velocity_halflife,
				20.0f * dt);

			// Update Simulation
			Vector3 simulation_position_prev = simulation_position;

			UpdateSimulationPositions(
				simulation_position,
				simulation_velocity,
				simulation_acceleration,
				desired_velocity,
				simulation_velocity_halflife,
				dt);

			UpdateSimulationRotations(
				simulation_rotation,
				simulation_angular_velocity,
				desired_rotation,
				simulation_rotation_halflife,
				dt);
		}

		// Trajectory & Gameplay Data

		Vector3 desired_velocity;
		Vector3 desired_velocity_change_curr;
		Vector3 desired_velocity_change_prev;
		float desired_velocity_change_threshold = 50.0f; // TODO

		Quaternion desired_rotation;
		Vector3 desired_rotation_change_curr;
		Vector3 desired_rotation_change_prev;
		float desired_rotation_change_threshold = 50.0f; // TODO

		Vector3 simulation_position;
		Vector3 simulation_velocity;
		Vector3 simulation_acceleration;
		Quaternion simulation_rotation;
		Vector3 simulation_angular_velocity;

		float simulation_velocity_halflife = 0.5f;
		float simulation_rotation_halflife = 0.5f;

		// All speeds in m/s
		float simulation_run_fwrd_speed = 4.0f;
		float simulation_run_side_speed = 3.0f;
		float simulation_run_back_speed = 2.5f;

		float simulation_walk_fwrd_speed = 1.75f;
		float simulation_walk_side_speed = 1.5f;
		float simulation_walk_back_speed = 1.25f;

		int trajectory_points_size = 4;

		std::vector<Vector3> trajectory_desired_velocities;
		std::vector<Quaternion> trajectory_desired_rotations;
		std::vector<Vector3> trajectory_positions;
		std::vector<Vector3> trajectory_velocities;
		std::vector<Vector3> trajectory_accelerations;
		std::vector<Quaternion> trajectory_rotations;
		std::vector<Vector3> trajectory_angular_velocities;

		float dt = 1.0f / 60.0f; // TODO

		Vector3 gamepadstick_left;
		Vector3 gamepadstick_right;

		// -----------------
		float max_forward_speed = 1.0f;
		float max_backwards_speed = 1.0f;
		float max_sideways_speed = 1.0f;
		float max_velocity_change = 0.2f;

	};
}