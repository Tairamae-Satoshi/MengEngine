#include "IKThreeBoneJob.h"

namespace Animation
{
	IKThreeBoneJob::IKThreeBoneJob()
	{
		joints.resize(4);
		joints_correction.resize(3);
	}

	struct IKThreeBoneConstantSetup
	{
		IKThreeBoneConstantSetup(const IKThreeBoneJob& job) {
			// Prepares constants

			// Computes inverse matrices required to change to start and mid spaces.
			inv_joint0 = job.joints[0].Invert();
			Matrix inv_joint1 = job.joints[1].Invert();
			Matrix inv_joint2 = job.joints[2].Invert();

			// Transform some positions to joint1 space (j1S)
			const Vector3 joint0_j1s = Vector3::Transform(job.joints[0].Translation(), inv_joint1);
			const Vector3 joint2_j1s = Vector3::Transform(job.joints[2].Translation(), inv_joint1);
			const Vector3 joint3_j1s = Vector3::Transform(job.joints[3].Translation(), inv_joint1);

			// Transform some positions to joint0 space (j0S)
			const Vector3 joint1_j0s = Vector3::Transform(job.joints[1].Translation(), inv_joint0);
			const Vector3 joint2_j0s = Vector3::Transform(job.joints[2].Translation(), inv_joint0);
			const Vector3 joint3_j0s = Vector3::Transform(job.joints[3].Translation(), inv_joint0);

			// Transform some positions to joint0 space (j2S)
			const Vector3 joint0_j2s = Vector3::Transform(job.joints[0].Translation(), inv_joint2);
			const Vector3 joint1_j2s = Vector3::Transform(job.joints[1].Translation(), inv_joint2);
			const Vector3 joint3_j2s = Vector3::Transform(job.joints[3].Translation(), inv_joint2);


			// Computes bones vectors and length in mid and start spaces.
			// Start joint position will be treated as 0 all joints are 
			// expressed in start joint space.
			joint0_joint1_j1s = -joint0_j1s;
			joint1_joint2_j1s = joint2_j1s;
			joint2_joint3_j1s = joint3_j1s - joint2_j1s;

			joint0_joint1_j0s = joint1_j0s;
			joint1_joint2_j0s = joint2_j0s - joint1_j0s;
			joint2_joint3_j0s = joint3_j0s - joint2_j0s;

			joint2_joint3_j2s = joint3_j2s;

			Vector3 joint0_joint3_j0s = joint3_j0s;
			joint0_joint1_j0s_len2 = joint0_joint1_j0s.LengthSquared();
			joint1_joint2_j0s_len2 = joint1_joint2_j0s.LengthSquared();
			joint2_joint3_j0s_len2 = joint2_joint3_j0s.LengthSquared();
			//joint0_joint3_j0s_len2 = joint0_joint3_j0s.LengthSquared();

			mid_ratio = sqrt(joint0_joint1_j0s_len2) / (sqrt(joint0_joint1_j0s_len2) + sqrt(joint2_joint3_j0s_len2));
			const Vector3 joint12_j0s = Vector3::Lerp(joint1_j0s, joint2_j0s, mid_ratio);
			joint1_joint12_j0s = joint12_j0s - joint1_j0s;
			joint12_joint2_j0s = joint2_j0s - joint12_j0s;
			joint1_joint12_j0s_len2 = joint1_joint12_j0s.LengthSquared();
			joint12_joint2_j0s_len2 = joint12_joint2_j0s.LengthSquared();
			joint0_joint12_j0s_len2 = joint12_j0s.LengthSquared();
			joint12_joint3_j0s_len2 = (joint3_j0s - joint12_j0s).LengthSquared();
		}

		Matrix inv_joint0;

		// Single Segment: 0-1, 1-2, 2-3
		// Bones vectors and length in mid and start spaces(_ms and _ss)
		Vector3 joint0_joint1_j0s;
		Vector3 joint1_joint2_j0s;
		Vector3 joint2_joint3_j0s;

		float joint0_joint1_j0s_len2;
		float joint1_joint2_j0s_len2;
		float joint2_joint3_j0s_len2;
		//float joint0_joint3_j0s_len2;

		Vector3 joint0_joint1_j1s;
		Vector3 joint1_joint2_j1s;
		Vector3 joint2_joint3_j1s;

		Vector3 joint2_joint3_j2s;

		// Mid point
		float mid_ratio;
		Vector3 joint1_joint12_j0s;
		Vector3 joint12_joint2_j0s;
		
		float joint1_joint12_j0s_len2;
		float joint12_joint2_j0s_len2;
		float joint0_joint12_j0s_len2;
		float joint12_joint3_j0s_len2;
	};

	Quaternion ComputeJoint1(const IKThreeBoneJob& _job,
		const IKThreeBoneConstantSetup& _setup,
		float joint0_midtarget_j0s_len2)
	{
		// c^2 = a^2 + b^2 - 2ab cosC
		// cosC = (a^2 + b^2 - c^2) / 2ab
		float joint0_joint1_joint12_sum_j0s_len2 = _setup.joint0_joint1_j0s_len2 + _setup.joint1_joint12_j0s_len2; // a^2 + b^2
		float joint0_joint1_joint12_j0s_len = sqrt(_setup.joint0_joint1_j0s_len2 * _setup.joint1_joint12_j0s_len2); // ab

		float joint1_cos_angle_unclamped = (joint0_joint1_joint12_sum_j0s_len2 - joint0_midtarget_j0s_len2) / (2.0f * joint0_joint1_joint12_j0s_len);
		float joint1_cos_angle = MathHelper::Clamp(joint1_cos_angle_unclamped, -1.0f, 1.0f);
		float joint1_corrected_angle = acos(joint1_cos_angle);

		float joint1_initial_cos_unclamped = (joint0_joint1_joint12_sum_j0s_len2 - _setup.joint0_joint12_j0s_len2) / (2.0f * joint0_joint1_joint12_j0s_len);
		float joint1_initial_cos = MathHelper::Clamp(joint1_initial_cos_unclamped, -1.0f, 1.0f);
		float joint1_initial_angle = acos(joint1_initial_cos);

		float joint1_diff_angle = joint1_corrected_angle - joint1_initial_angle;

		Vector3 mid_axis_j1s = Vector3::TransformVector(_job.mid_axis, _job.joints[1].Invert());

		return Quaternion::CreateFromAxisAngle(mid_axis_j1s, joint1_diff_angle);
		
	}

	Quaternion ComputeJoint2(const IKThreeBoneJob& _job,
		const IKThreeBoneConstantSetup& _setup,
		float midtarget_joint3_j0s_len2)
	{
		float joint12_joint2_joint3_sum_j0s_len2 = _setup.joint12_joint2_j0s_len2 + _setup.joint2_joint3_j0s_len2;
		float joint12_joint2_joint3_j0s_len = sqrt(_setup.joint12_joint2_j0s_len2 * _setup.joint2_joint3_j0s_len2);

		float joint2_cos_angle_unclamped = (joint12_joint2_joint3_sum_j0s_len2 - midtarget_joint3_j0s_len2) / (2.0f * joint12_joint2_joint3_j0s_len);
		float joint2_cos_angle = MathHelper::Clamp(joint2_cos_angle_unclamped, -1.0f, 1.0f);
		float joint2_corrected_angle = acos(joint2_cos_angle);

		float joint2_initial_cos_unclamped = (joint12_joint2_joint3_sum_j0s_len2 - _setup.joint12_joint3_j0s_len2) / (2.0f * joint12_joint2_joint3_j0s_len);
		float joint2_initial_cos = MathHelper::Clamp(joint2_initial_cos_unclamped, -1.0f, 1.0f);
		float joint2_initial_angle = acos(joint2_initial_cos);

		float joint2_diff_angle = joint2_corrected_angle - joint2_initial_angle;

		Vector3 mid_axis_j2s = Vector3::TransformVector(-_job.mid_axis, _job.joints[2].Invert());

		return Quaternion::CreateFromAxisAngle(mid_axis_j2s, joint2_diff_angle);
	}

	Quaternion ComputeJoint0(const IKThreeBoneJob& _job,
		const IKThreeBoneConstantSetup& _setup,
		const Quaternion& joint1_rot_j1s,
		const Quaternion& joint2_rot_j2s,
		Vector3 _start_target_j0s,
		float _start_target_j0s_len2)
	{
		Vector3 joint2_joint3_j2s_final = Vector3::Transform(_setup.joint2_joint3_j2s, joint2_rot_j2s);
		Vector3 joint2_joint3_ms_final = Vector3::TransformVector(joint2_joint3_j2s_final, _job.joints[2]); // In model space
		Vector3 joint2_joint3_j1s_final = Vector3::TransformVector(joint2_joint3_j2s_final, _job.joints[1].Invert());
		joint2_joint3_j1s_final = Vector3::Transform(joint2_joint3_j1s_final, joint1_rot_j1s);
		joint2_joint3_ms_final = Vector3::TransformVector(joint2_joint3_j1s_final, _job.joints[1]);
		Vector3 joint2_joint3_j0s_final = Vector3::TransformVector(joint2_joint3_ms_final, _setup.inv_joint0);

		Vector3 joint1_joint2_j1s_final = Vector3::Transform(_setup.joint1_joint2_j1s, joint1_rot_j1s);
		Vector3 joint1_joint2_ms_final = Vector3::TransformVector(joint1_joint2_j1s_final, _job.joints[1]); // In model space
		Vector3 joint1_joint2_j0s_final = Vector3::TransformVector(joint1_joint2_ms_final, _setup.inv_joint0);

		Vector3 joint0_joint3_j0s_final = _setup.joint0_joint1_j0s + joint1_joint2_j0s_final + joint2_joint3_j0s_final;

		Quaternion end_to_target_rot_j0s = Quaternion::CreateFromVectors(joint0_joint3_j0s_final.Normalized(), _start_target_j0s.Normalized());

		Quaternion joint0_rot_j0s = end_to_target_rot_j0s;
		if (_start_target_j0s_len2 > 0)
		{
			Vector3 pole_j0s = Vector3::TransformVector(_job.pole_vector, _setup.inv_joint0);
			Vector3 ref_plane_normal_j0s = pole_j0s.Cross(_start_target_j0s);

			Vector3 mid_axis_j0s = Vector3::TransformVector(_job.mid_axis, _setup.inv_joint0);
			Vector3 joint_plane_normal_j0s = Vector3::Transform(mid_axis_j0s, end_to_target_rot_j0s);

			float rotate_plane_angle = Vector3::Angle(ref_plane_normal_j0s, joint_plane_normal_j0s);

			Vector3 rotate_plane_axis_j0s = _start_target_j0s.Normalized();
			int start_axis_flip = joint_plane_normal_j0s.Dot(pole_j0s) > 0 ? -1 : 1;
			Vector3 rotate_plane_axis_flipped_j0s = start_axis_flip * rotate_plane_axis_j0s;

			Quaternion rotate_plane_j0s = Quaternion::CreateFromAxisAngle(
				rotate_plane_axis_flipped_j0s,
				rotate_plane_angle);

			joint0_rot_j0s = end_to_target_rot_j0s * rotate_plane_j0s;
		}
		return joint0_rot_j0s;
	}

	bool IKThreeBoneJob::Run()
	{
		const IKThreeBoneConstantSetup setup(*this);

		Vector3 joint0_joint3_j0s;
		float joint0_joint3_j0s_len2;
		//reached = 


		Vector3 joint0_target_j0s = Vector3::Transform(target, setup.inv_joint0);
		Quaternion joint1_rot_j1s = ComputeJoint1(*this, setup, setup.mid_ratio * setup.mid_ratio * joint0_target_j0s.LengthSquared());
		Quaternion joint2_rot_j2s = ComputeJoint2(*this, setup, (1 - setup.mid_ratio) * (1 - setup.mid_ratio) * joint0_target_j0s.LengthSquared());
		Quaternion joint0_rot_j0s = ComputeJoint0(*this, setup, joint1_rot_j1s, joint2_rot_j2s, joint0_target_j0s, joint0_target_j0s.LengthSquared());
		joints_correction[0] = joint0_rot_j0s;
		joints_correction[1] = joint1_rot_j1s;
		joints_correction[2] = joint2_rot_j2s;

		return true;
	}
}