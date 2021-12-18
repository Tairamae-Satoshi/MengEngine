#include "IKTwoBoneJob.h"

namespace Animation
{
	IKTwoBoneJob::IKTwoBoneJob()
		: target(Vector3::Zero),
		//mid_axis(Vector3::UnitZ),
		pole_vector(Vector3::UnitY),
		twist_angle(0.0f),
		soften(1.0f),
		weight(1.0f),
		start_joint(Matrix::Identity),
		mid_joint(Matrix::Identity),
		end_joint(Matrix::Identity),
		start_joint_correction(Quaternion::Identity),
		mid_joint_correction(Quaternion::Identity),
		reached(nullptr) {}

	bool IKTwoBoneJob::Validate() const
	{
		bool valid = true;

		return valid;
	}

	// Local data structure used to share constant data accross ik stages
	struct IKConstantSetup
	{
		IKConstantSetup(const IKTwoBoneJob& job) {
			// Prepares constants

			// Computes inverse matrices required to change to start and mid spaces.
			// If matrices aren't invertible, they'll be all 0 (ozz::math
			// implementation), which will result in identity correction quaternions.
			inv_start_joint = job.start_joint.Invert();
			Matrix inv_mid_joint = job.mid_joint.Invert();

			// Transform some positions to mid joint space (MS)
			Vector3 start_ms = Vector3::Transform(job.start_joint.Translation(), inv_mid_joint);
			Vector3 end_ms = Vector3::Transform(job.end_joint.Translation(), inv_mid_joint);

			// Transform some positions to start joint space (SS)
			const Vector3 mid_ss = Vector3::Transform(job.mid_joint.Translation(), inv_start_joint);
			const Vector3 end_ss = Vector3::Transform(job.end_joint.Translation(), inv_start_joint);

			// Computes bones vectors and length in mid and start spaces.
			// Start joint position will be treated as 0 all joints are 
			// expressed in start joint space.
			start_mid_ms = -start_ms;
			mid_end_ms = end_ms;
			start_mid_ss = mid_ss;
			mid_end_ss = end_ss - mid_ss;
			Vector3 start_end_ss = end_ss;
			start_mid_ss_len2 = start_mid_ss.LengthSquared();
			mid_end_ss_len2 = mid_end_ss.LengthSquared();
			start_end_ss_len2 = start_end_ss.LengthSquared();

			mid_axis = mid_end_ms.Cross(start_mid_ms).Normalized();
		}

		// constants

		// Inverse matrices
		Matrix inv_start_joint;

		// Bones vectors and length in mid and start spaces(_ms and _ss)
		Vector3 start_mid_ms;
		Vector3 mid_end_ms;
		Vector3 start_mid_ss;
		Vector3 mid_end_ss;
		float start_mid_ss_len2;
		float mid_end_ss_len2;
		float start_end_ss_len2;

		Vector3 mid_axis;
	};

	// Smoothen target position when it's further that a ratio of the joint chain
	// length, and start to target length isn't 0.
	bool SoftenTarget(const IKTwoBoneJob& _job, const IKConstantSetup& _setup,
		Vector3* _start_target_ss, float* _start_target_ss_len2)
	{
		// Hanlde position in start joint space (_ss)
		Vector3 start_target_original_ss = Vector3::Transform(_job.target, _setup.inv_start_joint);
		float start_target_original_ss_len2 = start_target_original_ss.LengthSquared();
		float start_mid_ss_len = sqrt(_setup.start_mid_ss_len2);
		float mid_end_ss_len = sqrt(_setup.mid_end_ss_len2);
		float start_target_original_ss_len = sqrt(start_target_original_ss_len2);
		float bone_len_diff_abs = abs(start_mid_ss_len - mid_end_ss_len);
		float bones_chain_len = start_mid_ss_len + mid_end_ss_len;
		float da = bones_chain_len * MathHelper::Clamp(_job.soften, 0.0f, 1.0f);
		float ds = bones_chain_len - da;

		// Sotftens target position if it is further than a ratio (_soften) of the
		// whole bone chain length. Needs to check also that ds and
		// start_target_original_ss_len2 are != 0, because they're used as a
		// denominator.
		if (start_target_original_ss_len > da &&
			start_target_original_ss_len > 0 &&
			start_target_original_ss_len > bone_len_diff_abs &&
			ds > 0) {
			// Finds interpolation ratio (aka alpha).
			float alpha = (start_target_original_ss_len - da) / ds;
			// Approximate an exponential function with : 1-(3^4)/(alpha+3)^4
			// The derivative must be 1 for x = 0, and y must never exceeds 1.
			// Negative x aren't used.
			float a = 3.0f / (alpha + 3.0f);
			float pow2 = a * a;
			float ratio = pow2 * pow2;

			// Recomputes start_target_ss vector and length.
			float start_target_ss_len = da + ds - ds * ratio;
			*_start_target_ss_len2 = start_target_ss_len * start_target_ss_len;
			*_start_target_ss = (start_target_ss_len / start_target_original_ss_len) * start_target_original_ss;
		}
		else {
			*_start_target_ss = start_target_original_ss;
			*_start_target_ss_len2 = start_target_original_ss_len2;
		}

		return true;
	}


	Quaternion ComputeMidJoint(const IKTwoBoneJob& _job,
		const IKConstantSetup& _setup,
		float _start_target_ss_Len2) {
		// Computes expected angle at mid_ss joint, using law of cosine (generalized
		// Pythagorean).
		// c^2 = a^2 + b^2 - 2ab cosC
		// cosC = (a^2 + b^2 - c^2) / 2ab
		// Computes both corrected and initial mid joint angles
		// cosine within a single SimdFloat4 (corrected is x component, initial is y).
		float start_mid_end_sum_ss_len2 = _setup.start_mid_ss_len2 + _setup.mid_end_ss_len2; // a^2 + b^2
		float start_mid_end_ss_len = sqrt(_setup.start_mid_ss_len2 * _setup.mid_end_ss_len2); // ab

		// Cos value needs to be clamped, as it will exit expected range if
		// start_target_ss_len2 is longer than the triangle can be (start_mid_ss +
		// mid_end_ss).
		float mid_cos_angle_unclamped = (start_mid_end_sum_ss_len2 - _start_target_ss_Len2) / (2.0f * start_mid_end_ss_len);
		float mid_cos_angle = MathHelper::Clamp(mid_cos_angle_unclamped, -1.0f, 1.0f);

		// Compute corrected angle
		float mid_corrected_angle = acos(mid_cos_angle);

		// Compute initial angle
		// The sign of this angle needs to be decided. It's considered negative if
		// mid-to-end joint is bent backward (mid_axis direction dictates valid
		// bent direction).
		// TODO: I'm not sure is this necessary?
		/*Vector3 bent_side_ref = -_setup.start_mid_ms.Cross(_setup.mid_axis);
		int bent_side_flip = bent_side_ref.Dot(_setup.mid_end_ms) > 0 ? -1 : 1;*/
		
		float mid_initial_cos = (start_mid_end_sum_ss_len2 - _setup.start_end_ss_len2) / (2.0f * start_mid_end_ss_len);

		float mid_initial_angle = acos(mid_initial_cos);
		float mid_diff_angle = mid_corrected_angle - mid_initial_angle;

		return Quaternion::CreateFromAxisAngle(_setup.mid_axis, /*bent_side_flip **/ mid_diff_angle);
	}

	Quaternion ComputeStartJoint(const IKTwoBoneJob& _job,
		const IKConstantSetup& _setup,
		const Quaternion& _mid_rot_ms,
		Vector3 _start_target_ss,
		float _start_target_ss_len2)
	{
		// start_mid_ss with quaternion mid_rot_ms applied
		Vector3 mid_end_ms_final = Vector3::Transform(_setup.mid_end_ms, _mid_rot_ms);
		Vector3 mid_end_model_final = Vector3::TransformVector(mid_end_ms_final, _job.mid_joint) ;
		Vector3 mid_end_ss_final = Vector3::TransformVector(mid_end_model_final, _setup.inv_start_joint) ;
		Vector3 start_end_ss_final = _setup.start_mid_ss + mid_end_ss_final;
		Vector3 start_end_ms_final = _setup.start_mid_ms + mid_end_ms_final;

		// Quaternion for rotating the effector onto the target
		Quaternion end_to_target_rot_ss = Quaternion::CreateFromVectors(start_end_ss_final.Normalized(), _start_target_ss.Normalized());

		// Calculates rotate_plane_ss quaternion which aligns joint chain plane to
		// the reference plane (pole vector). This can only be computed if start
		// target axis is valid (not 0 length)
		Quaternion start_rot_ss = end_to_target_rot_ss;
		if (_start_target_ss_len2 > 0)
		{
			// Pole vector in start joint space (_ss)
			Vector3 pole_ss = mid_end_ss_final;

			// Compute each plane normal
			Vector3 ref_plane_normal_ss = pole_ss.Cross(_start_target_ss);

			// Compute joint chain plane normal, which is the same as mid joint axis
			// (same triangle).
			Vector3 mid_axis_ss = Vector3::TransformVector(
				Vector3::TransformVector(_setup.mid_axis,
					_job.mid_joint),
				_setup.inv_start_joint);
			Vector3 joint_plane_normal_ss = Vector3::Transform(
				mid_axis_ss,
				end_to_target_rot_ss);

			// Computes angle cosine between 2 normalized normals
			float rotate_plane_cos_angle = ref_plane_normal_ss.Normalized().Dot(
				joint_plane_normal_ss.Normalized());

			// Computes rotation axis, which is either start_target_ss or
			// -start_target_ss depending on rotation direction.
			Vector3 rotate_plane_axis_ss = _start_target_ss.Normalized();
			int start_axis_flip = joint_plane_normal_ss.Dot(pole_ss) < 0 ? 1 : -1;
			Vector3 rotate_plane_axis_flipped_ss = start_axis_flip * rotate_plane_axis_ss;

			// Builds quaternion along rotation axis.
			Quaternion rotate_plane_ss = Quaternion::CreateFromAxisAngle(
				rotate_plane_axis_flipped_ss,
				acos(rotate_plane_cos_angle));

			if (_job.twist_angle != 0.0f){
				// If a twist angle is provided, rotation angle is rotated along
				// rotation plane axis.
				Quaternion twist_ss = Quaternion::CreateFromAxisAngle(rotate_plane_axis_ss, _job.twist_angle);
				start_rot_ss = end_to_target_rot_ss * rotate_plane_ss * twist_ss;
			}
			else{
				// Note the order of multiplication
				start_rot_ss =  end_to_target_rot_ss * rotate_plane_ss ;
			}
		}
		return start_rot_ss;
	}

	void WeightOutput(IKTwoBoneJob& _job, const IKConstantSetup& _setup,
		const Quaternion& _start_rot,
		const Quaternion& _mid_rot) {
		// Fix up quaternions so w is always positive, which is required for NLerp
		// (with identity quaternion) to lerp the shortest path.
		Quaternion start_rot_fu = _start_rot.w >= 0.0 ? _start_rot : -_start_rot;
		Quaternion mid_rot_fu = _mid_rot.w >= 0.0 ? _mid_rot : -_mid_rot;

		if (_job.weight < 1.0f){
			// SLerp start and mid joint rotations.
			Quaternion identity = Quaternion::Identity;
			Quaternion start_slerp = Quaternion::Slerp(Quaternion::Identity, start_rot_fu, _job.weight);
			Quaternion mid_slerp = Quaternion::Slerp(Quaternion::Identity, mid_rot_fu, _job.weight);
			
			_job.start_joint_correction = start_slerp;
			_job.mid_joint_correction = mid_slerp;
		}
		else{
			// Quatenions don't need interpolation
			_job.start_joint_correction = _start_rot;
			_job.mid_joint_correction = _mid_rot;
		}

	}

	bool IKTwoBoneJob::Run()
	{
		if (!Validate()) {
			return false;
		}

		// Early out if weight is 0.
		if (weight <= 0.0f) {
			start_joint_correction = Quaternion::Identity;
			mid_joint_correction = Quaternion::Identity;
			// Target isn't reached.
			reached = false;
			return true;
		}

		// Prepares constant ik data.
		const IKConstantSetup setup(*this);

		// Finds soften target position
		Vector3 start_target_ss;
		float start_target_ss_len2;
		bool lreached = SoftenTarget(*this, setup, &start_target_ss, &start_target_ss_len2);
		reached = lreached && weight >= 1.0f;// FIXME

		// Calculate mid_rot_local quaternion which solves for the midStartSpace joint rotation.
		const Quaternion mid_rot_ss = ComputeMidJoint(*this, setup, start_target_ss_len2);

		// Calculate end_to_target_rot_ss quaternion which solves for the effector rotating onto the target.
		const Quaternion start_rot_ss = ComputeStartJoint(
			*this, setup, mid_rot_ss, start_target_ss, start_target_ss_len2);

		// Finally apply weigh and output quaternions
		WeightOutput(*this, setup, start_rot_ss, mid_rot_ss);

		return true;
	}
}