#include"IKAimJob.h"

namespace Animation
{
	IKAimJob::IKAimJob():
		target(Vector3::Zero),
		forward(Vector3::Zero),
		offset(Vector3::Zero),
		up(Vector3::Zero),
		pole_vector(Vector3::Zero),
		twist_angle(0.0f),
		weight(1.0f),
		joint(nullptr),
		joint_correction(nullptr),
		reached(true){}

	bool IKAimJob::Validate() const {
		bool valid = true;
		valid = abs(forward.LengthSquared() - 1.0f) < 0.000001 ? valid : false;
		return valid;
	}

	// When there's an offset, the forward vector needs to be recomputed.
	// The idea is to find the vector that will allow the point at offset position
	// to aim at target position. This vector starts at joint position. It ends on a
	// line perpendicular to pivot-offset line, at the intersection with the sphere
	// defined by target position (centered on joint position). 
	bool ComputeOffsettedForward(Vector3 _forward, Vector3 _offset, Vector3 _target, Vector3* _offsetted_forward)
	{
		// AO is projected offset vector onto the normalized forward vector.
		_forward.Normalize();
		float AOl = _forward.Dot(_offset);

		// Compute square length of ac using Pythagorean theorem.
		float ACl2 = _offset.LengthSquared() - AOl * AOl;

		// Square length of target vector, aka circle radius.
		float r2 = _target.LengthSquared();

		// If offset is outside of the sphere defined by target length, the target
		// isn't reachable.
		if (ACl2 > r2){
			return false;
		}

		// AIl is the length of the vector from offset to sphere intersection.
		float AIl = sqrt(r2 - ACl2);

		// The distance from offset position to the intersection with the sphere is
		// (AIl - AOl) Intersection point on the sphere can thus be computed.
		*_offsetted_forward = _offset + _forward * (AIl - AOl);
	}

	bool IKAimJob::Run() {
		if (!Validate()){
			return false;
		}

		Matrix inv_joint = joint->Invert();

		// Computes joint to target vector, in joint local-space (_js).
		Vector3 joint_to_target_js = Vector3::Transform(target, inv_joint);

		// Recomputes forward vector to account for offset.
		// If the offset is further than target, it won't be reachable.
		Vector3 offsetted_forward;
		reached = ComputeOffsettedForward(forward, offset, joint_to_target_js,
			&offsetted_forward);


		if (!reached || joint_to_target_js.LengthSquared() == 0){
			*joint_correction = Quaternion::Identity;
			return true;
		}

		// Calculates joint_to_target_rot_js quaternion which solves for
		// offsetted_forward vector rotating onto the target.
		Quaternion joint_to_target_rot_js = Quaternion::CreateFromVectors(offsetted_forward, joint_to_target_js);

		// Calculate rotate_plane_js quaternion which aligns joint up tp the pole
		// vector.
		Vector3 corrected_up_js = Vector3::Transform(up, joint_to_target_rot_js);

		// Compute (and normalize) reference and pole planes normals.
		Vector3 pole_vector_js = Vector3::TransformVector(pole_vector, inv_joint);
		Vector3 ref_joint_normal_js = pole_vector_js.Cross(joint_to_target_js);
		Vector3 joint_normal_js = corrected_up_js.Cross(joint_to_target_js);

		Vector3 rotate_plane_axis_js;
		Quaternion rotate_plane_js;
		// Computing rotation axis and plane requires valid normals.
		if (joint_to_target_js.LengthSquared() != 0 && joint_normal_js.LengthSquared() != 0 && ref_joint_normal_js.LengthSquared() != 0)
		{
			// Computes rotation axis, which is either joint_to_target_js or
			// -joint_to_target_js depending on rotation direction.
			rotate_plane_axis_js = joint_to_target_js.Normalized();

			// Computes angle cosine between the 2 normalized plane normals.
			float rotate_plane_cos_angle = joint_normal_js.Normalized().Dot(ref_joint_normal_js.Normalized());
			float axis_flip = ref_joint_normal_js.Dot(corrected_up_js) > 0 ? 1.0f : -1.0f;
			Vector3 rotate_plane_axis_flipped_js = axis_flip * rotate_plane_axis_js;

			// Builds quaternion along rotation axis.
			float rotate_plane_angle = acos(MathHelper::Clamp(rotate_plane_cos_angle, -1.0f, 1.0f));
			rotate_plane_js = Quaternion::CreateFromAxisAngle(rotate_plane_axis_flipped_js, rotate_plane_angle);
		}
		else{
			rotate_plane_axis_js = joint_to_target_js.Normalized();
			rotate_plane_js = Quaternion::Identity;
		}

		// Twists rotation plane.
		Quaternion twisted;
		if (twist_angle != 0.0f){

		}
		else{
			twisted = joint_to_target_rot_js * rotate_plane_js;// FIXME
		}

		// Weights output Quaternion.

		// Fix up quaternions so w is always positive, which is required for NLerp
		// (with identity quaternion) to lerp the shortest path.
		Quaternion twisted_fu = twisted.w >= 0.0f ? twisted : -twisted;

		if (weight < 1.0f){
			*joint_correction = Quaternion::Slerp(Quaternion::Identity, twisted_fu, weight);
		}
		else{
			*joint_correction = twisted_fu;
		}
	}
}