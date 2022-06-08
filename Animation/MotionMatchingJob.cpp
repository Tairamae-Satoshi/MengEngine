#include "MotionMatchingJob.h"

namespace Animation
{
	void ForwardKinematics(
		Vector3& bone_position,
		Quaternion& bone_rotation,
		const std::vector<Transform>& bone_transforms,
		const std::vector<int>& bone_parents,
		const int bone)
	{
		if (bone_parents[bone] != -1)
		{
			Vector3 parent_position;
			Quaternion parent_rotation;
			ForwardKinematics(
				parent_position,
				parent_rotation,
				bone_transforms,
				bone_parents,
				bone_parents[bone]);

			bone_position = quat_mul_vec3(parent_rotation, bone_transforms[bone].mTrans.mValue) + parent_position;
			bone_rotation = parent_rotation * bone_transforms[bone].mRot.mValue;
		}
		else
		{
			bone_position = bone_transforms[bone].mTrans.mValue;
			bone_rotation = bone_transforms[bone].mRot.mValue;
		}
	}
}