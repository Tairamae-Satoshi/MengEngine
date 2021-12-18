#include "LocalToModelJob.h"

namespace Animation
{
	LocalToModelJob::LocalToModelJob()
		:skeleton(nullptr) {}

	bool LocalToModelJob::Validate() const
	{
		bool valid = true;

		if (!skeleton)
		{
			return false;
		}

		return valid;
	}


	bool LocalToModelJob::Run(bool local, bool offset)
	{
		if (!Validate())
		{
			return false;
		}

		int jointNum = skeleton->JointCount();
		output.clear();
		output.resize(jointNum);

		// The root bone has index 0.  The root bone has no parent, so its toRootTransform
		// is just its local bone transform.
		Vector3 scale = input[0].mScale.mValue;
		Vector3 translation;
		if (local)
			translation = Vector3(0.0f, input[0].mTrans.mValue.y, 0.0f);
		else
			translation = input[0].mTrans.mValue;
		Quaternion rotation = input[0].mRot.mValue;
		output[0] = Matrix::CreateAffineTransformation(scale, translation, rotation);

		// Now find the toRootTransform of the children.
		for (UINT i = 1; i < jointNum; ++i)
		{
			Vector3 scale = input[i].mScale.mValue;
			Vector3 translation = input[i].mTrans.mValue;
			Quaternion rotation = input[i].mRot.mValue;
			Matrix toParent = Matrix::CreateAffineTransformation(scale, translation, rotation);

			int parentIndex = skeleton->GetJointParentIndex(i);

			output[i] = toParent * output[parentIndex];
		}

		if (offset)
		{
			// Premultiply by the bone offset transform to get the final transform.
			for (UINT i = 0; i < jointNum; ++i)
			{
				Matrix offset = skeleton->GetJointOffset(i);
				Matrix toRoot = output[i];
				output[i] = (offset * toRoot).Transpose();
			}
		}
		

		return true;
	}
}