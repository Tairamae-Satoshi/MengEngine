#include "../pch.h"
#include "AnimationDatabase.h"

using namespace DirectX;

namespace Animation
{
	const AnimationClip* AnimationDatabase::GetAnimationClipByName(const std::string& clipName) const
	{
		return &(mAnimations.at(clipName));
	}

	float AnimationDatabase::GetClipStartTime(const std::string& clipName)const
	{
		auto clip = mAnimations.find(clipName);
		return clip->second.get_clip_start_time();
	}

	float AnimationDatabase::GetClipEndTime(const std::string& clipName)const
	{
		auto clip = mAnimations.find(clipName);
		return clip->second.get_clip_end_time();
	}

	UINT AnimationDatabase::GetAnimationNum() const
	{
		return mAnimations.size();
	}


	UINT AnimationDatabase::JointCount()const
	{
		return mJointHierarchy.size();
	}
	
	void AnimationDatabase::Set(std::vector<int>& jointHierarchy,
		std::vector<std::string>& jointNames,
		std::vector<XMFLOAT4X4>& jointOffsets,
		std::vector<SkinnedVertex> vertices,
		std::unordered_map<std::string, AnimationClip>& animations)
	{
		mJointHierarchy = jointHierarchy;
		mJointNames = jointNames;
		mJointOffsets = jointOffsets;
		mVertices = vertices;
		mBindPose = animations.begin()->second;
	}

	//void AnimationDatabase::SetBindPose(const AnimationClip* bindPose)
	//{
	//	mBindPose = bindPose;
	//}

	void AnimationDatabase::AddAnimation(std::unordered_map<std::string, AnimationClip>& animations)
	{
		for (auto [name, animation] : animations)
		{
			AddAnimation(name, animation);
		}
	}

	const AnimationClip* AnimationDatabase::AddAnimation(std::string name, AnimationClip animation)
	{
		offsets[name] = totalPoseCount;
		totalPoseCount += animation.get_pose_count();
		for (int i = 0; i < animation.get_pose_count(); i++)
		{
			rangeStarts.push_back(offsets[name]);
			rangeStops.push_back(totalPoseCount);
		}
		animation_names.push_back(name);
		mAnimations[name] = animation;
		return &(mAnimations.at(name));
	}


	std::string AnimationDatabase::GetAnimationClipName(UINT index) const
	{
		std::unordered_map<std::string, AnimationClip>::const_iterator it = mAnimations.begin();
		for (UINT i = 0; i < mAnimations.size(); i++, it++)
		{
			if (i == index)	return it->first;
		}
		return "";
	}

	std::string AnimationDatabase::GetAnimationClipNameByPoseId(int poseId) const
	{
		int rangeStart = rangeStarts[poseId];

		for (auto offset: offsets)
		{
			if (rangeStart == offset.second)
			{
				return offset.first;				 
			}
		}
	}

	std::vector<Transform> AnimationDatabase::GetTransformsAtPoseId(int poseId) const
	{
		std::string name = GetAnimationClipNameByPoseId(poseId);
		std::vector<Transform> transforms;
		int rangeStart = rangeStarts[poseId];
		
		return mAnimations.at(name).get_transform_at(poseId - rangeStarts[poseId]);
	}

	Vector3 AnimationDatabase::GetBonePosition(int poseId, int boneId) const
	{
		std::string name = GetAnimationClipNameByPoseId(poseId);
		int frameId = poseId - rangeStarts[poseId];
		return mAnimations.at(name).mSamples[boneId].mLocalPose[frameId].mTrans.mValue;
	}

	Quaternion AnimationDatabase::GetBoneRotation(int poseId, int boneId) const
	{
		std::string name = GetAnimationClipNameByPoseId(poseId);
		int frameId = poseId - rangeStarts[poseId];
		return mAnimations.at(name).mSamples[boneId].mLocalPose[frameId].mRot.mValue;
	}


	std::vector<int> AnimationDatabase::GetJointChildrenIndex(int index) const
	{
		std::vector<int> childrenIndex;

		for (size_t i = 0; i < mJointHierarchy.size(); i++)
		{
			if (mJointHierarchy[i] == index)
				childrenIndex.push_back(i);
		}

		return childrenIndex;
	}

	int AnimationDatabase::GetJointParentIndex(int i) const
	{
		return mJointHierarchy[i];
	}

	const std::vector<int>& AnimationDatabase::GetParentIndex() const
	{
		return mJointHierarchy;
	}

	std::string AnimationDatabase::GetJointName(int index) const
	{
		return mJointNames[index];
	}

	Matrix AnimationDatabase::GetJointOffset(int index) const
	{
		return mJointOffsets[index];
	}

	const AnimationClip* AnimationDatabase::GetBindPose() const
	{
		return &mBindPose;
	}

	const std::vector<SkinnedVertex>& AnimationDatabase::GetVertices() const
	{
		return mVertices;
	}

	int AnimationDatabase::ClampDatabaseTrajectoryIndex(int frame, int offset) const
	{
		for (int i = 0; i < this->rangeStarts.size(); i++)
		{
			if (frame >= this->rangeStarts[i] && frame < this->rangeStops[i])
			{
				return clamp(frame + offset, this->rangeStarts[i], this->rangeStops[i] - 1);
			}
		}

		assert(false);
	}

	void AnimationDatabase::convert_to_fps(float fps)
	{
		totalPoseCount = 0;
		rangeStarts.clear();
		rangeStops.clear();
		for (auto name : animation_names)
		{
			mAnimations.at(name).convert_to_fps(60.0f);
			offsets[name] = totalPoseCount;
			totalPoseCount += mAnimations.at(name).get_pose_count();
			for (int i = 0; i < mAnimations.at(name).get_pose_count(); i++)
			{
				rangeStarts.push_back(offsets[name]);
				rangeStops.push_back(totalPoseCount);
			}
		}
	}


	void TraverseSkeletonHierachy(const Animation::AnimationDatabase& skeleton, int jointIndex)
	{
		std::string jointName = skeleton.GetJointName(jointIndex);
		if (ImGui::TreeNode(jointName.data()))
		{
			std::vector<int> childrenIndex = skeleton.GetJointChildrenIndex(jointIndex);
			for (auto index : childrenIndex)
			{
				TraverseSkeletonHierachy(skeleton, index);
			}
			ImGui::TreePop();
		}
	}

	void ShowSkeletonHierachy(const Animation::AnimationDatabase& skeleton, int jointIndex)
	{
		if (ImGui::TreeNode("Skeleton Hierachy"))
		{
			TraverseSkeletonHierachy(skeleton, jointIndex);
			ImGui::TreePop();
		}
	}

	bool AnimationDatabase::OnGui()
	{
		ShowSkeletonHierachy(*this, 0);

		return true;
	}

}