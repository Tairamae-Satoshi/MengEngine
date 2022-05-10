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
		return clip->second.GetClipStartTime();
	}

	float AnimationDatabase::GetClipEndTime(const std::string& clipName)const
	{
		auto clip = mAnimations.find(clipName);
		return clip->second.GetClipEndTime();
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
		mAnimations = animations;
	}

	//void AnimationDatabase::SetBindPose(const AnimationClip* bindPose)
	//{
	//	mBindPose = bindPose;
	//}

	void AnimationDatabase::AddAnimation(std::unordered_map<std::string, AnimationClip>& animations)
	{
		for (auto [name, animation] : animations)
		{
			mAnimations[name] = animation;
		}
	}

	const AnimationClip* AnimationDatabase::AddAnimation(std::string name, AnimationClip animation)
	{
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
		return mBindPose;
	}

	const std::vector<SkinnedVertex>& AnimationDatabase::GetVertices() const
	{
		return mVertices;
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