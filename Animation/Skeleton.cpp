#include "../pch.h"
#include "Skeleton.h"

using namespace DirectX;

namespace Animation
{
	const AnimationClip* Skeleton::GetAnimationClipByName(const std::string& clipName) const
	{
		return &(mAnimations.at(clipName));
	}

	float Skeleton::GetClipStartTime(const std::string& clipName)const
	{
		auto clip = mAnimations.find(clipName);
		return clip->second.GetClipStartTime();
	}

	float Skeleton::GetClipEndTime(const std::string& clipName)const
	{
		auto clip = mAnimations.find(clipName);
		return clip->second.GetClipEndTime();
	}

	UINT Skeleton::GetAnimationNum() const
	{
		return mAnimations.size();
	}


	UINT Skeleton::JointCount()const
	{
		return mJointHierarchy.size();
	}
	
	void Skeleton::Set(std::vector<int>& jointHierarchy,
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

	void Skeleton::SetBindPose(const AnimationClip* bindPose)
	{
		mBindPose = bindPose;
	}

	void Skeleton::AddAnimation(std::unordered_map<std::string, AnimationClip>& animations)
	{
		for (auto [name, animation] : animations)
		{
			mAnimations[name] = animation;
		}
	}

	const AnimationClip* Skeleton::AddAnimation(std::string name, AnimationClip animation)
	{
		mAnimations[name] = animation;
		return &(mAnimations.at(name));
	}


	std::string Skeleton::GetAnimationClipName(UINT index) const
	{
		std::unordered_map<std::string, AnimationClip>::const_iterator it = mAnimations.begin();
		for (UINT i = 0; i < mAnimations.size(); i++, it++)
		{
			if (i == index)	return it->first;
		}
		return "";
	}

	std::vector<int> Skeleton::GetJointChildrenIndex(int index) const
	{
		std::vector<int> childrenIndex;

		for (size_t i = 0; i < mJointHierarchy.size(); i++)
		{
			if (mJointHierarchy[i] == index)
				childrenIndex.push_back(i);
		}

		return childrenIndex;
	}

	int Skeleton::GetJointParentIndex(int i) const
	{
		return mJointHierarchy[i];
	}

	std::string Skeleton::GetJointName(int index) const
	{
		return mJointNames[index];
	}

	Matrix Skeleton::GetJointOffset(int index) const
	{
		return mJointOffsets[index];
	}

	const AnimationClip* Skeleton::GetBindPose() const
	{
		return mBindPose;
	}

	const std::vector<SkinnedVertex>& Skeleton::GetVertices() const
	{
		return mVertices;
	}


	void TraverseSkeletonHierachy(const Animation::Skeleton& skeleton, int jointIndex)
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

	void ShowSkeletonHierachy(const Animation::Skeleton& skeleton, int jointIndex)
	{
		if (ImGui::TreeNode("Skeleton Hierachy"))
		{
			TraverseSkeletonHierachy(skeleton, jointIndex);
			ImGui::TreePop();
		}
	}

	bool Skeleton::OnGui()
	{
		ShowSkeletonHierachy(*this, 0);

		return true;
	}

}