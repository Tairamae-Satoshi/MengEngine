#pragma once
#include "Animation.h"
#include "../pch.h"

using namespace DirectX::SimpleMath;

namespace Animation
{
#define NUM_BONES_PER_VEREX 4

	struct SkinnedVertex
	{
		Vector3 Pos;
		Vector3 Normal;
		Vector3 TangentU;
		Vector2 TexC;
		Vector3 BoneWeights;
		BYTE BoneIndices[NUM_BONES_PER_VEREX];
	};

	class Skeleton
	{
	public:
		UINT JointCount()const;

		const AnimationClip* GetAnimationClipByName(const std::string& clipName) const;
		float GetClipStartTime(const std::string& clipName)const;
		float GetClipEndTime(const std::string& clipName)const;
		UINT GetAnimationNum() const;

		void Set(
			std::vector<int>& jointHierarchy,
			std::vector<std::string>& jointNames,
			std::vector<DirectX::XMFLOAT4X4>& jointOffsets,
			std::vector<SkinnedVertex> vertices,
			std::unordered_map<std::string, AnimationClip>& animations);

		void SetBindPose(const AnimationClip* bindPose);

		void AddAnimation(std::unordered_map<std::string, AnimationClip>& animations);

		const AnimationClip* AddAnimation(std::string name, AnimationClip animation);

		std::string GetAnimationClipName(UINT i) const;

		std::vector<int> GetJointChildrenIndex(int index) const;

		int GetJointParentIndex(int i) const;

		std::string GetJointName(int index) const;

		Matrix GetJointOffset(int index) const;

		const AnimationClip* GetBindPose() const;

		const std::vector<SkinnedVertex>& GetVertices() const;

		bool OnGui();

	private:
		// Gives parentIndex of ith bone.
		std::vector<int> mJointHierarchy;

		std::vector<std::string> mJointNames;

		std::vector<DirectX::XMFLOAT4X4> mJointOffsets;

		const AnimationClip* mBindPose;

		std::vector<SkinnedVertex> mVertices;

		std::unordered_map<std::string, AnimationClip> mAnimations;
	};
}
