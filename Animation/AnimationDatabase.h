#pragma once
#include "Animation.h"
#include "../pch.h"
#include "Common.h"

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

	class AnimationDatabase
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

		//void SetBindPose(const AnimationClip* bindPose);

		void AddAnimation(std::unordered_map<std::string, AnimationClip>& animations);

		const AnimationClip* AddAnimation(std::string name, AnimationClip animation);

		std::string GetAnimationClipName(UINT i) const;

		std::string GetAnimationClipNameByPoseId(int poseId) const;

		std::vector<Transform> GetTransformsAtPoseId(int poseId) const;

		Vector3 GetBonePosition(int poseId, int boneId) const;

		Quaternion GetBoneRotation(int poseId, int boneId) const;

		std::vector<int> GetJointChildrenIndex(int index) const;

		int GetJointParentIndex(int i) const;

		std::string GetJointName(int index) const;

		Matrix GetJointOffset(int index) const;

		const AnimationClip* GetBindPose() const;

		const std::vector<SkinnedVertex>& GetVertices() const;

		int ClampDatabaseTrajectoryIndex(int frame, int offset) const;

		void convert_to_fps(float fps);

		bool OnGui();

		int totalPoseCount = 0;

		std::vector<int> rangeStarts;

		std::vector<int> rangeStops;

		std::map<std::string, int> offsets;

		std::vector<int> mJointHierarchy;

	private:
		// Gives parentIndex of ith bone.

		std::vector<std::string> mJointNames;

		std::vector<DirectX::XMFLOAT4X4> mJointOffsets;

		AnimationClip mBindPose;

		std::vector<SkinnedVertex> mVertices;

		std::vector<std::string> animation_names;

		std::unordered_map<std::string, AnimationClip> mAnimations;


	};
}
