#pragma once
#include "../pch.h"
#include "Animation.h"
#include "AnimationDatabase.h"
#include "MotionAnalyzer.h"
#include "GradientBandInterpolator.h"
#include "SamplingJob.h"
#include "LocalToModelJob.h"

using namespace DirectX::SimpleMath;

namespace Animation
{
	class LegInfo 
	{
	public:
		int hip;
		int knee;
		int ankle;
		int toe;
		// Transform hip, ankle, toe
		//float footWidth;
		Vector3 ankleHeelVector;
		Vector3 toeToetipVector;
		float footLength;
		//Vector2 footOffset;

	};

	class MotionGroupInfo 
	{
	public:
		std::string name;

		//std::vector<MotionAnalyzer> motions;

		Interpolator interpolator;

		std::vector<float> GetMotionWeights(Vector3 velocity)
		{
			return interpolator.Interpolate(Vector2(velocity.x, velocity.z));
		}
	};

	class LegController
	{
	public:
		enum { kLeft, kRight, legSize };

		LegInfo legs[legSize];

		AnimationClip* groundedPose;

		//MotionAnalyzer* sourceAnimations;

		MotionGroupInfo* motionGroups;

		AnimationDatabase* skeleton;

		const char* kLeftJointNames[4] = { "LeftUpLeg", "LeftLeg", "LeftFoot", "LeftToe" };
		const char* kRightJointNames[4] = { "RightUpLeg", "RightLeg", "RightFoot", "RightToe" };
		// FIXME:
		//const char* kLeftJointNames[4] = { "Hips", "LeftArm", "LeftForeArm", "LeftHand" };
		//const char* kRightJointNames[4] = { "Hips", "RightArm", "RightForeArm", "RightHand" };



		// Find the index of hip, ankle and toe
		bool InitLegInfo(int leg, const char* jointNames[4]);

		bool Initialize();
		
	};
}

	