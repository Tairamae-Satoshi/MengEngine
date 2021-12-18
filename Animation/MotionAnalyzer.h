#pragma once
#include "../pch.h"

using namespace DirectX::SimpleMath;

namespace Animation
{
	class LegController;
	class AnimationClip;

	struct LegCycleSample
	{
		Vector3 knee;
		Vector3 heel;
		Vector3 toetip;
		Vector3 middle;
		float balance;
		Vector3 footBase;
		Vector3 footBaseNormalized;
	};

	struct CycleDebugInfo
	{
		float toeLiftTime;
		float toeLandTime;
		float ankleLiftTime;
		float ankleLandTime;
		float footLiftTime;
		float footLandTime;
	};

	struct LegCycleData
	{
		Vector3 cycleCenter;
		float cycleScaling;
		Vector3 cycleDirection;
		float cycleDistance;
		float stanceTime;
		float liftTime;
		float liftoffTime;
		float postliftTime;
		float prelandTime;
		float strikeTime;
		float landTime;
		float ankleMin;
		float ankleMax;
		float toeMin;
		float toeMax;
		Vector3 heelToetipVector; // foot direction
		std::vector<LegCycleSample> samples;
		int stanceIndex;
		CycleDebugInfo debugInfo;
	};

	class MotionAnalyzer
	{
	public:
		enum { kLeft, kRight, legSize };

		LegController* legC;

		const AnimationClip* animation;

		LegCycleData cycles[2];

		int sampleNum = 0;

		void Analyze();

		float mCycleDistance;

		Vector3 mCycleDirection;

		float mCycleDuration;

		float mCycleSpeed;

	private:

		float FindContactTime(int leg, bool useToe, int searchDirection, float threshold);

		float FindSwingChangeTime(int leg, int searchDirection, float threshold);

		void FindCycleAxis(int leg);

		float GetFootBalance(float heelElevation, float toeElevation, float footLength);

		float GetTimeFromIndex(int index);

		int GetIndexFromTime(float time);
	};
}