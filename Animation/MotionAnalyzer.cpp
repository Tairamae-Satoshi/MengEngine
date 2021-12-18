#include "MotionAnalyzer.h"
#include "LegController.h"
#include "Animation.h"
#include "SamplingJob.h"
#include "LocalToModelJob.h"

namespace Animation
{
	void MotionAnalyzer::Analyze()
	{
		char out[500];

		sampleNum = 50;

		for (size_t leg = 0; leg < 2; leg++)
		{
			int hip = legC->legs[leg].hip;
			int knee = legC->legs[leg].knee;
			int ankle = legC->legs[leg].ankle;
			int toe = legC->legs[leg].toe;

			float rangeMax = 0;
			cycles[leg].ankleMin = std::numeric_limits<float>::max();
			cycles[leg].ankleMax = std::numeric_limits<float>::lowest();
			cycles[leg].toeMin = std::numeric_limits<float>::max();
			cycles[leg].toeMax = std::numeric_limits<float>::lowest();
			// Foot stance time
			// Find the time when the foot stands most firmly on the ground.
			for (int i = 0; i < sampleNum + 1; ++i)
			{
				SamplingJob samplingJob;
				samplingJob.animation = animation;
				samplingJob.ratio = (float)i / (float)sampleNum;
				samplingJob.Run();
				
				LocalToModelJob ltmJob;
				ltmJob.skeleton = legC->skeleton;
				ltmJob.input = samplingJob.output;
				ltmJob.Run(true, false);

				LegCycleSample sample;
				sample.knee = Vector3::Transform(Vector3::Zero, ltmJob.output[knee]);
				sample.heel = Vector3::Transform(legC->legs[leg].ankleHeelVector, ltmJob.output[ankle]);
				sample.toetip = Vector3::Transform(legC->legs[leg].toeToetipVector, ltmJob.output[toe]);
				sample.middle = (sample.heel + sample.toetip) / 2;
				if (i == 0)	legC->legs[leg].footLength = (sample.toetip - sample.heel).Length();
				// For each sample in time we want to know if the heel or toetip is closer to the ground.
				// We need a smooth curve with 0 = ankle is closer and 1 = toe is closer.
				sample.balance = GetFootBalance(sample.heel.y, sample.toetip.y, legC->legs[leg].footLength);
				//sprintf(out, "index: %d, balance: %f, ankle: %f, toe: %f", i, sample.balance, sample.ankle.y, sample.toe.y);
				//Debug::Log(LOG_LEVEL::LOG_LEVEL_INFO, "FindSwingChangeTime", "MotionAnalyzer", 256, out);

				cycles[leg].samples.push_back(sample);

				// Find the minimum and maximum extends on all axes of the ankle and toe positions.
				cycles[leg].ankleMin = std::min(cycles[leg].ankleMin, sample.heel.y);
				cycles[leg].ankleMax = std::max(cycles[leg].ankleMax, sample.heel.y);
				cycles[leg].toeMin = std::min(cycles[leg].toeMin, sample.toetip.y);
				cycles[leg].toeMax = std::max(cycles[leg].toeMax, sample.toetip.y);
			}
			rangeMax = std::max(cycles[leg].ankleMax - cycles[leg].ankleMin, cycles[leg].toeMax - cycles[leg].toeMin);

			FindCycleAxis(leg);

			// Find stance time
			float cost = std::numeric_limits<float>::max();
			int stanceIndex = 0;
			int index = 0;
			for (auto sample: cycles[leg].samples)
			{
				float a = std::max(sample.heel.y - cycles[leg].ankleMin, sample.toetip.y - cycles[leg].toeMin) / rangeMax;
				float b = std::fabs((sample.middle - cycles[leg].cycleCenter).Dot(cycles[leg].cycleDirection)) / cycles[leg].cycleScaling;
				float curCost = a + b;
				if (curCost < cost)
				{
					stanceIndex = index;
					cost = curCost;
				}
				index++;
			}
			cycles[leg].stanceTime = GetTimeFromIndex(stanceIndex);
			cycles[leg].stanceIndex = stanceIndex;

			// The vector from heel to toetip at the stance pose 
			cycles[leg].heelToetipVector = cycles[leg].samples[stanceIndex].toetip - cycles[leg].samples[stanceIndex].heel;
			cycles[leg].heelToetipVector.y = 0;
			cycles[leg].heelToetipVector.Normalize();
			cycles[leg].heelToetipVector *= legC->legs[leg].footLength;

			// Calculate foot flight path based on weighted average between ankle flight path and toe flight path,
			// using foot balance as weight.
			// The distance between ankle and toe is accounted for, using the stance pose for reference.
			for (int i = 0; i < sampleNum + 1; ++i)
			{
				LegCycleSample& s = cycles[leg].samples[i];
				// NOTE: the second row is what the paper write, which is wrong.
				s.footBase = s.heel * (1.0f - s.balance) + (s.toetip - cycles[leg].heelToetipVector) * s.balance;
				//s.footBase = s.ankle * s.balance + (s.toe - cycles[leg].heelToetipVector) * (1.0f - s.balance);
			}

			// Find contact times:
			// Strike time: foot first touches the ground (0% grounding)
			// Down time: all of the foot touches the ground (100% grounding)
			// Lift time: all of the foot still touches the ground but begins to lift (100% grounding)
			// Liftoff time: last part of the foot leaves the ground (0% grounding)
			float timeA;
			float timeB;
			timeA = FindContactTime(leg, false, +1, 0.1f);
			cycles[leg].debugInfo.ankleLiftTime = timeA;
			timeB = FindContactTime(leg, true, +1, 0.1f);
			cycles[leg].debugInfo.toeLiftTime = timeB;
			cycles[leg].liftTime = std::min(timeA, timeB);
			cycles[leg].liftoffTime = std::max(timeA, timeB);

			// Find time where swing direction and speed changes significantly.
			// If this happens sooner than the found liftoff time,
			// then the liftoff time must be overwritten with this value.
			timeA = FindSwingChangeTime(leg, +1, 0.5f);
			cycles[leg].debugInfo.footLiftTime = timeA;
			if (cycles[leg].liftoffTime > timeA)
			{
				cycles[leg].liftoffTime = timeA;
				if (cycles[leg].liftTime > cycles[leg].liftoffTime)
				{
					cycles[leg].liftTime = cycles[leg].liftoffTime;
				}
			}

			// Find downwards contact times for projected ankle and toe
			// Use the first occurance as strike time and the second as down time
			timeA = FindContactTime(leg, false, -1, 0.5f);
			timeB = FindContactTime(leg, true, -1, 0.5f);

			cycles[leg].strikeTime = std::min(timeA, timeB);
			cycles[leg].landTime = std::max(timeA, timeB);
			// Find time where swing direction and speed changes significantly.
			// If this happens later than the found strike time,
			// then the strike time must be overwritten with this value.
			timeA = FindSwingChangeTime(leg, -1, 0.5f);
			cycles[leg].debugInfo.footLandTime = timeA;
			if (cycles[leg].strikeTime < timeA) {
				cycles[leg].strikeTime = timeA;
				if (cycles[leg].landTime < cycles[leg].strikeTime) {
					cycles[leg].landTime = cycles[leg].strikeTime;
				}
			}


			// Set postliftTime and prelandTime
			float softening = 0.2f;
			cycles[leg].postliftTime = std::max(cycles[leg].liftoffTime, cycles[leg].liftTime + softening);
			cycles[leg].prelandTime = std::min(cycles[leg].strikeTime, cycles[leg].landTime - softening);


			// Calculate the distance traveled during one cycle(for this root).
			Vector3 stanceSlideVector(
				cycles[leg].samples[GetIndexFromTime(cycles[leg].liftoffTime)].footBase
				- cycles[leg].samples[GetIndexFromTime(cycles[leg].strikeTime)].footBase
			);
			// Assum horizontal ground plane
			stanceSlideVector.y = 0;
			cycles[leg].cycleDistance = stanceSlideVector.Length() / (cycles[leg].liftoffTime - cycles[leg].strikeTime + 1);
			cycles[leg].cycleDirection = -(stanceSlideVector.Normalized());

			//sprintf(out, "prelandTime: %f", cycles[leg].prelandTime);
			//Debug::Log(LOG_LEVEL::LOG_LEVEL_INFO, "Analyze", "MotionAnalyzer", 188, out);
			//sprintf(out, "strikeTime: %f", cycles[leg].strikeTime);
			//Debug::Log(LOG_LEVEL::LOG_LEVEL_INFO, "Analyze", "MotionAnalyzer", 190, out);
			//sprintf(out, "landTime: %f", cycles[leg].landTime);
			//Debug::Log(LOG_LEVEL::LOG_LEVEL_INFO, "Analyze", "MotionAnalyzer", 192, out);
			//sprintf(out, "stanceTime: %f", cycles[leg].stanceTime);
			//Debug::Log(LOG_LEVEL::LOG_LEVEL_INFO, "Analyze", "MotionAnalyzer", 194, out);
			//sprintf(out, "liftTime: %f", cycles[leg].liftTime);
			//Debug::Log(LOG_LEVEL::LOG_LEVEL_INFO, "Analyze", "MotionAnalyzer", 196, out);
			//sprintf(out, "liftoffTime: %f", cycles[leg].liftoffTime);
			//Debug::Log(LOG_LEVEL::LOG_LEVEL_INFO, "Analyze", "MotionAnalyzer", 198, out);
			//sprintf(out, "postliftTime£º %f", cycles[leg].postliftTime);
			//Debug::Log(LOG_LEVEL::LOG_LEVEL_INFO, "Analyze", "MotionAnalyzer", 200, out);
		}

		// Find the overall speed and direction traveled during one cycle
		// based on average of speed values for each individual foot
		// (They should be very close, but animations are often imperfect,
		// leading to different speeds for different legs.)
		mCycleDistance = 0.0f;
		mCycleDirection = Vector3::Zero;
		for (int leg = 0; leg < 2; leg++)
		{
			mCycleDistance += cycles[leg].cycleDistance;
			mCycleDirection += cycles[leg].cycleDirection;
		}
		int legs = 2;
		mCycleDistance /= legs;
		mCycleDirection /= legs;

		// Eliminate error
		if (std::abs(mCycleDirection.x) >= 0.99) mCycleDirection = Vector3(mCycleDirection.x / std::abs(mCycleDirection.x), 0.0f, 0.0f);
		if (std::abs(mCycleDirection.z) >= 0.99) mCycleDirection = Vector3(0.0f, 0.0f, mCycleDirection.z / std::abs(mCycleDirection.z));

		mCycleDuration = animation->GetDuration();
		mCycleSpeed = mCycleDistance / mCycleDuration;
		sprintf(out, "Cycle direction: (%f, %f, %f) with step distance %f and speed %f", mCycleDirection.x, mCycleDirection.y, mCycleDirection.z, mCycleDistance, mCycleSpeed);
		Debug::Log(LOG_LEVEL::LOG_LEVEL_INFO, "Analyze", "MotionAnalyzer", 196, out);

	}

	float ComputeCurvature(Vector2 p0, Vector2 p1, Vector2 p2)
	{
		float dx1 = p1.x - p0.x;
		float dy1 = p1.y - p0.y;
		float dx2 = p2.x - p0.x;
		float dy2 = p2.y - p0.y;
		float area = dx1 * dy2 - dy1 * dx2;
		float len0 = Vector2::Distance(p0, p1);
		float len1 = Vector2::Distance(p1, p2);
		float len2 = Vector2::Distance(p2, p0);
		return 4 * area / (len0 * len1 * len2);
	}

	float MotionAnalyzer::FindContactTime(int leg, bool useToe, int searchDirection, float threshold)
	{
		// Find the contact time on the height curve, where the (projected ankle or toe)
		// hits or leaves the ground (depending on search direction in time).
		int spread = 5;
		float curvatureMax = 0;
		int curvatureMaxIndex = cycles[leg].stanceIndex;
		for (int i = 0; i<sampleNum && i>-sampleNum; i += searchDirection)
		{
			// Find second derived by sampling three positions on curve.
			// Spred samples a bit to ignore high frequency noise.
			// FIXME magic number for spread value
			int j[3];
			float value[3];
			for (int s = 0; s < 3; s++)
			{
				j[s] = (i + cycles[leg].stanceIndex - spread + spread * s + sampleNum) % sampleNum;
				value[s] = useToe ? cycles[leg].samples[j[s]].toetip.y : cycles[leg].samples[j[s]].heel.y;
			}
			//float curvatureCurrent = std::atan((value[2] - value[1]) * 10 / yRange) - std::atan((value[1] - value[0]) * 10 / yRange);
			float curvatureCurrent = ComputeCurvature(Vector2(j[0], value[0]), Vector2(j[1], value[1]), Vector2(j[2], value[2]));

			if (curvatureCurrent > curvatureMax &&
				(value[2] - value[0] > 0) == (searchDirection > 0))
			{
				curvatureMax = curvatureCurrent;
				curvatureMaxIndex = j[1];
			}
			// Terminate search when foot height is above threshold height above ground
			float lowest = useToe ? cycles[leg].toeMin : cycles[leg].ankleMin;
			float yRange = (useToe ? cycles[leg].toeMax : cycles[leg].ankleMax) - lowest;
			if (value[1] - lowest >yRange * threshold)
				break;
		}

		return GetTimeFromIndex((curvatureMaxIndex + sampleNum) % sampleNum);
	}

	float MotionAnalyzer::FindSwingChangeTime(int leg, int searchDirection, float threshold)
	{
		// Find the contact time on the height curve, where the (projected ankle or toe)
		// hits or leaves the ground (depending on search direction in time).
		int spread =  sampleNum / 5;
		float stanceSpeed = 0;

		for (int i = 0; i<sampleNum && i>-sampleNum; i += searchDirection)
		{
			// Find speed by sampling curve value ahead and behind.
			int j[3];
			float value[3];
			for (int s = 0; s < 3; ++s)
			{
				j[s] = (i + cycles[leg].stanceIndex - spread + spread * s + sampleNum) % sampleNum;
				value[s] = cycles[leg].samples[j[s]].footBase.Dot(cycles[leg].cycleDirection);
			}
			float currentSpeed = value[2] - value[0];
			if (i == 0)	stanceSpeed = currentSpeed;

			// If speed is too different from speed at stance time,
			// the current time is determined as the swing change time
			if (std::fabs((currentSpeed - stanceSpeed) / std::fabs(stanceSpeed)) > 0.5){
				return GetTimeFromIndex((j[1] + sampleNum) % sampleNum);
			}

		}
		return searchDirection * -0.01f;
	}

	void MotionAnalyzer::FindCycleAxis(int leg)
	{
		// Find axis that feet are moving back and forth along
		// (i.e. Z for characters facing Z, that are walking forward, but could be any direction)
		// FIXME
		// First find the average point of all the points in the foot motion curve
		// (projeted onto the ground plane). This gives us a center.
		// Find c which is the average of all middle positions
		cycles[leg].cycleCenter = Vector3::Zero;
		for (auto sample : cycles[leg].samples)
		{
			cycles[leg].cycleCenter += sample.middle;
		}
		cycles[leg].cycleCenter /= sampleNum;

		// Find a which is the point furthest away from c
		Vector3 a = cycles[leg].cycleCenter;
		float dis2 = 0.0f;
		for (auto sample : cycles[leg].samples)
		{
			float curDis2 = Vector3::DistanceSquared(cycles[leg].cycleCenter, sample.middle);
			if (curDis2 >= dis2)
			{
				a = sample.middle;
				dis2 = curDis2;
			}
		}

		// Find b which is the point furthest away from a
		Vector3 b = a;
		dis2 = 0.0f;
		for (auto sample : cycles[leg].samples)
		{
			float curDis2 = Vector3::DistanceSquared(a, sample.middle);
			if (curDis2 >= dis2)
			{
				b = sample.middle;
				dis2 = curDis2;
			}
		}

		cycles[leg].cycleDirection = b - a;
		cycles[leg].cycleDirection.Normalize();
		cycles[leg].cycleScaling = (b - a).Length();
	}

	float MotionAnalyzer::GetFootBalance(float heelElevation, float toeElevation, float footLength)
	{
		// For any moment in time we want to know if the heel or toe is closer to the ground.
		// Rather than a binary value, we need a smooth curve with 0 = heel is closer and 1 = toe is closer.
		// We use the inverse tangens for this as it maps arbritarily large positive or negative values into a -1 to 1 range.
		return std::atan((heelElevation - toeElevation) / footLength * 20) / MathHelper::Pi + 0.5f;
		// The 20 multiplier is found by trial and error. A rapid but still slightly smooth change of weight is wanted.
	}

	float MotionAnalyzer::GetTimeFromIndex(int index) {
		return (float)index / (float)sampleNum;
	}

	int MotionAnalyzer::GetIndexFromTime(float time) {
		time = time - floorf(time);
		return static_cast<int>(time * sampleNum);
	}
}