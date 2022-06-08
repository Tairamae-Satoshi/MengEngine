#include "LegController.h"

namespace Animation
{
	// Find the index of hip, ankle and toe
	bool LegController::InitLegInfo(int leg, const char* jointNames[4])
	{
		int found = 0;
		int joints[4] = { -1, -1, -1, -1 };

		for (int i = 0; i < skeleton->JointCount() && found != 4; i++)
		{
			std::string jointName = skeleton->GetJointName(i);

			if (jointName.find(jointNames[found]) != jointName.npos)
			{
				joints[found] = i;
				++found;
			}
		}
		legs[leg].hip = joints[0];
		legs[leg].knee = joints[1];
		legs[leg].ankle = joints[2];
		legs[leg].toe = joints[3];
		//char out[100];
		//sprintf(out, "-----------%d", legs[leg].ankle);
		//Debug::Log(LOG_LEVEL::LOG_LEVEL_INFO, "Analyze", "MotionAnalyzer", 196, out);

		return found == 4;
	}

	bool LegController::Initialize()
	{
		if (!InitLegInfo(kLeft, kLeftJointNames) || !InitLegInfo(kRight, kRightJointNames))
		{
			return false;
		}

		SamplingJob samplingJob;
		samplingJob.animation = skeleton->GetBindPose();
		samplingJob.ratio = 0.0f;
		if (!samplingJob.Run())
		{
			// return false;
		}

		LocalToModelJob ltmJob;
		ltmJob.skeleton = skeleton;
		ltmJob.input = samplingJob.output;
		if (!ltmJob.Run(true, true))
		{
			// return false;
		}

		LocalToModelJob ltmJobWithoutOffset;
		ltmJobWithoutOffset.skeleton = skeleton;
		ltmJobWithoutOffset.input = samplingJob.output;
		if (!ltmJobWithoutOffset.Run(true, false))
		{
			// return false;
		}


		// Find toe and heel
		for (size_t leg = 0; leg < legSize; leg++)
		{
			int count = 0;
			float xMin = std::numeric_limits<float>::max();
			float xMax = std::numeric_limits<float>::min();

			float zMin = std::numeric_limits<float>::max();
			float xAvg = 0.0f;
			float yAvg = 0.0f;
			float zAvg = 0.0f;

			Vector3 modelPos;
			Matrix M = ltmJobWithoutOffset.output[legs[leg].toe];
			legs[leg].toeToetipVector = Vector3(0.0f, 0.0f, -7.0f);//TODO
			M = ltmJobWithoutOffset.output[legs[leg].ankle];
			legs[leg].ankleHeelVector = Vector3(0.0f, -8.5f, 2.0f);

			//for (auto v : skeleton->GetVertices())
			//{
			//	float weight[4] = { v.BoneWeights.x, v.BoneWeights.y, v.BoneWeights.z, 1.0f - v.BoneWeights.x - v.BoneWeights.y - v.BoneWeights.z };
			//	for (size_t i = 0; i < NUM_BONES_PER_VEREX; i++)
			//	{
			//		if (v.BoneIndices[i] == legs[leg].toe)
			//		{
			//			if (weight[i] == 1.0f)
			//			{
			//				Matrix M = ltmJob.output[v.BoneIndices[i]];
			//				//Vector3 modelPos = Vector3(modelPos4.x, modelPos4.y, modelPos4.z);
			//				Vector3 modelPos = Vector3::Transform(v.Pos, M, false);

			//				//if (index++==0)
			//				//{
			//				//	legs[leg].toeVector = modelPos;
			//				//	sprintf(out, "index: %f, %f, %f", modelPos.x, modelPos.y, modelPos.z);
			//				//	Debug::Log(LOG_LEVEL::LOG_LEVEL_INFO, "Init", "LegController", 100, out);
			//				//} 
			//				xAvg += modelPos.x;
			//				yAvg += modelPos.y;
			//				zAvg += modelPos.z;
			//				xMin = std::min(xMin, modelPos.x);
			//				xMax = std::max(xMax, modelPos.x);
			//				sprintf(out, "xmin: %f, xmax: %f", xMin, xMax);
			//				Debug::Log(LOG_LEVEL::LOG_LEVEL_INFO, "Init", "LegController", 100, out);

			//				zMin = std::min(zMin, modelPos.z);
			//				count++;
			//			}
			//			break;
			//		}
			//	}
			//}
		}

	}
}