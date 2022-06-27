#pragma once
#include "..//..//pch.h"
#include "..//LocalToModelJob.h"
#include "IKChain.h"
#include "IKPoint.h"

namespace Animation
{
	class IKRig 
	{
	public:
		void Init(AnimationDatabase* db, const std::vector<Transform>* tpose, bool is_mixamo);

		void AddPoint(std::string point_name, std::string joint_name);

		void AddChain(std::string chain_name, std::vector<std::string> joint_names, std::string end_name, std::string solver);

		void UpdateWorld();

		void Init_Mixamo_Rig();

		const std::vector<Transform>* tpose;

		std::vector<Transform> pose;

		std::vector<Transform> pose_world;

		std::vector<Transform> tpose_world;

		std::map<std::string, IKChain> chains; // The joint index of bone chains, usually limbs / spine / hair / tail

		std::map<std::string, IKPoint> points; // Main single bones of the rig, like head / hip / chest

		AnimationDatabase* skeleton;

	};
}