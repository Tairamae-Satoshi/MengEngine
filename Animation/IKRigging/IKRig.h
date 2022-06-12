#pragma once
#include "..//..//pch.h"
#include "IKChain.h"

namespace Animation
{
	class IKRig 
	{
	public:

		void AddPoint(std::string point_name, std::string joint_name)
		{
			for (int i = 0; i < skeleton->JointCount(); i++)
			{
				std::string jointName = skeleton->GetJointName(i);

				if (jointName.find(joint_name) != jointName.npos)
				{
					this->points[point_name] = i;
					break;
				}
			}

		}

		void AddChain(std::string chain_name, std::vector<std::string> joint_names, std::string solver)
		{
			int found = 0;
			IKChain ch;

			for (int i = 0; i < skeleton->JointCount() && found != joint_names.size(); i++)
			{
				std::string jointName = skeleton->GetJointName(i);

				if (jointName.find(joint_names[found]) != jointName.npos)
				{
					ch.AddBone(i, 0); // TODO: Add length
					++found;
				}
			}

			this->chains[chain_name] = std::move(ch);
		}

	private:
		std::map<std::string, IKChain> chains; // The joint index of bone chains, usually limbs / spine / hair / tail

		std::map<std::string, unsigned int> points; // Main single bones of the rig, like head / hip / chest

		AnimationDatabase* skeleton;


	};
}