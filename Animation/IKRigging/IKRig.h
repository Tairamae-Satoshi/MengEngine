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
		void Init(AnimationDatabase* db, const std::vector<Transform>* tpose, bool is_mixamo)
		{
			this->tpose = tpose;
			this->pose = *tpose; //TODO
			this->skeleton = db;

			if (this->tpose != nullptr) {
				LocalToModelJob ltmJob;
				ltmJob.skeleton = this->skeleton;
				ltmJob.input = *this->tpose;
				ltmJob.Run(true, false);

				tpose_world.clear();
				for (auto m : ltmJob.output){
					Transform t;
					m.Decompose(t.mScale.mValue, t.mRot.mValue, t.mTrans.mValue);
					tpose_world.push_back(t);
				}
			}

			UpdateWorld();

			if(is_mixamo) Init_Mixamo_Rig();
		}


		void AddPoint(std::string point_name, std::string joint_name)
		{
			for (unsigned int i = 0; i < skeleton->JointCount(); i++)
			{
				std::string jointName = skeleton->GetJointName(i);

				if (jointName.find(joint_name) != jointName.npos)
				{
					this->points[point_name] = IKPoint(i);
					break;
				}
			}

		}

		void AddChain(std::string chain_name, std::vector<std::string> joint_names, std::string end_name, std::string solver)
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

			if (end_name != ""){
				for (int i = 0; i < skeleton->JointCount(); i++){
					std::string jointName = skeleton->GetJointName(i);
					if (jointName.find(end_name) != jointName.npos){
						ch.end_idx = i;
						break;
					}
				}
			}

			ch.ComputeLen(tpose_world);
			this->chains[chain_name] = std::move(ch);
		}

		void UpdateWorld()
		{
			LocalToModelJob ltmJob;
			ltmJob.skeleton = this->skeleton;
			ltmJob.input = this->pose;
			ltmJob.Run(true, false);

			pose_world.clear();
			for (auto m : ltmJob.output) {
				pose_world.push_back(Transform::FromMatrix(m));
			}

			/*int jointNum = skeleton->JointCount();
			for (UINT i = 1; i < jointNum; ++i)
			{
				Quaternion rotation = this->pose[i].mRot.mValue;

				int parentIndex = skeleton->GetJointParentIndex(i);

				Quaternion q = rotation * pose_world[parentIndex].mRot.mValue;
				Quaternion q1 = pose_world[i].mRot.mValue;
				pose_world[i].mRot.mValue = rotation * pose_world[parentIndex].mRot.mValue;
			}*/

		}

		void Init_Mixamo_Rig()
		{
			this->AddPoint("hip", "Hips");
			this->AddPoint("head", "Head");
			this->AddPoint("neck", "Neck");
			this->AddPoint("chest", "Spine2");
			this->AddPoint("foot_l", "LeftFoot");
			this->AddPoint("foot_r", "RightFoot");

			this->AddPoint("hand_l", "LeftHand");
			this->AddPoint("hand_r", "RightHand");

			this->AddChain("arm_r", { "RightArm", "RightForeArm" }, "RightHand", "TwoBone");
			this->AddChain("arm_l", { "LeftArm", "LeftForeArm" }, "LeftHand", "TwoBone");

			this->AddChain("leg_r", { "RightUpLeg", "RightLeg" }, "RightFoot", "TwoBone");
			this->AddChain("leg_l", { "LeftUpLeg", "LeftLeg" }, "LeftFoot", "TwoBone");

			this->AddChain("spine", { "Spine", "Spine1", "Spine2" }, "", "");

			this->points["head"].SetAlt(Vector3::Forward, Vector3::Up, tpose_world); // Look = Fwd, Twist = Up

			this->points["foot_l"].SetAlt(Vector3::Forward, Vector3::Up, tpose_world); // Look = Fwd, Twist = Up
			this->points["foot_r"].SetAlt(Vector3::Forward, Vector3::Up, tpose_world); // Look = Fwd, Twist = Up

			this->points["hand_l"].SetAlt(Vector3::Down, Vector3::Right, tpose_world); // Look = Down, Twist = Right
			this->points["hand_r"].SetAlt(Vector3::Down, Vector3::Left, tpose_world); // Look = Down, Twist = Left

			this->chains["leg_l"].SetAlt(Vector3::Down, Vector3::Forward, tpose_world); // Look = Forward, Twist = Down
			this->chains["leg_r"].SetAlt(Vector3::Down, Vector3::Forward, tpose_world);

			this->chains["arm_l"].SetAlt(Vector3::Right, Vector3::Backward, tpose_world); 
			this->chains["arm_r"].SetAlt(Vector3::Left, Vector3::Backward, tpose_world);

			this->chains["spine"].SetAlt(Vector3::Up, Vector3::Forward, tpose_world); // Look = Up, Twist = Forward
		}

		const std::vector<Transform>* tpose;

		std::vector<Transform> pose;

		std::vector<Transform> pose_world;

		std::vector<Transform> tpose_world;

		std::map<std::string, IKChain> chains; // The joint index of bone chains, usually limbs / spine / hair / tail

		std::map<std::string, IKPoint> points; // Main single bones of the rig, like head / hip / chest



		AnimationDatabase* skeleton;

	};
}