#pragma once

#include "AnimationDatabase.h"
#include <map>
#include <algorithm>

using namespace Animation;

class BinLoader {
public:
	void LoadCharacter(std::vector<Animation::SkinnedVertex>& vertices,
		std::vector<USHORT>& indices,
		AnimationDatabase& db, 
		const char* filename)
	{
		std::vector<float> positions; // 3
		std::vector<float> normals; // 3
		std::vector<float> texcoords; // 2
		std::vector<unsigned short> triangles;

		std::vector<std::vector<float>> bone_weights;
		std::vector<std::vector<unsigned short>> bone_indices;

		std::vector<float> bone_rest_positions; // 3
		std::vector<float> bone_rest_rotations; // 4

		for (int i = 0; i < positions.size() / 3; i++)
		{
			vertices[i].Pos = Vector3(positions[3 * i], positions[3 * i + 1], positions[3 * i + 2]);
			vertices[i].Normal = Vector3(normals[3 * i], normals[3 * i + 1], normals[3 * i + 2]);
			vertices[i].TexC = Vector2(texcoords[2 * i], texcoords[2 * i + 1]);
			vertices[i].BoneWeights;
		}

	}

	void LoadBin(AnimationDatabase& db, const char* filename)
	{

		// Read bin
		FILE* f = fopen(filename, "rb");
		assert(f != NULL);
		array2d_read(bone_positions, f, 3);
		array2d_read(bone_velocities, f, 3);
		array2d_read(bone_rotations, f, 4);
		array2d_read(bone_angular_velocities, f, 4);
		array1d_read(bone_parents, f, 1);

		array1d_read(range_starts, f);
		array1d_read(range_stops, f);

		array2d_read(contact_states, f, 1);

		fclose(f);

		// Fill db


		db.mJointHierarchy = bone_parents;
		db.rangeStarts = range_starts;
		db.rangeStops = range_stops;
	}

	template<typename T>
	void array2d_read(std::vector<std::vector<T>>& arr, FILE* f, int ele_num = 1)
	{
		int rows, cols;
		fread(&rows, sizeof(int), 1, f);
		fread(&cols, sizeof(int), 1, f);
		arr.resize(rows);
		for (auto& a : arr)
			a.resize(cols * ele_num);

		size_t num = fread(arr.data, sizeof(T), rows * cols * ele_num, f);
		assert((int)num == rows * cols);
	}

	template<typename T>
	void array1d_read(std::vector<T>& arr, FILE* f, int ele_num = 1)
	{
		int size;
		fread(&size, sizeof(int), 1, f);
		arr.resize(size);
		size_t num = fread(arr.data, sizeof(T), size * ele_num, f);
		assert((int)num == rows * cols);
	}

private:
	std::vector<std::vector<float>> bone_positions;
	std::vector<std::vector<float>> bone_velocities;
	std::vector<std::vector<float>> bone_rotations;
	std::vector<std::vector<float>> bone_angular_velocities;
	std::vector<int> bone_parents;

	std::vector<int> range_starts;
	std::vector<int> range_stops;

	std::vector<std::vector<bool>> contact_states;


	std::vector<DirectX::XMFLOAT4X4> jointOffsets;
	std::vector<int> jointIndexToParentIndex;
	std::vector<std::string> nodeIndexToName;// <NodeIndex, NodeName>
	std::unordered_map<std::string, AnimationClip> animations;

	UINT m_NumBones = 0;
	std::map<std::string, UINT> m_BoneMapping;
	std::map<std::string, bool> necessityMap;

};