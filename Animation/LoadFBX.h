#pragma once
#include <assimp/Importer.hpp>      // C++ importer interface
#include <assimp/scene.h>           // Output data structure
#include <assimp/postprocess.h>     // Post processing flags
#include "AnimationDatabase.h"
#include <map>
#include <algorithm>

using namespace Animation;

class FBXLoader
{
private:
#define NUM_BONES_PER_VEREX 4

public:
	struct Vertex
	{
		DirectX::XMFLOAT3 Pos;
		DirectX::XMFLOAT3 Normal;
		DirectX::XMFLOAT3 TangentU;
		DirectX::XMFLOAT2 TexC;
	};

	struct Subset
	{
		UINT Id = -1;
		UINT MaterialIndex = -1;
		UINT VertexStart = 0;
		UINT VertexCount = 0;
		UINT FaceStart = 0;
		UINT FaceCount = 0;
	};

	struct FbxMaterial
	{
		std::string Name;

		DirectX::XMFLOAT4 DiffuseAlbedo = { 1.0f, 1.0f, 1.0f, 1.0f };
		DirectX::XMFLOAT3 FresnelR0 = { 0.01f, 0.01f, 0.01f };
		float Roughness = 0.8f;
		bool AlphaClip = false;

		std::string MaterialTypeName;
		std::string DiffuseMapName;
		std::string NormalMapName;
	};

	bool LoadFBX(const std::string& filename,
		std::vector<Vertex>& vertices,
		std::vector<USHORT>& indices,
		std::vector<Subset>& subsets,
		std::vector<FbxMaterial>& mats);

	bool LoadBindPose(const std::string& filename,
		std::vector<Animation::SkinnedVertex>& vertices,
		std::vector<USHORT>& indices,
		std::vector<Subset>& subsets,
		std::vector<FbxMaterial>& mats,
		AnimationDatabase& skeleton);

	bool GetBindPose(const aiScene* pScene);

	bool LoadFBXClip(const std::string& filename, AnimationDatabase& skeleton);

private:
	std::vector<Matrix> jointOffsets;
	std::vector<int> jointIndexToParentIndex;
	std::vector<std::string> nodeIndexToName;// <NodeIndex, NodeName>
	std::unordered_map<std::string, AnimationClip> animations;
	std::vector<Transform> bind_pose;


	UINT m_NumBones = 0;
	std::map<std::string, UINT> m_BoneMapping;
	std::map<std::string, bool> necessityMap;

	// Load No skinned model
	bool InitFromScene(const aiScene* pScene,
		const std::string& Filename,
		std::vector<Subset>& subsets,
		std::vector<Vertex>& vertices,
		std::vector<USHORT>& indices,
		std::vector<FbxMaterial>& mats);
	void InitMesh(UINT MeshIndex,
		const aiMesh* paiMesh,
		std::vector<Subset>& subsets,
		std::vector<Vertex>& vertices,
		std::vector<USHORT>& indices);
	bool InitFromScene(const aiScene* pScene, 
		const std::string& Filename, 
		std::vector<Subset>& subsets,
		std::vector<Animation::SkinnedVertex>& vertices,
		std::vector<USHORT>& indices,
		std::vector<FbxMaterial>& mats);
	void InitMesh(UINT MeshIndex, 
		const aiMesh* paiMesh,
		std::vector<Subset>& subsets,
		std::vector<Animation::SkinnedVertex>& vertices,
		std::vector<USHORT>& indices);
	void LoadBones(UINT MeshIndex, 
		const aiMesh* pMesh,
		std::vector<Subset>& subsets,
		std::vector<Animation::SkinnedVertex>& vertices,
		std::vector<USHORT>& indices);
	void InitMaterials(std::vector<FbxMaterial>& mats, const aiScene* pScene);
	std::vector<Matrix> ReorganizeBoneOffsets(const aiScene* pScene);

	// Create Skeleton Hierachy
	void CreateSkeletonHierachy(const aiScene* scene);
	void BuildNodeMappingWithBone(aiNode* node, const aiScene* scene);
	void SetNodeMappingWithBone(aiNode* node, const aiScene* scene);
	void CreateSkeletonHierachyWithNecessityMap(aiNode* node, int index);

	void ReadAnimationClips(const std::string& path, const aiScene* pScene);
	void ReadBoneKeyframes(aiNodeAnim* nodeAnim, BoneAnimationSample& boneAnimation);
};

