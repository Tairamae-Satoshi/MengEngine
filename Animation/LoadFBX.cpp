#include "..//pch.h"
#include "LoadFBX.h"

using namespace DirectX;

bool FBXLoader::LoadFBX(const std::string& filename,
	std::vector<Vertex>& vertices,
	std::vector<USHORT>& indices,
	std::vector<Subset>& subsets,
	std::vector<FbxMaterial>& mats)
{
	Assimp::Importer importer;
	importer.SetPropertyBool(AI_CONFIG_IMPORT_FBX_PRESERVE_PIVOTS, false);

	// FIXUP: why not clock wise?
	const aiScene* pScene = importer.ReadFile(filename,
		aiProcessPreset_TargetRealtime_Fast |
		aiProcess_ConvertToLeftHanded
		/*aiProcess_MakeLeftHanded |
		aiProcess_FlipUVs*/);

	// If the import failed, report it
	if (!pScene) {
		::OutputDebugStringA(importer.GetErrorString());
		return false;
	}

	InitFromScene(pScene, filename, subsets, vertices, indices, mats);
	jointOffsets = ReorganizeBoneOffsets(pScene);

	return true;
}

bool FBXLoader::InitFromScene(const aiScene* pScene,
	const std::string& Filename,
	std::vector<Subset>& subsets,
	std::vector<Vertex>& vertices,
	std::vector<USHORT>& indices,
	std::vector<FbxMaterial>& mats)
{
	subsets.resize(pScene->mNumMeshes);

	UINT NumVertices = 0;
	UINT NumFaces = 0;
	// Count the number of vertices and indices
	for (UINT i = 0; i < subsets.size(); i++)
	{
		subsets[i].MaterialIndex = pScene->mMeshes[i]->mMaterialIndex;
		//subsets[i].MaterialIndex = 0;
		subsets[i].FaceStart = NumFaces;
		subsets[i].FaceCount = pScene->mMeshes[i]->mNumFaces;
		subsets[i].VertexStart = NumVertices;
		subsets[i].VertexCount = pScene->mMeshes[i]->mNumVertices;

		NumFaces += subsets[i].FaceCount;
		NumVertices += subsets[i].VertexCount;
	}

	// Reserve space in the vectors for the vertex attributes and indices
	vertices.resize(NumVertices);
	indices.reserve(NumFaces * 3);

	// Initialize the meshes in the scene one by one
	for (UINT i = 0; i < subsets.size(); i++)
	{
		const aiMesh* paiMesh = pScene->mMeshes[i];
		InitMesh(i, paiMesh, subsets, vertices, indices);
	}

	InitMaterials(mats, pScene);
	return true;
}

void FBXLoader::InitMesh(UINT MeshIndex,
	const aiMesh* paiMesh,
	std::vector<Subset>& subsets,
	std::vector<Vertex>& vertices,
	std::vector<USHORT>& indices)
{
	const aiVector3D Zero3D(0.0f, 0.0f, 0.0f);

	// Populate the vertex attribute vectors
	UINT vertexIndex = subsets[MeshIndex].VertexStart;
	for (UINT i = 0; i < paiMesh->mNumVertices; i++)
	{
		Animation::SkinnedVertex vertex;
		const aiVector3D& Pos = paiMesh->mVertices[i];
		const aiVector3D& Normal = paiMesh->mNormals[i];
		const aiVector3D& Tangent = paiMesh->mTangents[i];
		const aiVector3D& TexCoord = paiMesh->HasTextureCoords(0) ? paiMesh->mTextureCoords[0][i] : Zero3D;

		vertices[vertexIndex + i].Pos = XMFLOAT3(Pos.x, Pos.y, Pos.z);
		vertices[vertexIndex + i].Normal = XMFLOAT3(Normal.x, Normal.y, Normal.z);
		vertices[vertexIndex + i].TangentU = XMFLOAT3(Tangent.x, Tangent.y, Tangent.z);
		vertices[vertexIndex + i].TexC = XMFLOAT2(TexCoord.x, TexCoord.y);
	}

	// Populate the index buffer
	for (UINT i = 0; i < paiMesh->mNumFaces; i++)
	{
		const aiFace& Face = paiMesh->mFaces[i];
		/*if (Face.mIndices[0] <= subsets[MeshIndex].VertexCount && Face.mIndices[1] <= subsets[MeshIndex].VertexCount && Face.mIndices[2] <= subsets[MeshIndex].VertexCount)
		{*/
		indices.push_back(Face.mIndices[0] + vertexIndex);
		indices.push_back(Face.mIndices[1] + vertexIndex);
		indices.push_back(Face.mIndices[2] + vertexIndex);
		//}
	}
}

bool FBXLoader::LoadBindPose(const std::string& filename,
	std::vector<Animation::SkinnedVertex>& vertices,
	std::vector<USHORT>& indices,
	std::vector<Subset>& subsets,
	std::vector<FbxMaterial>& mats,
	AnimationDatabase& db)
{
	Assimp::Importer importer;
	importer.SetPropertyBool(AI_CONFIG_IMPORT_FBX_PRESERVE_PIVOTS, false);

	// FIXUP: why not clock wise?
	const aiScene* pScene = importer.ReadFile(filename,
		aiProcessPreset_TargetRealtime_Fast |
		aiProcess_ConvertToLeftHanded
		/*aiProcess_MakeLeftHanded | 
		aiProcess_FlipUVs*/);

	// If the import failed, report it
	if (!pScene) {
		::OutputDebugStringA(importer.GetErrorString());
		return false;
	}

	animations.clear();
	CreateSkeletonHierachy(pScene);
	InitFromScene(pScene, filename, subsets, vertices, indices, mats);
	jointOffsets = ReorganizeBoneOffsets(pScene);
	GetBindPose(pScene);
	//ReadAnimationClips(filename, pScene);
	db.Set(jointIndexToParentIndex, nodeIndexToName,jointOffsets, vertices, bind_pose);
	
	return true;
}

bool FBXLoader::GetBindPose(const aiScene* pScene)
{
	for (size_t i = 0; i < nodeIndexToName.size(); i++)
	{
		std::string name = nodeIndexToName[i];
		Matrix m(&(pScene->mRootNode->FindNode(aiString(name))->mTransformation.a1));

		Transform transform;

		m.Transpose().Decompose(transform.mScale.mValue, transform.mRot.mValue, transform.mTrans.mValue);
	
		/*char out[50];
		if (i == 50)
		{
			Quaternion& q = transform.mRot.mValue;
			sprintf(out, "index: %f, %f, %f, %f", q.x, q.y, q.z, q.w);
			Debug::Log(LOG_LEVEL::LOG_LEVEL_INFO, "Load", "MotionAnalyzer", 256, out);
		}*/

		bind_pose.push_back(transform);
	}

	return true;
}

std::string GetFileName(const std::string& path)
{
	std::string::size_type iPos = path.find_last_of("/") + 1;
	std::string filename = path.substr(iPos);
	filename = filename.substr(0, filename.rfind("."));
	return filename;
}

bool FBXLoader::LoadFBXClip(const std::string& filename, AnimationDatabase& db)
{
	Assimp::Importer importer;

	const aiScene* pScene = importer.ReadFile(filename,
		aiProcessPreset_TargetRealtime_Fast |
		aiProcess_ConvertToLeftHanded
		/*aiProcess_MakeLeftHanded | 
		aiProcess_FlipUVs*/);

	// If the import failed, report it
	if (!pScene) {
		::OutputDebugStringA(importer.GetErrorString());
		return false;
	}

	animations.clear();
	ReadAnimationClips(filename,pScene);
	db.AddAnimation(animations);
}

bool FBXLoader::InitFromScene(const aiScene* pScene,
	const std::string& Filename,
	std::vector<Subset>& subsets,
	std::vector<Animation::SkinnedVertex>& vertices,
	std::vector<USHORT>& indices,
	std::vector<FbxMaterial>& mats)
{
	subsets.resize(pScene->mNumMeshes);

	UINT NumVertices = 0;
	UINT NumFaces = 0;
	// Count the number of vertices and indices
	for (UINT i = 0; i < subsets.size(); i++)
	{
		subsets[i].MaterialIndex = pScene->mMeshes[i]->mMaterialIndex;
		//subsets[i].MaterialIndex = 0;
		subsets[i].FaceStart = NumFaces;
		subsets[i].FaceCount = pScene->mMeshes[i]->mNumFaces;
		subsets[i].VertexStart = NumVertices;
		subsets[i].VertexCount = pScene->mMeshes[i]->mNumVertices;

		NumFaces += subsets[i].FaceCount;
		NumVertices += subsets[i].VertexCount;
	}

	// Reserve space in the vectors for the vertex attributes and indices
	vertices.resize(NumVertices);
	indices.reserve(NumFaces * 3);
	
	// Initialize the meshes in the scene one by one
	for (UINT i = 0; i < subsets.size(); i++)
	{
		const aiMesh* paiMesh = pScene->mMeshes[i];
		InitMesh(i, paiMesh, subsets, vertices, indices);
	}

	InitMaterials(mats, pScene);
	return true;
}

void FBXLoader::InitMesh(UINT MeshIndex, 
	const aiMesh* paiMesh,
	std::vector<Subset>& subsets,
	std::vector<Animation::SkinnedVertex>& vertices,
	std::vector<USHORT>& indices)
{
	const aiVector3D Zero3D(0.0f, 0.0f, 0.0f);

	// Populate the vertex attribute vectors
	UINT vertexIndex = subsets[MeshIndex].VertexStart;
	for (UINT i = 0; i < paiMesh->mNumVertices; i++)
	{
		Animation::SkinnedVertex vertex;
		const aiVector3D& Pos = paiMesh->mVertices[i];
		const aiVector3D& Normal = paiMesh->mNormals[i];
		const aiVector3D& Tangent = paiMesh->mTangents[i];
		const aiVector3D& TexCoord = paiMesh->HasTextureCoords(0) ? paiMesh->mTextureCoords[0][i] : Zero3D;	

		vertices[vertexIndex + i].Pos = XMFLOAT3(Pos.x, Pos.y, Pos.z);
		vertices[vertexIndex + i].Normal = XMFLOAT3(Normal.x, Normal.y, Normal.z);
		vertices[vertexIndex + i].TangentU = XMFLOAT3(Tangent.x, Tangent.y, Tangent.z);
		vertices[vertexIndex + i].TexC = XMFLOAT2(TexCoord.x, TexCoord.y);
	}

	LoadBones(MeshIndex, paiMesh, subsets, vertices, indices);

	// Populate the index buffer
	for (UINT i = 0; i < paiMesh->mNumFaces; i++)
	{
		const aiFace& Face = paiMesh->mFaces[i];
		if (Face.mIndices[0] <=subsets[MeshIndex].VertexCount&& Face.mIndices[1] <= subsets[MeshIndex].VertexCount&& Face.mIndices[2] <= subsets[MeshIndex].VertexCount)
		{
			indices.push_back(Face.mIndices[0] + vertexIndex);
			indices.push_back(Face.mIndices[1] + vertexIndex);
			indices.push_back(Face.mIndices[2] + vertexIndex);
		}
	}
}

void FBXLoader::LoadBones(UINT MeshIndex, 
	const aiMesh* pMesh,
	std::vector<Subset>& subsets,
	std::vector<Animation::SkinnedVertex>& vertices,
	std::vector<USHORT>& indices)
{
	for (UINT i = 0; i < pMesh->mNumBones; i++)
	{
		UINT BoneIndex = 0;
		std::string BoneName(pMesh->mBones[i]->mName.data);

		if (m_BoneMapping.find(BoneName) == m_BoneMapping.end())
		{
			// Allocate an index for a new bone
			BoneIndex = m_NumBones++;
			// Bone Info
			aiMatrix4x4 aiOffsetMatrix = pMesh->mBones[i]->mOffsetMatrix;

			// need to transpose
			aiOffsetMatrix.Transpose();
			Matrix OffsetMatrix(&aiOffsetMatrix.a1);
			jointOffsets.push_back(OffsetMatrix);
			m_BoneMapping[BoneName] = BoneIndex;
		}
		else
		{
			BoneIndex = m_BoneMapping[BoneName];
		}

		for (UINT j = 0; j < pMesh->mBones[i]->mNumWeights; j++)
		{
			UINT VertexID = subsets[MeshIndex].VertexStart + pMesh->mBones[i]->mWeights[j].mVertexId;
			float Weight = pMesh->mBones[i]->mWeights[j].mWeight;
			float BoneWeight[4] = { vertices[VertexID].BoneWeights.x, vertices[VertexID].BoneWeights.y, vertices[VertexID].BoneWeights.z, 0.0f };
			for (UINT i = 0; i < NUM_BONES_PER_VEREX; i++)
			{
				if (BoneWeight[i] == 0.0)
				{
					vertices[VertexID].BoneIndices[i] = BoneIndex;
					BoneWeight[i] = Weight;
					vertices[VertexID].BoneWeights = DirectX::XMFLOAT3(BoneWeight[0], BoneWeight[1], BoneWeight[2]);
					break;
				}
			}
		}
	}
}

std::vector<Matrix> FBXLoader::ReorganizeBoneOffsets(const aiScene* pScene)
{
	std::vector<Matrix> newBoneOffsets;
	newBoneOffsets.resize(jointOffsets.size());
	for (UINT i = 0; i < jointIndexToParentIndex.size(); i++)
	{
		const std::string& name = nodeIndexToName[i];
		UINT boneIndex = m_BoneMapping[name];
		newBoneOffsets[i] = jointOffsets[boneIndex];
	}

	return newBoneOffsets;
}

void FBXLoader::ReadAnimationClips(const std::string& path, const aiScene* pScene)
{
	char out[100];

	aiAnimation** aiAnimationClips = pScene->mAnimations;
	
	for (UINT i = 0; i < pScene->mNumAnimations; i++)
	{
		aiAnimation* aiAnimationClip = aiAnimationClips[i];
		AnimationClip animationClip;
		animationClip.mTicksPerSecond = aiAnimationClips[i]->mTicksPerSecond;
		animationClip.mSamples.resize(nodeIndexToName.size());

		for (UINT j = 0; j < aiAnimationClip->mNumChannels; j++)
		{
			aiNodeAnim* pNodeAnim = aiAnimationClip->mChannels[j];
			const std::string& boneName = pNodeAnim->mNodeName.data;
			std::vector<std::string>::iterator it = std::find(nodeIndexToName.begin(), nodeIndexToName.end(), boneName);
			if (it != nodeIndexToName.end())
			{
				UINT index = it - nodeIndexToName.begin();
				ReadBoneKeyframes(pNodeAnim, animationClip.mSamples[index]);
			}

		}

		std::string fileName = GetFileName(path);
		std::string clipName = aiAnimationClips[i]->mName.data;
		animationClip.mName = fileName;
		animations[fileName + "/" + clipName] = animationClip;
	}
}

void FBXLoader::ReadBoneKeyframes(aiNodeAnim* nodeAnim, BoneAnimationSample& boneAnimation)
{
	assert(nodeAnim->mNumPositionKeys == nodeAnim->mNumRotationKeys && nodeAnim->mNumRotationKeys == nodeAnim->mNumScalingKeys);
	boneAnimation.mName = nodeAnim->mNodeName.data;

	boneAnimation.mLocalPose.resize(nodeAnim->mNumPositionKeys);

	for (UINT i = 0; i < nodeAnim->mNumPositionKeys; i++)
	{
		const aiVectorKey& translationKey = nodeAnim->mPositionKeys[i];
		Vector3 translation;
		translation = Vector3(translationKey.mValue.x, translationKey.mValue.y, translationKey.mValue.z);
		boneAnimation.mLocalPose[i].mTrans.mValue = translation;
		boneAnimation.mLocalPose[i].mTrans.mTimeTick = translationKey.mTime;

		const aiQuatKey& rotationKey = nodeAnim->mRotationKeys[i];
		Quaternion rotation;
		rotation = rotationKey.mValue.w >= 0.0f?
			Quaternion(rotationKey.mValue.x, rotationKey.mValue.y, rotationKey.mValue.z, rotationKey.mValue.w):
			-Quaternion(rotationKey.mValue.x, rotationKey.mValue.y, rotationKey.mValue.z, rotationKey.mValue.w);
		boneAnimation.mLocalPose[i].mRot.mValue = rotation;
		boneAnimation.mLocalPose[i].mRot.mTimeTick = rotationKey.mTime;

		aiVectorKey scalingKey = nodeAnim->mScalingKeys[i];
		Vector3 scaling;
		scaling = Vector3(scalingKey.mValue.x, scalingKey.mValue.y, scalingKey.mValue.z);
		boneAnimation.mLocalPose[i].mScale.mValue = scaling;
		boneAnimation.mLocalPose[i].mScale.mTimeTick = scalingKey.mTime;
	}
}

void FBXLoader::CreateSkeletonHierachy(const aiScene* scene)
{
	aiNode* rootNode = scene->mRootNode;

	BuildNodeMappingWithBone(rootNode, scene);
	SetNodeMappingWithBone(rootNode, scene);
	CreateSkeletonHierachyWithNecessityMap(rootNode, -1);
}

void FBXLoader::BuildNodeMappingWithBone(aiNode* node, const aiScene* scene)
{
	necessityMap[node->mName.data] = false;
	Debug::Log(LOG_LEVEL::LOG_LEVEL_INFO, "BuildNodeMappingWithBone", "MotionAnalyzer", 411, node->mName.data);

	for (UINT i = 0; i < node->mNumChildren; i++)
	{
		this->BuildNodeMappingWithBone(node->mChildren[i], scene);
	}
}

void FBXLoader::SetNodeMappingWithBone(aiNode* node, const aiScene* scene)
{
	for (UINT i = 0; i < node->mNumMeshes; i++)
	{
		UINT meshIndex = node->mMeshes[i];
		aiMesh* mesh = scene->mMeshes[meshIndex];

		for (UINT j = 0; j < mesh->mNumBones; j++)
		{	
			// Mark this node as "yes" in the necessityMap
			aiBone* bone = mesh->mBones[j];
			const aiString& name = bone->mName;
			necessityMap[name.data] = true;

			// Mark all of its parents the same way until ...
			// 1. find the mesh's node or
			// 2. find parent of the mesh's node
			aiNode* parent = scene->mRootNode->FindNode(name);
			if (parent != nullptr)
			{
				parent = parent->mParent;
			}

			while (parent != node && parent != node->mParent && parent != nullptr)
			{
				necessityMap[parent->mName.data] = true;
				parent = parent->mParent;
			}
		}
	}

	for (UINT i = 0; i < node->mNumChildren; i++)
	{
		this->SetNodeMappingWithBone(node->mChildren[i], scene);
	}
}

void FBXLoader::CreateSkeletonHierachyWithNecessityMap(aiNode* node, int index)
{
	if (necessityMap[node->mName.data] == true)
	{
		std::string name = node->mName.C_Str();
		jointIndexToParentIndex.push_back(index);
		nodeIndexToName.push_back(name);
	}

	int parentIndex = jointIndexToParentIndex.size() - 1;
	for (UINT i = 0; i < node->mNumChildren; i++)
	{
		CreateSkeletonHierachyWithNecessityMap(node->mChildren[i], parentIndex);
	}
}

void FBXLoader::InitMaterials(std::vector<FbxMaterial>& mats, const aiScene* pScene)
{
	// Initialize the materials
	for (UINT i = 0; i < pScene->mNumMaterials; i++)
	{
		aiMaterial* pMat = pScene->mMaterials[i];
		FbxMaterial mat;

		aiString name;
		pMat->Get(AI_MATKEY_NAME, name);
		mat.Name = name.data;

		aiColor4D diffuse;
		pMat->Get(AI_MATKEY_COLOR_DIFFUSE, diffuse);
		mat.DiffuseAlbedo = XMFLOAT4(diffuse.r, diffuse.g, diffuse.b, diffuse.a);

		mat.DiffuseMapName = "white.dds";
		mat.NormalMapName = "default_nmap.dds";
		mat.MaterialTypeName = pScene->HasAnimations()?"Skinned":"NoSkinned";
		mat.FresnelR0 = { 0.05,0.05, 0.05 };

		mats.push_back(mat);
	}
}