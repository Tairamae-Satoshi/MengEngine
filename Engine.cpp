//***************************************************************************************
// Meng Engine is a skeleton animation engine.
//***************************************************************************************

#include "Common/d3dApp.h"
#include "Common/MathHelper.h"
#include "Common/UploadBuffer.h"
#include "Common/GeometryGenerator.h"
#include "Common/Camera.h"
#include "Renderer/VertexFactory.h"
#include "Renderer/FrameResource.h"
#include "Renderer/Material.h"
#include "Renderer/ResourceManager.h"
#include "Animation/LoadFBX.h"
#include "Animation/Utils.h"
#include "Animation/GradientBandInterpolator.h"
#include "Animation/MotionAnalyzer.h"
#include "Animation/Character/Character.h"
#define STB_IMAGE_IMPLEMENTATION
#include "Common/stb_image.h"
#include "DirectXTex/DirectXTex/DirectXTex.h"

using namespace Animation;


// Lightweight structure stores parameters to draw a shape.  This will
// vary from app-to-app.
struct RenderItem
{
	RenderItem() = default;
	RenderItem(const RenderItem& rhs) = delete;

	// World matrix of the shape that describes the object's local space
	// relative to the world space, which defines the position, orientation,
	// and scale of the object in the world.
	Matrix World = MathHelper::Identity4x4();

	Matrix TexTransform = MathHelper::Identity4x4();

	// Index into GPU constant buffer corresponding to the ObjectCB for this render item.
	UINT ObjCBIndex = -1;

	Material* Mat = nullptr;
	MeshGeometry* Geo = nullptr;

	// Primitive topology.
	D3D12_PRIMITIVE_TOPOLOGY PrimitiveType = D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST;

	// DrawIndexedInstanced parameters.
	UINT IndexCount = 0;
	UINT StartIndexLocation = 0;
	int BaseVertexLocation = 0;

	// Only applicable to skinned render-items.
	UINT SkinnedCBIndex = -1;

	// nullptr if this render-item is not animated by skinned mesh.
	const AnimationClip* SkinnedModelInst = nullptr;

	Vector4 Color = Vector4::One;
	//PBRMaterialConstants* materialCB = nullptr;
};

enum class RenderLayer : int
{
	Opaque = 0,
	SkinnedOpaque,
	Checkboard,
	Debug,
	Sky,
	Count
};

class Engine : public D3DApp
{
public:
    Engine(HINSTANCE hInstance);
    Engine(const Engine& rhs) = delete;
    Engine& operator=(const Engine& rhs) = delete;
    ~Engine();

    virtual bool Initialize()override;

private:
    virtual void OnResize()override;
    virtual void Update(const GameTimer& gt)override;
    virtual void Render(const GameTimer& gt)override;

    virtual void OnMouseDown(WPARAM btnState, int x, int y)override;
    virtual void OnMouseUp(WPARAM btnState, int x, int y)override;
    virtual void OnMouseMove(WPARAM btnState, int x, int y)override;
	void OnKeyboardInput(const GameTimer& gt);

    void UpdateSkinnedCBs(void* perPassCB, const GameTimer& gt);
	void UpdateMainPassCB(void* perPassCB, const GameTimer& gt);
	void UpdateGUI();
	void UpdateTarget(const GameTimer& gt);
	void UpdateShadowTransform(const GameTimer& gt);
	void UpdateShadowPerPassCB(const GameTimer& gt);

    void BuildShapeGeometry();
	void LoadSkinnedModel();
    void BuildMaterials();
    void BuildRenderItems();
	void BuildGUI();
    void DrawRenderItems(Graphics::GraphicsContext& graphicsContext, Graphics::PipelineState* pipelineState,Graphics::GpuDynamicBuffer* perDrawCB,const std::vector<RenderItem*>& ritems, bool useMaterial = true);

private:
	// Main Pass
	std::shared_ptr<Graphics::GpuDynamicBuffer> m_PerDrawCB = nullptr;
	std::shared_ptr<Graphics::GpuDynamicBuffer> m_PerPassCB = nullptr;
	std::shared_ptr<Graphics::GpuDynamicBuffer> m_LightCB = nullptr;
	std::shared_ptr<Graphics::GpuDynamicBuffer> m_MaterialCB = nullptr;

	std::shared_ptr<Graphics::Shader> m_StandardVS = nullptr;
	std::shared_ptr<Graphics::Shader> m_StandardPS = nullptr;
	std::unique_ptr<Graphics::PipelineState> m_MainPassPSO = nullptr;

	// Skinned Pass
	std::shared_ptr<Graphics::GpuDynamicBuffer> m_SkinnedPerDrawCB = nullptr;
	std::shared_ptr<Graphics::GpuDynamicBuffer> m_SkinnedPerPassCB = nullptr;
	std::shared_ptr<Graphics::GpuDynamicBuffer> m_SkinnedLightCB = nullptr;
	std::shared_ptr<Graphics::GpuDynamicBuffer> m_SkinnedCB = nullptr;
	std::shared_ptr<Graphics::GpuDynamicBuffer> m_SkinnedMaterialCB = nullptr;

	std::shared_ptr<Graphics::Shader> m_SkinnedVS = nullptr;
	std::shared_ptr<Graphics::Shader> m_SkinnedPS = nullptr;
	std::unique_ptr<Graphics::PipelineState> m_SkinnedPassPSO = nullptr;

	// Checkboard Pass
	std::shared_ptr<Graphics::GpuDynamicBuffer> m_CkBPerDrawCB = nullptr;
	std::shared_ptr<Graphics::GpuDynamicBuffer> m_CkBPerPassCB = nullptr;
	std::shared_ptr<Graphics::GpuDynamicBuffer> m_CkBLightCB = nullptr;
	std::shared_ptr<Graphics::GpuDynamicBuffer> m_CkBMaterialCB = nullptr;

	std::shared_ptr<Graphics::Shader> m_CkBVS = nullptr;
	std::shared_ptr<Graphics::Shader> m_CkBPS = nullptr;
	std::unique_ptr<Graphics::PipelineState> m_CkBPSO = nullptr;


	// ShadowMap Pass
	std::shared_ptr<Graphics::GpuDynamicBuffer> m_ShadowMapPerDrawCB = nullptr;
	std::shared_ptr<Graphics::GpuDynamicBuffer> m_ShadowMapPerPassCB = nullptr;

	std::shared_ptr<Graphics::Shader> m_ShadowMapVS = nullptr;
	std::shared_ptr<Graphics::Shader> m_ShadowMapPS = nullptr;
	std::unique_ptr<Graphics::PipelineState> m_ShadowMapPSO = nullptr;

	// Skinned ShadowMap Pass
	std::shared_ptr<Graphics::GpuDynamicBuffer> m_SkinnedShadowMapPerDrawCB = nullptr;
	std::shared_ptr<Graphics::GpuDynamicBuffer> m_SkinnedShadowMapPerPassCB = nullptr;
	std::shared_ptr<Graphics::GpuDynamicBuffer> m_SkinnedShadowSkinnedCB = nullptr;

	std::shared_ptr<Graphics::Shader> m_SkinnedShadowMapVS = nullptr;
	std::shared_ptr<Graphics::Shader> m_SkinnedShadowMapPS = nullptr;
	std::unique_ptr<Graphics::PipelineState> m_SkinnedShadowMapPSO = nullptr;

	std::shared_ptr<Graphics::GpuRenderTextureDepth> m_ShadowMap = nullptr;
	std::shared_ptr<Graphics::GpuResourceDescriptor> m_ShadowMapDSV = nullptr;
	std::shared_ptr<Graphics::GpuResourceDescriptor> m_ShadowMapSRV = nullptr;

	const float m_ShadowMapSize = 4096.0f;
	CD3DX12_VIEWPORT m_ShadowMapViewport;
	CD3DX12_RECT m_ShadowMapScissorRect;


	// List of all the render items.
	std::vector<std::unique_ptr<RenderItem>> mAllRitems;
	std::unordered_map<std::string, std::unique_ptr<MeshGeometry>> mGeometries;

	std::unordered_map<std::string, std::unique_ptr<Material>> mMaterials;
	std::unordered_map<std::string, std::unique_ptr<Texture>> mTextures;

	// Render items divided by PSO.
	std::vector<RenderItem*> mRitemLayer[(int)RenderLayer::Count];

    PassConstants mMainPassCB;  // index 0 of pass cbuffer.
    PassConstants mShadowPassCB;// index 1 of pass cbuffer.
	LightConstants mLightConstants;
	SkinnedConstants mSkinnedConstants;
	PBRMaterialConstants mMaterialConstants[2];

	// Animation
	enum { Animation_Num = 9 };

	const std::string mSkeletonName = "mixamo";

	const std::string bind_pose_filename = "Contents/Models/bind.fbx";

	//const std::string mAnimationFilename[Animation_Num] =
	//{
	//	"Contents/Models/Locomotion//Standard Idle.fbx",
	//	"Contents/Models/Locomotion//Standard Walk.fbx",
	//	"Contents/Models/Locomotion//Standard Run.fbx",
	//	"Contents/Models/Locomotion/Walking Backwards.fbx",
	//	"Contents/Models/Locomotion/Running Backward.fbx",
	//	"Contents/Models/Locomotion/left strafe walking.fbx",
	//	"Contents/Models/Locomotion/right strafe walking.fbx",
	//	"Contents/Models/Locomotion/left strafe.fbx",
	//	"Contents/Models/Locomotion/right strafe.fbx"
	//};

	// Data for motion matching
	const std::string mAnimationFilename[Animation_Num] =
	{
		"Contents/Models/Locomotion//Standard Idle.fbx",
		"Contents/Models/Locomotion//Standard Walk.fbx",
		"Contents/Models/Locomotion//Standard Run.fbx",
		"Contents/Models/Locomotion/Walking Backwards.fbx",
		"Contents/Models/Locomotion/Running Backward.fbx",
		"Contents/Models/Locomotion/left strafe walking.fbx",
		"Contents/Models/Locomotion/right strafe walking.fbx",
		"Contents/Models/Locomotion/left strafe.fbx",
		"Contents/Models/Locomotion/right strafe.fbx"
	};


	//const std::string bind_pose_filename = "Contents/Models/Standard Idle.fbx";

	//const std::string mAnimationFilename[Animation_Num] =
	//{
	//	"Contents/Models/Female Basic Locomotion Pack/idle.fbx",
	//	"Contents/Models/Female Basic Locomotion Pack/walking.fbx",
	//	"Contents/Models/Female Basic Locomotion Pack/running.fbx",
	//	"Contents/Models/Female Basic Locomotion Pack/left strafe walk.fbx",
	//	"Contents/Models/Female Basic Locomotion Pack/right strafe walk.fbx",
	//	"Contents/Models/Female Basic Locomotion Pack/left strafe.fbx",
	//	"Contents/Models/Female Basic Locomotion Pack/right strafe.fbx",
	//};

	MotionAnalyzer analyzer[Animation_Num];
	PlaybackController mController;
	Character character;

	// Two Bone IK
	RenderItem* pTargetRitem;
	Vector3 target = Vector3(-1.0f, 2.0f, -3.0f);

	// Head Aim IK
	RenderItem* pHeadAimTargetRitem;
	Vector3 head_target_offset = Vector3(-1.0f, 7.0f, -3.0f);
	Vector3 head_target = Vector3(-1.0f, 10.0f, -3.0f);

	// Trajectory Points
	RenderItem* pTrajectoryPointRitems[4];
	RenderItem* pMatchedPointRitems[4];

	std::shared_ptr<PolarGradientBandInterpolator> interpolator;

	const AnimationClip* mSamplers[Animation_Num] = { nullptr };
	std::vector<FBXLoader::Subset> mSkinnedSubsets;
    std::vector<FBXLoader::FbxMaterial> mSkinnedMats;
    std::vector<std::string> mSkinnedTextureNames;

	BlendingJob blender;

	Camera mCamera;

    DirectX::BoundingSphere mSceneBounds;

    float mLightNearZ = 0.0f;
    float mLightFarZ = 0.0f;
    XMFLOAT3 mLightPosW;
    XMFLOAT4X4 mLightView = MathHelper::Identity4x4();
    XMFLOAT4X4 mLightProj = MathHelper::Identity4x4();
    XMFLOAT4X4 mShadowTransform = MathHelper::Identity4x4();

    float mLightRotationAngle = 0.0f;
    XMFLOAT3 mBaseLightDirections[3] = {
        XMFLOAT3(0.57735f, -0.57735f, 0.57735f),
        XMFLOAT3(-0.57735f, -0.57735f, 0.57735f),
        XMFLOAT3(0.0f, -0.707f, -0.707f)
    };
    XMFLOAT3 mRotatedLightDirections[3];

    POINT mLastMousePos;

};

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE prevInstance,
    PSTR cmdLine, int showCmd)
{
    // Enable run-time memory check for debug builds.
#if defined(DEBUG) | defined(_DEBUG)
    _CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
#endif

    try
    {
        Engine theApp(hInstance);
        if(!theApp.Initialize())
            return 0;

        return theApp.Run();
    }
    catch(DxException& e)
    {
        MessageBox(nullptr, e.ToString().c_str(), L"HR Failed", MB_OK);
        return 0;
    }
}

Engine::Engine(HINSTANCE hInstance)
    : D3DApp(hInstance)
{
    // Estimate the scene bounding sphere manually since we know how the scene was constructed.
    // The grid is the "widest object" with a width of 20 and depth of 30.0f, and centered at
    // the world space origin.  In general, you need to loop over every world space vertex
    // position and compute the bounding sphere.
    mSceneBounds.Center = XMFLOAT3(0.0f, 0.0f, 0.0f);
    mSceneBounds.Radius = sqrtf(10.0f*10.0f + 15.0f*15.0f);
}

Engine::~Engine()
{
	D3DApp::~D3DApp();
}

bool Engine::Initialize()
{
    if(!D3DApp::Initialize())
        return false;

	//
	// PSO for main pass.
	//
	Graphics::ShaderCreateInfo shaderCI;

	shaderCI.FilePath = L"Shaders\\Default.hlsl";
	shaderCI.EntryPoint = "VS";
	//const D3D_SHADER_MACRO opaqueDefines[] =
	//{
	//	"OPAQUE", "1",
	//	NULL, NULL
	//};
	//shaderCI.d3dMacros = opaqueDefines;
	shaderCI.Desc.ShaderType = Graphics::SHADER_TYPE_VERTEX;
	m_StandardVS = std::make_shared<Graphics::Shader>(shaderCI);

	shaderCI.EntryPoint = "PS";
	shaderCI.Desc.ShaderType = Graphics::SHADER_TYPE_PIXEL;
	m_StandardPS = std::make_shared<Graphics::Shader>(shaderCI);

	Graphics::PipelineStateDesc PSODesc;
	PSODesc.PipelineType = Graphics::PIPELINE_TYPE_GRAPHIC;
	PSODesc.GraphicsPipeline.VertexShader = m_StandardVS;
	PSODesc.GraphicsPipeline.PixelShader = m_StandardPS;
	PSODesc.GraphicsPipeline.GraphicPipelineState.RasterizerState = CD3DX12_RASTERIZER_DESC(D3D12_DEFAULT);
	//PSODesc.GraphicsPipeline.GraphicPipelineState.RasterizerState.FillMode = D3D12_FILL_MODE_WIREFRAME;
	PSODesc.GraphicsPipeline.GraphicPipelineState.BlendState = CD3DX12_BLEND_DESC(D3D12_DEFAULT);
	PSODesc.GraphicsPipeline.GraphicPipelineState.DepthStencilState = CD3DX12_DEPTH_STENCIL_DESC(D3D12_DEFAULT);
	PSODesc.GraphicsPipeline.GraphicPipelineState.SampleMask = UINT_MAX;
	PSODesc.GraphicsPipeline.GraphicPipelineState.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
	PSODesc.GraphicsPipeline.GraphicPipelineState.NumRenderTargets = 1;
	PSODesc.GraphicsPipeline.GraphicPipelineState.RTVFormats[0] = DXGI_FORMAT_R8G8B8A8_UNORM;
	PSODesc.GraphicsPipeline.GraphicPipelineState.SampleDesc.Count = 1;
	PSODesc.GraphicsPipeline.GraphicPipelineState.SampleDesc.Quality = 0;
	// TODO: SwapChain的Depth Buffer格式需要跟这里相同
	PSODesc.GraphicsPipeline.GraphicPipelineState.DSVFormat = DXGI_FORMAT_D24_UNORM_S8_UINT;

	// TODO: InputLayout
	UINT inputLayoutIndex = VertexFactory::GetSingleton().GetInputLayoutIndex(false, true, true, true, false, false, false, false);
	std::vector<D3D12_INPUT_ELEMENT_DESC>* inputLayoutDesc = VertexFactory::GetSingleton().GetInputElementDesc(inputLayoutIndex);
	PSODesc.GraphicsPipeline.GraphicPipelineState.InputLayout.NumElements = inputLayoutDesc->size();
	PSODesc.GraphicsPipeline.GraphicPipelineState.InputLayout.pInputElementDescs = inputLayoutDesc->data();

	// shader变量更新频率
	PSODesc.VariableConfig.Variables.push_back(Graphics::ShaderResourceVariableDesc{ Graphics::SHADER_TYPE_PIXEL, "BaseColorTex", Graphics::SHADER_RESOURCE_VARIABLE_TYPE_MUTABLE });
	PSODesc.VariableConfig.Variables.push_back(Graphics::ShaderResourceVariableDesc{ Graphics::SHADER_TYPE_PIXEL, "MetallicRoughnessTex", Graphics::SHADER_RESOURCE_VARIABLE_TYPE_MUTABLE });
	PSODesc.VariableConfig.Variables.push_back(Graphics::ShaderResourceVariableDesc{ Graphics::SHADER_TYPE_PIXEL, "EmissiveTex", Graphics::SHADER_RESOURCE_VARIABLE_TYPE_MUTABLE });
	PSODesc.VariableConfig.Variables.push_back(Graphics::ShaderResourceVariableDesc{ Graphics::SHADER_TYPE_PIXEL, "cbMaterial", Graphics::SHADER_RESOURCE_VARIABLE_TYPE_MUTABLE });

	m_MainPassPSO = std::make_unique<Graphics::PipelineState>(&Graphics::RenderDevice::GetSingleton(), PSODesc);

	m_PerDrawCB = std::make_shared<Graphics::GpuDynamicBuffer>(1, sizeof(ObjectConstants));
	m_PerPassCB = std::make_shared<Graphics::GpuDynamicBuffer>(1, sizeof(PassConstants));
	m_LightCB = std::make_shared<Graphics::GpuDynamicBuffer>(1, sizeof(LightConstants));

	Graphics::ShaderVariable* perDrawVariable = m_MainPassPSO->GetStaticVariableByName(Graphics::SHADER_TYPE_VERTEX, "cbPerObject");
	perDrawVariable->Set(m_PerDrawCB);
	Graphics::ShaderVariable* perPassVariable = m_MainPassPSO->GetStaticVariableByName(Graphics::SHADER_TYPE_VERTEX, "cbPass");
	perPassVariable->Set(m_PerPassCB);
	Graphics::ShaderVariable* lightVariable = m_MainPassPSO->GetStaticVariableByName(Graphics::SHADER_TYPE_PIXEL, "cbLight");
	if (lightVariable != nullptr)
		lightVariable->Set(m_LightCB);
	/*Graphics::ShaderVariable* materialVariable = m_MainPassPSO->GetStaticVariableByName(Graphics::SHADER_TYPE_PIXEL, "cbMaterial");
	materialVariable->Set(m_MaterialCB);*/


	// PSO for checkboard
	{
		Graphics::ShaderCreateInfo shaderCI;

		shaderCI.FilePath = L"Shaders\\Default.hlsl";
		shaderCI.EntryPoint = "VS";
		shaderCI.d3dMacros = nullptr;
		shaderCI.Desc.ShaderType = Graphics::SHADER_TYPE_VERTEX;
		m_CkBVS = std::make_shared<Graphics::Shader>(shaderCI);

		shaderCI.EntryPoint = "PS";
		const D3D_SHADER_MACRO checkboardDefines[] =
		{
			"CHECKBOARD", "1",
			NULL, NULL
		};
		shaderCI.d3dMacros = checkboardDefines;
		shaderCI.Desc.ShaderType = Graphics::SHADER_TYPE_PIXEL;
		m_CkBPS = std::make_shared<Graphics::Shader>(shaderCI);

		Graphics::PipelineStateDesc PSODesc;
		PSODesc.PipelineType = Graphics::PIPELINE_TYPE_GRAPHIC;
		PSODesc.GraphicsPipeline.VertexShader = m_CkBVS;
		PSODesc.GraphicsPipeline.PixelShader = m_CkBPS;
		PSODesc.GraphicsPipeline.GraphicPipelineState.RasterizerState = CD3DX12_RASTERIZER_DESC(D3D12_DEFAULT);
		//PSODesc.GraphicsPipeline.GraphicPipelineState.RasterizerState.FillMode = D3D12_FILL_MODE_WIREFRAME;
		PSODesc.GraphicsPipeline.GraphicPipelineState.BlendState = CD3DX12_BLEND_DESC(D3D12_DEFAULT);
		PSODesc.GraphicsPipeline.GraphicPipelineState.DepthStencilState = CD3DX12_DEPTH_STENCIL_DESC(D3D12_DEFAULT);
		PSODesc.GraphicsPipeline.GraphicPipelineState.SampleMask = UINT_MAX;
		PSODesc.GraphicsPipeline.GraphicPipelineState.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
		PSODesc.GraphicsPipeline.GraphicPipelineState.NumRenderTargets = 1;
		PSODesc.GraphicsPipeline.GraphicPipelineState.RTVFormats[0] = DXGI_FORMAT_R8G8B8A8_UNORM;
		PSODesc.GraphicsPipeline.GraphicPipelineState.SampleDesc.Count = 1;
		PSODesc.GraphicsPipeline.GraphicPipelineState.SampleDesc.Quality = 0;
		PSODesc.GraphicsPipeline.GraphicPipelineState.DSVFormat = DXGI_FORMAT_D24_UNORM_S8_UINT;

		// TODO: InputLayout
		UINT inputLayoutIndex = VertexFactory::GetSingleton().GetInputLayoutIndex(false, true, true, true, false, false, false, false);
		std::vector<D3D12_INPUT_ELEMENT_DESC>* inputLayoutDesc = VertexFactory::GetSingleton().GetInputElementDesc(inputLayoutIndex);
		PSODesc.GraphicsPipeline.GraphicPipelineState.InputLayout.NumElements = inputLayoutDesc->size();
		PSODesc.GraphicsPipeline.GraphicPipelineState.InputLayout.pInputElementDescs = inputLayoutDesc->data();

		// shader变量更新频率
		PSODesc.VariableConfig.Variables.clear();
		PSODesc.VariableConfig.Variables.push_back(Graphics::ShaderResourceVariableDesc{ Graphics::SHADER_TYPE_PIXEL, "BaseColorTex", Graphics::SHADER_RESOURCE_VARIABLE_TYPE_MUTABLE });
		PSODesc.VariableConfig.Variables.push_back(Graphics::ShaderResourceVariableDesc{ Graphics::SHADER_TYPE_PIXEL, "MetallicRoughnessTex", Graphics::SHADER_RESOURCE_VARIABLE_TYPE_MUTABLE });
		PSODesc.VariableConfig.Variables.push_back(Graphics::ShaderResourceVariableDesc{ Graphics::SHADER_TYPE_PIXEL, "EmissiveTex", Graphics::SHADER_RESOURCE_VARIABLE_TYPE_MUTABLE });
		PSODesc.VariableConfig.Variables.push_back(Graphics::ShaderResourceVariableDesc{ Graphics::SHADER_TYPE_PIXEL, "cbMaterial", Graphics::SHADER_RESOURCE_VARIABLE_TYPE_MUTABLE });

		m_CkBPSO = std::make_unique<Graphics::PipelineState>(&Graphics::RenderDevice::GetSingleton(), PSODesc);

		m_CkBPerDrawCB = std::make_shared<Graphics::GpuDynamicBuffer>(1, sizeof(ObjectConstants));
		m_CkBPerPassCB = std::make_shared<Graphics::GpuDynamicBuffer>(1, sizeof(PassConstants));
		m_CkBLightCB = std::make_shared<Graphics::GpuDynamicBuffer>(1, sizeof(LightConstants));

		Graphics::ShaderVariable* ckBPerDrawVariable = m_CkBPSO->GetStaticVariableByName(Graphics::SHADER_TYPE_VERTEX, "cbPerObject");
		ckBPerDrawVariable->Set(m_CkBPerDrawCB);
		Graphics::ShaderVariable* ckBPerPassVariable = m_CkBPSO->GetStaticVariableByName(Graphics::SHADER_TYPE_VERTEX, "cbPass");
		ckBPerPassVariable->Set(m_CkBPerPassCB);
		Graphics::ShaderVariable* ckBlightVariable = m_CkBPSO->GetStaticVariableByName(Graphics::SHADER_TYPE_PIXEL, "cbLight");
		if (ckBlightVariable != nullptr)
			ckBlightVariable->Set(m_CkBLightCB);
		//Graphics::ShaderVariable* ckBMaterialVariable = m_CkBPSO->GetStaticVariableByName(Graphics::SHADER_TYPE_PIXEL, "cbMaterial");
		//ckBMaterialVariable->Set(m_CkBMaterialCB);
	}
	

	//
	// PSO for ShadowMap pass.
	//
	shaderCI.FilePath = L"Shaders\\Shadows.hlsl";
	shaderCI.EntryPoint = "VS";
	shaderCI.Desc.ShaderType = Graphics::SHADER_TYPE_VERTEX;
	m_ShadowMapVS = std::make_shared<Graphics::Shader>(shaderCI);

	shaderCI.EntryPoint = "PS";
	shaderCI.Desc.ShaderType = Graphics::SHADER_TYPE_PIXEL;
	m_ShadowMapPS = std::make_shared<Graphics::Shader>(shaderCI);

	PSODesc.GraphicsPipeline.VertexShader = m_ShadowMapVS;
	PSODesc.GraphicsPipeline.PixelShader = m_ShadowMapPS;
	PSODesc.GraphicsPipeline.GraphicPipelineState.NumRenderTargets = 0;
	PSODesc.GraphicsPipeline.GraphicPipelineState.RTVFormats[0] = DXGI_FORMAT_UNKNOWN;
	PSODesc.VariableConfig.Variables.clear();
	// Bias设置
	PSODesc.GraphicsPipeline.GraphicPipelineState.RasterizerState.DepthBias = 100000;
	PSODesc.GraphicsPipeline.GraphicPipelineState.RasterizerState.DepthBiasClamp = 0.0f;
	PSODesc.GraphicsPipeline.GraphicPipelineState.RasterizerState.SlopeScaledDepthBias = 1.0f;

	m_ShadowMapPSO = std::make_unique<Graphics::PipelineState>(&Graphics::RenderDevice::GetSingleton(), PSODesc);

	m_ShadowMapPerDrawCB = std::make_shared<Graphics::GpuDynamicBuffer>(1, sizeof(ObjectConstants));
	m_ShadowMapPerPassCB = std::make_shared<Graphics::GpuDynamicBuffer>(1, sizeof(PassConstants));

	Graphics::ShaderVariable* shadowMapPerDrawVariable = m_ShadowMapPSO->GetStaticVariableByName(Graphics::SHADER_TYPE_VERTEX, "cbPerObject");
	shadowMapPerDrawVariable->Set(m_ShadowMapPerDrawCB);
	Graphics::ShaderVariable* shadowMapPerPassVariable = m_ShadowMapPSO->GetStaticVariableByName(Graphics::SHADER_TYPE_VERTEX, "cbPass");
	shadowMapPerPassVariable->Set(m_ShadowMapPerPassCB);


	//
	// PSO for skinned pass.
	//
	shaderCI.FilePath = L"Shaders\\Default.hlsl";
	shaderCI.EntryPoint = "VS";
	const D3D_SHADER_MACRO skinnedDefines[] =
	{
		"SKINNED", "1",
		NULL, NULL
	};
	shaderCI.d3dMacros = skinnedDefines;
	shaderCI.Desc.ShaderType = Graphics::SHADER_TYPE_VERTEX;
	m_SkinnedVS = std::make_shared<Graphics::Shader>(shaderCI);

	shaderCI.EntryPoint = "PS";
	shaderCI.Desc.ShaderType = Graphics::SHADER_TYPE_PIXEL;
	m_SkinnedPS = std::make_shared<Graphics::Shader>(shaderCI);

	PSODesc.GraphicsPipeline.VertexShader = m_SkinnedVS;
	PSODesc.GraphicsPipeline.PixelShader = m_SkinnedPS;
	PSODesc.GraphicsPipeline.GraphicPipelineState.RasterizerState = CD3DX12_RASTERIZER_DESC(D3D12_DEFAULT);
	//PSODesc.GraphicsPipeline.GraphicPipelineState.RasterizerState.FillMode = D3D12_FILL_MODE_WIREFRAME;
	PSODesc.GraphicsPipeline.GraphicPipelineState.BlendState = CD3DX12_BLEND_DESC(D3D12_DEFAULT);
	PSODesc.GraphicsPipeline.GraphicPipelineState.DepthStencilState = CD3DX12_DEPTH_STENCIL_DESC(D3D12_DEFAULT);
	PSODesc.GraphicsPipeline.GraphicPipelineState.SampleMask = UINT_MAX;
	PSODesc.GraphicsPipeline.GraphicPipelineState.PrimitiveTopologyType = D3D12_PRIMITIVE_TOPOLOGY_TYPE_TRIANGLE;
	PSODesc.GraphicsPipeline.GraphicPipelineState.NumRenderTargets = 1;
	PSODesc.GraphicsPipeline.GraphicPipelineState.RTVFormats[0] = DXGI_FORMAT_R8G8B8A8_UNORM;
	PSODesc.GraphicsPipeline.GraphicPipelineState.SampleDesc.Count = 1;
	PSODesc.GraphicsPipeline.GraphicPipelineState.SampleDesc.Quality = 0;
	// TODO: SwapChain的Depth Buffer格式需要跟这里相同
	PSODesc.GraphicsPipeline.GraphicPipelineState.DSVFormat = DXGI_FORMAT_D24_UNORM_S8_UINT;

	inputLayoutIndex = VertexFactory::GetSingleton().GetInputLayoutIndex(false, true, true, true, false, false, false, true);
	inputLayoutDesc = VertexFactory::GetSingleton().GetInputElementDesc(inputLayoutIndex);
	PSODesc.GraphicsPipeline.GraphicPipelineState.InputLayout.NumElements = inputLayoutDesc->size();
	PSODesc.GraphicsPipeline.GraphicPipelineState.InputLayout.pInputElementDescs = inputLayoutDesc->data();

	// shader
	PSODesc.VariableConfig.Variables.clear();
	PSODesc.VariableConfig.Variables.push_back(Graphics::ShaderResourceVariableDesc{ Graphics::SHADER_TYPE_PIXEL, "BaseColorTex", Graphics::SHADER_RESOURCE_VARIABLE_TYPE_MUTABLE });
	PSODesc.VariableConfig.Variables.push_back(Graphics::ShaderResourceVariableDesc{ Graphics::SHADER_TYPE_PIXEL, "MetallicRoughnessTex", Graphics::SHADER_RESOURCE_VARIABLE_TYPE_MUTABLE });
	PSODesc.VariableConfig.Variables.push_back(Graphics::ShaderResourceVariableDesc{ Graphics::SHADER_TYPE_PIXEL, "EmissiveTex", Graphics::SHADER_RESOURCE_VARIABLE_TYPE_MUTABLE });
	PSODesc.VariableConfig.Variables.push_back(Graphics::ShaderResourceVariableDesc{ Graphics::SHADER_TYPE_PIXEL, "cbMaterial", Graphics::SHADER_RESOURCE_VARIABLE_TYPE_MUTABLE });

	m_SkinnedPassPSO = std::make_unique<Graphics::PipelineState>(&Graphics::RenderDevice::GetSingleton(), PSODesc);

	m_SkinnedPerDrawCB = std::make_shared<Graphics::GpuDynamicBuffer>(1, sizeof(ObjectConstants));
	m_SkinnedPerPassCB = std::make_shared<Graphics::GpuDynamicBuffer>(1, sizeof(PassConstants));
	m_SkinnedLightCB = std::make_shared<Graphics::GpuDynamicBuffer>(1, sizeof(LightConstants));
	m_SkinnedCB = std::make_shared<Graphics::GpuDynamicBuffer>(1, sizeof(SkinnedConstants));

	Graphics::ShaderVariable* skinnedPerDrawVariable = m_SkinnedPassPSO->GetStaticVariableByName(Graphics::SHADER_TYPE_VERTEX, "cbPerObject");
	skinnedPerDrawVariable->Set(m_SkinnedPerDrawCB);
	Graphics::ShaderVariable* skinnedPerPassVariable = m_SkinnedPassPSO->GetStaticVariableByName(Graphics::SHADER_TYPE_VERTEX, "cbPass");
	skinnedPerPassVariable->Set(m_SkinnedPerPassCB);
	Graphics::ShaderVariable* skinnedVariable = m_SkinnedPassPSO->GetStaticVariableByName(Graphics::SHADER_TYPE_VERTEX, "cbSkinned");
	skinnedVariable->Set(m_SkinnedCB);
	Graphics::ShaderVariable* skinnedlightVariable = m_SkinnedPassPSO->GetStaticVariableByName(Graphics::SHADER_TYPE_PIXEL, "cbLight");
	if (skinnedlightVariable != nullptr)
		skinnedlightVariable->Set(m_SkinnedLightCB);
	//Graphics::ShaderVariable* skinnedMaterialVariable = m_SkinnedPassPSO->GetStaticVariableByName(Graphics::SHADER_TYPE_PIXEL, "cbMaterial");
	//skinnedMaterialVariable->Set(m_SkinnedMaterialCB);

	//
	// PSO for Skinned ShadowMap pass.
	//
	shaderCI.FilePath = L"Shaders\\Shadows.hlsl";
	shaderCI.EntryPoint = "VS";
	shaderCI.Desc.ShaderType = Graphics::SHADER_TYPE_VERTEX;
	m_SkinnedShadowMapVS = std::make_shared<Graphics::Shader>(shaderCI);

	shaderCI.EntryPoint = "PS";
	shaderCI.Desc.ShaderType = Graphics::SHADER_TYPE_PIXEL;
	m_SkinnedShadowMapPS = std::make_shared<Graphics::Shader>(shaderCI);

	PSODesc.GraphicsPipeline.VertexShader = m_SkinnedShadowMapVS;
	PSODesc.GraphicsPipeline.PixelShader = m_SkinnedShadowMapPS;
	PSODesc.GraphicsPipeline.GraphicPipelineState.NumRenderTargets = 0;
	PSODesc.GraphicsPipeline.GraphicPipelineState.RTVFormats[0] = DXGI_FORMAT_UNKNOWN;
	PSODesc.VariableConfig.Variables.clear();
	// Bias设置
	PSODesc.GraphicsPipeline.GraphicPipelineState.RasterizerState.DepthBias = 100000;
	PSODesc.GraphicsPipeline.GraphicPipelineState.RasterizerState.DepthBiasClamp = 0.0f;
	PSODesc.GraphicsPipeline.GraphicPipelineState.RasterizerState.SlopeScaledDepthBias = 1.0f;

	m_SkinnedShadowMapPSO = std::make_unique<Graphics::PipelineState>(&Graphics::RenderDevice::GetSingleton(), PSODesc);

	m_SkinnedShadowMapPerDrawCB = std::make_shared<Graphics::GpuDynamicBuffer>(1, sizeof(ObjectConstants));
	m_SkinnedShadowMapPerPassCB = std::make_shared<Graphics::GpuDynamicBuffer>(1, sizeof(PassConstants));
	m_SkinnedShadowSkinnedCB = std::make_shared<Graphics::GpuDynamicBuffer>(1, sizeof(SkinnedConstants));

	Graphics::ShaderVariable* skinnedShadowMapPerDrawVariable = m_SkinnedShadowMapPSO->GetStaticVariableByName(Graphics::SHADER_TYPE_VERTEX, "cbPerObject");
	skinnedShadowMapPerDrawVariable->Set(m_SkinnedShadowMapPerDrawCB);
	Graphics::ShaderVariable* skinnedShadowMapPerPassVariable = m_SkinnedShadowMapPSO->GetStaticVariableByName(Graphics::SHADER_TYPE_VERTEX, "cbPass");
	skinnedShadowMapPerPassVariable->Set(m_SkinnedShadowMapPerPassCB);
	Graphics::ShaderVariable* skinnedShadowSkinnedVariable = m_SkinnedShadowMapPSO->GetStaticVariableByName(Graphics::SHADER_TYPE_VERTEX, "cbSkinned");
	skinnedShadowSkinnedVariable->Set(m_SkinnedShadowSkinnedCB);

	// 绑定ShadowMap
	m_ShadowMap = std::make_shared<Graphics::GpuRenderTextureDepth>(m_ShadowMapSize, m_ShadowMapSize, DXGI_FORMAT_R24G8_TYPELESS);
	m_ShadowMap->SetName(L"ShadowMap");
	m_ShadowMapDSV = m_ShadowMap->CreateDSV();
	m_ShadowMapSRV = m_ShadowMap->CreateDepthSRV();

	m_ShadowMapViewport = CD3DX12_VIEWPORT(0.0f, 0.0f, static_cast<float>(m_ShadowMapSize), static_cast<float>(m_ShadowMapSize));
	m_ShadowMapScissorRect = CD3DX12_RECT(0, 0, static_cast<LONG>(m_ShadowMapSize), static_cast<LONG>(m_ShadowMapSize));

	Graphics::ShaderVariable* shadowMapVariable = m_MainPassPSO->GetStaticVariableByName(Graphics::SHADER_TYPE_PIXEL, "ShadowMap");
	if (shadowMapVariable != nullptr)
		shadowMapVariable->Set(m_ShadowMapSRV);
	shadowMapVariable = m_SkinnedPassPSO->GetStaticVariableByName(Graphics::SHADER_TYPE_PIXEL, "ShadowMap");
	if (shadowMapVariable != nullptr)
		shadowMapVariable->Set(m_ShadowMapSRV);

	shadowMapVariable = m_CkBPSO->GetStaticVariableByName(Graphics::SHADER_TYPE_PIXEL, "ShadowMap");
	if (shadowMapVariable != nullptr)
		shadowMapVariable->Set(m_ShadowMapSRV);
	shadowMapVariable = m_SkinnedPassPSO->GetStaticVariableByName(Graphics::SHADER_TYPE_PIXEL, "ShadowMap");
	if (shadowMapVariable != nullptr)
		shadowMapVariable->Set(m_ShadowMapSRV);



	// Set Camera
	mCamera.SetPosition(0.0f, 25.0f, -50.0f);
	mCamera.SetLens(0.25f * MathHelper::Pi, AspectRatio(), 1.0f, 1000.0f);

	LoadSkinnedModel();
	BuildShapeGeometry();
	BuildMaterials();
	BuildRenderItems();
	BuildGUI();

    return true;
}
 
void Engine::OnResize()
{
    D3DApp::OnResize();

	mCamera.SetLens(0.25f*MathHelper::Pi, AspectRatio(), 1.0f, 1000.0f);
}

void Engine::Update(const GameTimer& gt)
{
    OnKeyboardInput(gt);

	//mLightRotationAngle += 0.1f * gt.DeltaTime();

	XMMATRIX R = XMMatrixRotationY(mLightRotationAngle);
	for (int i = 0; i < 3; ++i)
	{
		XMVECTOR lightDir = XMLoadFloat3(&mBaseLightDirections[i]);
		lightDir = XMVector3TransformNormal(lightDir, R);
		XMStoreFloat3(&mRotatedLightDirections[i], lightDir);
	}

	UpdateShadowTransform(gt);
	UpdateShadowPerPassCB(gt);
	UpdateMainPassCB(nullptr, gt);
	UpdateSkinnedCBs(nullptr, gt);
	UpdateTarget(gt);
}


void Engine::Render(const GameTimer& gt)
{
	Graphics::GraphicsContext& graphicsContext = Graphics::GraphicsContext::Begin(L"ForwardRenderer");

	ID3D12DescriptorHeap* cbvsrvuavHeap = Graphics::RenderDevice::GetSingleton().GetGPUDescriptorHeap(D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV).GetD3D12DescriptorHeap();
	ID3D12DescriptorHeap* samplerHeap = Graphics::RenderDevice::GetSingleton().GetGPUDescriptorHeap(D3D12_DESCRIPTOR_HEAP_TYPE_SAMPLER).GetD3D12DescriptorHeap();
	graphicsContext.SetDescriptorHeap(cbvsrvuavHeap, samplerHeap);

	Graphics::SwapChain& swapChain = *m_SwapChain;
	// <------------------------------------ShadowMap Pass---------------------------------------------->
	graphicsContext.SetViewport(m_ShadowMapViewport);
	graphicsContext.SetScissor(m_ShadowMapScissorRect);

	graphicsContext.TransitionResource(*m_ShadowMap, D3D12_RESOURCE_STATE_DEPTH_WRITE);
	graphicsContext.ClearDepthAndStencil(*m_ShadowMapDSV);
	graphicsContext.SetRenderTargets(0, nullptr, m_ShadowMapDSV.get());

	// Draw skinned to shadowmap
	graphicsContext.SetPipelineState(m_SkinnedShadowMapPSO.get());

	void* pSkinnedShadowPerPassCB = m_SkinnedShadowMapPerPassCB->Map(graphicsContext, 256);
	memcpy(pSkinnedShadowPerPassCB, &mShadowPassCB, sizeof(mShadowPassCB));

	void* pShadowSkinnedCB = m_SkinnedShadowSkinnedCB->Map(graphicsContext, 256);
	memcpy(pShadowSkinnedCB, &mSkinnedConstants, sizeof(mSkinnedConstants));

	DrawRenderItems(graphicsContext, m_SkinnedShadowMapPSO.get(), m_SkinnedShadowMapPerDrawCB.get(), mRitemLayer[(int)RenderLayer::SkinnedOpaque], false);

	// Draw opaque to shadowmap
	graphicsContext.SetPipelineState(m_ShadowMapPSO.get());

	void* pShadowPerPassCB = m_ShadowMapPerPassCB->Map(graphicsContext, 256);
	memcpy(pShadowPerPassCB, &mShadowPassCB, sizeof(mShadowPassCB));

	DrawRenderItems(graphicsContext, m_ShadowMapPSO.get(), m_ShadowMapPerDrawCB.get(), mRitemLayer[(int)RenderLayer::Checkboard], false);
	DrawRenderItems(graphicsContext, m_ShadowMapPSO.get(), m_ShadowMapPerDrawCB.get(), mRitemLayer[(int)RenderLayer::Opaque], false);

	// Shadow Map过度到Read状态
	graphicsContext.TransitionResource(*m_ShadowMap, D3D12_RESOURCE_STATE_GENERIC_READ);

	graphicsContext.SetViewport(swapChain.GetViewport());
	graphicsContext.SetScissor(swapChain.GetScissorRect());

	Graphics::GpuResource* backBuffer = swapChain.GetCurBackBuffer();
	Graphics::GpuResourceDescriptor* backBufferRTV = swapChain.GetCurBackBufferRTV();
	Graphics::GpuResource* depthStencilBuffer = swapChain.GetDepthStencilBuffer();
	Graphics::GpuResourceDescriptor* depthStencilBufferDSV = swapChain.GetDepthStencilDSV();

	graphicsContext.TransitionResource(*backBuffer, D3D12_RESOURCE_STATE_RENDER_TARGET);
	graphicsContext.TransitionResource(*depthStencilBuffer, D3D12_RESOURCE_STATE_DEPTH_WRITE);
	graphicsContext.ClearColor(*backBufferRTV, Colors::LightSteelBlue);
	graphicsContext.ClearDepthAndStencil(*depthStencilBufferDSV);
	graphicsContext.SetRenderTargets(1, &backBufferRTV, depthStencilBufferDSV);

	// <------------------------------------Floor Pass---------------------------------------------->
	graphicsContext.SetPipelineState(m_CkBPSO.get());

	void* ckBPerPassCB = m_CkBPerPassCB->Map(graphicsContext, 256);
	memcpy(ckBPerPassCB, &mMainPassCB, sizeof(mMainPassCB));

	void* ckBLightCB = m_CkBLightCB->Map(graphicsContext, 256);
	memcpy(ckBLightCB, &mLightConstants, sizeof(mLightConstants));

	DrawRenderItems(graphicsContext, m_CkBPSO.get(), m_CkBPerDrawCB.get(), mRitemLayer[(int)RenderLayer::Checkboard]);

	// <------------------------------------Main Pass---------------------------------------------->

	graphicsContext.SetPipelineState(m_MainPassPSO.get());

	void* perPassCB = m_PerPassCB->Map(graphicsContext, 256);
	memcpy(perPassCB, &mMainPassCB, sizeof(mMainPassCB));

	void* lightCB = m_LightCB->Map(graphicsContext, 256);
	memcpy(lightCB, &mLightConstants, sizeof(mLightConstants));

	DrawRenderItems(graphicsContext, m_MainPassPSO.get(), m_PerDrawCB.get(), mRitemLayer[(int)RenderLayer::Opaque]);

	// <------------------------------------Skinned Pass---------------------------------------------->
	graphicsContext.SetPipelineState(m_SkinnedPassPSO.get());
	
	void* skinnedPerPassCB = m_SkinnedPerPassCB->Map(graphicsContext, 256);
	memcpy(skinnedPerPassCB, &mMainPassCB, sizeof(mMainPassCB));

	void* skinnedCB = m_SkinnedCB->Map(graphicsContext, 256);
	memcpy(skinnedCB, &mSkinnedConstants, sizeof(mSkinnedConstants));

	void* skinnedlightCB = m_SkinnedLightCB->Map(graphicsContext, 256);
	memcpy(skinnedlightCB, &mLightConstants, sizeof(mLightConstants));

	DrawRenderItems(graphicsContext, m_SkinnedPassPSO.get(), m_SkinnedPerDrawCB.get(), mRitemLayer[(int)RenderLayer::SkinnedOpaque]);

	// <------------------------------------GUI Pass---------------------------------------------->
	UpdateGUI();

	/*ImGuiIO& io = ImGui::GetIO(); (void)io;
	io.DisplaySize = ImVec2((float)D3DApp::mClientWidth, (float)D3DApp::mClientHeight);*/

	ImGui::Render();
	graphicsContext.SetDescriptorHeap(cbvsrvuavHeap, samplerHeap);
	ImGui_ImplDX12_RenderDrawData(ImGui::GetDrawData(), graphicsContext.GetD3DCommandList());

	graphicsContext.Finish();
	m_SwapChain->Present();


}

void Engine::OnMouseDown(WPARAM btnState, int x, int y)
{
    mLastMousePos.x = x;
    mLastMousePos.y = y;

    SetCapture(mhMainWnd);
}

void Engine::OnMouseUp(WPARAM btnState, int x, int y)
{
    ReleaseCapture();
}

void Engine::OnMouseMove(WPARAM btnState, int x, int y)
{
    if((btnState & MK_RBUTTON) != 0)
    {
		// Make each pixel correspond to a quarter of a degree.
		float dx = XMConvertToRadians(0.25f*static_cast<float>(x - mLastMousePos.x));
		float dy = XMConvertToRadians(0.25f*static_cast<float>(y - mLastMousePos.y));

		mCamera.Pitch(dy);
		mCamera.RotateY(dx);
    }

    mLastMousePos.x = x;
    mLastMousePos.y = y;
}
 
void Engine::OnKeyboardInput(const GameTimer& gt)
{
	const float dt = gt.DeltaTime();

	// Camera Control
	if(GetAsyncKeyState('W') & 0x8000)
		mCamera.Walk(10.0f*dt);

	if(GetAsyncKeyState('S') & 0x8000)
		mCamera.Walk(-10.0f*dt);

	if(GetAsyncKeyState('A') & 0x8000)
		mCamera.Strafe(-10.0f*dt);

	if(GetAsyncKeyState('D') & 0x8000)
		mCamera.Strafe(10.0f*dt);


	// Character Control
	//if (GetAsyncKeyState(VK_UP) & 0x8000)
	//	//character.character_controller_.Update(Vector3(0.0f, 0.0f, -1.0f), gt.DeltaTime());
	//	character.character_controller_.gamepadstick_left = Vector3(0.0f, 0.0f, 1.0f);
	//else if (GetAsyncKeyState(VK_DOWN) & 0x8000)
	//	//character.character_controller_.Update(Vector3(0.0f, 0.0f, 1.0f), gt.DeltaTime());
	//	character.character_controller_.gamepadstick_left = Vector3(0.0f, 0.0f, -1.0f);
	//else if (GetAsyncKeyState(VK_LEFT) & 0x8000)
	//	//character.character_controller_.Update(Vector3(-1.0f, 0.0f, 0.0f), gt.DeltaTime());
	//	character.character_controller_.gamepadstick_left = Vector3(-1.0f, 0.0f, 0.0f);
	//else if (GetAsyncKeyState(VK_RIGHT) & 0x8000)
	//	//character.character_controller_.Update(Vector3(1.0f, 0.0f, 0.0f), gt.DeltaTime());
	//	character.character_controller_.gamepadstick_left = Vector3(1.0f, 0.0f, 0.0f);
	//else
	//	//character.character_controller_.Update(Vector3(0.0f, 0.0f, 0.0f), gt.DeltaTime());
	//	character.character_controller_.gamepadstick_left = Vector3(0.0f, 0.0f, 0.0f);

	// TODO
	if (GetAsyncKeyState(' ') & 0x8000)
	{
		if (!mAppPaused)
		{
			mAppPaused = true;
			mTimer.Stop();
		}
		else
		{
			mAppPaused = false;
			mTimer.Start();
		}
	}
	mCamera.UpdateViewMatrix();
}
 
void Engine::UpdateSkinnedCBs(void* perPassCB, const GameTimer& gt)
{   
    // We only have one skinned model being animated.
	
	mController.Update(blender.GetDuration(), gt.DeltaTime());

	//blender.t = mController.GetTimeRatio();
	blender.deltaT = mController.GetDeltaTime();
	character.UpdateController(gt.DeltaTime());
	//character.UpdateBlendingMotion(blender);
	character.UpdateFinalModelTransform();

    std::copy(
        std::begin(character.models),
        std::end(character.models),
        &mSkinnedConstants.BoneTransforms[0]);

	for (int i = 0; i < mRitemLayer[(int)RenderLayer::SkinnedOpaque].size(); i++) {
		XMMATRIX m = XMMatrixScaling(0.05f, 0.05f, 0.05f);
		m *= XMMatrixRotationAxis(Vector3(0, 1, 0), MathHelper::Pi);// TODO
		m *= XMMatrixRotationQuaternion(character.character_controller.simulation_rotation);
		m *= XMMatrixTranslationFromVector(character.character_controller.simulation_position * 0.05f);
		XMStoreFloat4x4(&mRitemLayer[(int)RenderLayer::SkinnedOpaque][i]->World, m);
	}

}

void Engine::UpdateShadowTransform(const GameTimer& gt)
{
	// Only the first "main" light casts a shadow.
	XMVECTOR lightDir = XMLoadFloat3(&mRotatedLightDirections[0]);
	XMVECTOR lightPos = -2.0f * mSceneBounds.Radius * lightDir;
	XMVECTOR targetPos = XMLoadFloat3(&mSceneBounds.Center);
	XMVECTOR lightUp = XMVectorSet(0.0f, 1.0f, 0.0f, 0.0f);
	XMMATRIX lightView = XMMatrixLookAtLH(lightPos, targetPos, lightUp);

	XMStoreFloat3(&mLightPosW, lightPos);

	// Transform bounding sphere to light space.
	XMFLOAT3 sphereCenterLS;
	XMStoreFloat3(&sphereCenterLS, XMVector3TransformCoord(targetPos, lightView));

	// Ortho frustum in light space encloses scene.
	float l = sphereCenterLS.x - mSceneBounds.Radius;
	float b = sphereCenterLS.y - mSceneBounds.Radius;
	float n = sphereCenterLS.z - mSceneBounds.Radius;
	float r = sphereCenterLS.x + mSceneBounds.Radius;
	float t = sphereCenterLS.y + mSceneBounds.Radius;
	float f = sphereCenterLS.z + mSceneBounds.Radius;

	mLightNearZ = n;
	mLightFarZ = f;
	XMMATRIX lightProj = XMMatrixOrthographicOffCenterLH(l, r, b, t, n, f);

	// Transform NDC space [-1,+1]^2 to texture space [0,1]^2
	XMMATRIX T(
		0.5f, 0.0f, 0.0f, 0.0f,
		0.0f, -0.5f, 0.0f, 0.0f,
		0.0f, 0.0f, 1.0f, 0.0f,
		0.5f, 0.5f, 0.0f, 1.0f);

	XMMATRIX S = lightView * lightProj * T;
	XMStoreFloat4x4(&mLightView, lightView);
	XMStoreFloat4x4(&mLightProj, lightProj);
	XMStoreFloat4x4(&mShadowTransform, S);
}

void Engine::UpdateShadowPerPassCB(const GameTimer& gt)
{
	XMMATRIX view = XMLoadFloat4x4(&mLightView);
	XMMATRIX proj = XMLoadFloat4x4(&mLightProj);

	XMMATRIX viewProj = XMMatrixMultiply(view, proj);
	XMMATRIX invView = XMMatrixInverse(&XMMatrixDeterminant(view), view);
	XMMATRIX invProj = XMMatrixInverse(&XMMatrixDeterminant(proj), proj);
	XMMATRIX invViewProj = XMMatrixInverse(&XMMatrixDeterminant(viewProj), viewProj);

	UINT w = 4096;
	UINT h = 4096;

	XMStoreFloat4x4(&mShadowPassCB.View, XMMatrixTranspose(view));
	XMStoreFloat4x4(&mShadowPassCB.InvView, XMMatrixTranspose(invView));
	XMStoreFloat4x4(&mShadowPassCB.Proj, XMMatrixTranspose(proj));
	XMStoreFloat4x4(&mShadowPassCB.InvProj, XMMatrixTranspose(invProj));
	XMStoreFloat4x4(&mShadowPassCB.ViewProj, XMMatrixTranspose(viewProj));
	XMStoreFloat4x4(&mShadowPassCB.InvViewProj, XMMatrixTranspose(invViewProj));
	mShadowPassCB.EyePosW = mLightPosW;
	mShadowPassCB.RenderTargetSize = XMFLOAT2((float)w, (float)h);
	mShadowPassCB.InvRenderTargetSize = XMFLOAT2(1.0f / w, 1.0f / h);
	mShadowPassCB.NearZ = mLightNearZ;
	mShadowPassCB.FarZ = mLightFarZ;
	//memcpy(shadowPerPassCB, &mShadowPassCB, sizeof(PassConstants));
	//auto currPassCB = mCurrFrameResource->PassCB.get();
	//currPassCB->CopyData(1, mShadowPassCB);
}

void Engine::UpdateMainPassCB(void* perPassCB, const GameTimer& gt)
{
	XMMATRIX view = mCamera.GetView();
	XMMATRIX proj = mCamera.GetProj();

	XMMATRIX viewProj = XMMatrixMultiply(view, proj);
	XMMATRIX invView = XMMatrixInverse(&XMMatrixDeterminant(view), view);
	XMMATRIX invProj = XMMatrixInverse(&XMMatrixDeterminant(proj), proj);
	XMMATRIX invViewProj = XMMatrixInverse(&XMMatrixDeterminant(viewProj), viewProj);

    // Transform NDC space [-1,+1]^2 to texture space [0,1]^2
    XMMATRIX T(
        0.5f, 0.0f, 0.0f, 0.0f,
        0.0f, -0.5f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f,
        0.5f, 0.5f, 0.0f, 1.0f);

    XMMATRIX viewProjTex = XMMatrixMultiply(viewProj, T);
    XMMATRIX shadowTransform = XMLoadFloat4x4(&mShadowTransform);

	XMStoreFloat4x4(&mMainPassCB.View, XMMatrixTranspose(view));
	XMStoreFloat4x4(&mMainPassCB.InvView, XMMatrixTranspose(invView));
	XMStoreFloat4x4(&mMainPassCB.Proj, XMMatrixTranspose(proj));
	XMStoreFloat4x4(&mMainPassCB.InvProj, XMMatrixTranspose(invProj));
	XMStoreFloat4x4(&mMainPassCB.ViewProj, XMMatrixTranspose(viewProj));
	XMStoreFloat4x4(&mMainPassCB.InvViewProj, XMMatrixTranspose(invViewProj));
    XMStoreFloat4x4(&mMainPassCB.ViewProjTex, XMMatrixTranspose(viewProjTex));
    XMStoreFloat4x4(&mMainPassCB.ShadowTransform, XMMatrixTranspose(shadowTransform));
	mMainPassCB.EyePosW = mCamera.GetPosition3f();
	mMainPassCB.RenderTargetSize = XMFLOAT2((float)mClientWidth, (float)mClientHeight);
	mMainPassCB.InvRenderTargetSize = XMFLOAT2(1.0f / mClientWidth, 1.0f / mClientHeight);
	mMainPassCB.NearZ = 1.0f;
	mMainPassCB.FarZ = 1000.0f;
	mMainPassCB.TotalTime = gt.TotalTime();
	mMainPassCB.DeltaTime = gt.DeltaTime();
	mLightConstants.AmbientLight = { 0.25f, 0.25f, 0.25f, 1.0f };
	mLightConstants.Lights[0].Direction = mRotatedLightDirections[0];
	mLightConstants.Lights[0].Strength = { 0.9f, 0.9f, 0.9f };
	mLightConstants.Lights[1].Direction = mRotatedLightDirections[1];
	mLightConstants.Lights[1].Strength = { 0.4f, 0.4f, 0.4f };
	mLightConstants.Lights[2].Direction = mRotatedLightDirections[2];
	mLightConstants.Lights[2].Strength = { 0.2f, 0.2f, 0.2f };

}

void Engine::UpdateGUI()
{
	ImGui_ImplDX12_NewFrame();
	ImGui_ImplWin32_NewFrame();
	ImGui::NewFrame();

	if (ImGui::Begin("Panel"))
	{
		character.db.OnGui();
		mController.OnGui();
		//blender.OnGui();
		float t[3] = { target.x, target.y, target.z };
		if (ImGui::InputFloat3("Target", t))
		{
			target = Vector3(t);
			XMStoreFloat4x4(&pTargetRitem->World, XMMatrixTranslation(target.x, target.y, target.z));
		}
		ImGui::SliderFloat("Weight", &character.foot_ik_weight, 0.0f, 1.0f);
		ImGui::SliderFloat("Soften", &character.foot_ik_soften, 0.0f, 1.0f);

		//ImGui::Image((ImTextureID)m_ShadowMapSRV->GetGpuHandle().ptr, ImVec2(1280, 720));

		ImVec2 canvas_p0 = ImGui::GetCursorScreenPos();      // ImDrawList API uses screen coordinates!
		float radius = 100.0f;
		ImVec2 canvas_sz = ImVec2(2.0f * radius, 2.0f * radius);   // Resize canvas to what's available
		ImVec2 canvas_p1 = ImVec2(canvas_p0.x + canvas_sz.x, canvas_p0.y + canvas_sz.y);

		// Draw gamepad UI
		static ImVec2 scrolling(0.0f, 0.0f);
		static ImVec2 gamepadEndPoint(0.0f, 0.0f);

		ImGuiIO& io = ImGui::GetIO();
		ImDrawList* draw_list = ImGui::GetWindowDrawList();
		const ImVec2 origin((canvas_p0.x + canvas_p1.x) * 0.5f, (canvas_p0.y + canvas_p1.y) * 0.5f); // Lock scrolled origin
		draw_list->AddCircleFilled(origin, radius, IM_COL32(50, 50, 50, 255));
		draw_list->AddCircle(origin, radius, IM_COL32(255, 255, 255, 255));
		//draw_list->AddRectFilled(canvas_p0, canvas_p1, IM_COL32(50, 50, 50, 255));
		//draw_list->AddRect(canvas_p0, canvas_p1, IM_COL32(255, 255, 255, 255));

		ImGui::InvisibleButton("canvas", canvas_sz, ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight);
		const bool is_hovered = ImGui::IsItemHovered(); // Hovered
		const bool is_active = ImGui::IsItemActive();   // Held
		//const ImVec2 origin_in_canvas((canvas_p0.x + canvas_p1.x) * 0.5f - origin.x, (canvas_p0.y + canvas_p1.y) * 0.5f - origin.y); // Lock scrolled origin
		const ImVec2 mouse_pos_in_canvas(io.MousePos.x - origin.x, io.MousePos.y - origin.y);
		const float length = sqrtf(mouse_pos_in_canvas.x * mouse_pos_in_canvas.x + mouse_pos_in_canvas.y * mouse_pos_in_canvas.y);
		if (is_hovered)
		{
			gamepadEndPoint.x = mouse_pos_in_canvas.x / radius;
			gamepadEndPoint.y = mouse_pos_in_canvas.y / radius;
		}
		else
		{
			gamepadEndPoint.x = 0.0f;
			gamepadEndPoint.y = 0.0f;
		}
		draw_list->AddLine(ImVec2(origin.x, origin.y), ImVec2(origin.x + 0.8f * radius * gamepadEndPoint.x, origin.y + 0.8f * radius * gamepadEndPoint.y), IM_COL32(255, 255, 0, 255), 2.0f);

		character.character_controller.gamepadstick_left = Vector3(gamepadEndPoint.x, 0.0f, -gamepadEndPoint.y);

		ImGui::End();
	}

	//ImGui::End();
}

void Engine::UpdateTarget(const GameTimer& gt)
{
	//float time = gt.TotalTime();
	//Vector3 animated_target(std::sin(time * 0.5f),
	//	std::cos(time * 0.25f) * 0.5,
	//	std::cos(time * 0.5f + .5f));
	//head_target = head_target_offset + animated_target * 5.0f;
	//XMStoreFloat4x4(&pHeadAimTargetRitem->World, XMMatrixTranslation(5, 10, 10));

	for (int i = 0; i < 4; i++) {
		XMStoreFloat4x4(&pTrajectoryPointRitems[i]->World, XMMatrixTranslation(
			character.character_controller.trajectory_positions[i].x * 0.05f,
			0.0f,
			character.character_controller.trajectory_positions[i].z * 0.05f));
	}

	for (int i = 0; i < 4; i++) {
		XMStoreFloat4x4(&pMatchedPointRitems[i]->World, XMMatrixTranslation(
			character.character_controller.matched_positions[i].x * 0.05f,
			0.0f,
			character.character_controller.matched_positions[i].z * 0.05f));
	}
}


void Engine::BuildShapeGeometry()
{
    GeometryGenerator geoGen;
	GeometryGenerator::MeshData box = geoGen.CreateBox(1.0f, 1.0f, 1.0f, 3);
	GeometryGenerator::MeshData grid = geoGen.CreateGrid(1.0f, 1.0f, 10, 10);
	GeometryGenerator::MeshData sphere = geoGen.CreateSphere(0.5f, 20, 20);
	GeometryGenerator::MeshData cylinder = geoGen.CreateCylinder(0.5f, 0.3f, 3.0f, 20, 20);
    GeometryGenerator::MeshData quad = geoGen.CreateQuad(0.0f, 0.0f, 1.0f, 1.0f, 0.0f);
    
	//
	// We are concatenating all the geometry into one big vertex/index buffer.  So
	// define the regions in the buffer each submesh covers.
	//

	// Cache the vertex offsets to each object in the concatenated vertex buffer.
	UINT boxVertexOffset = 0;
	UINT gridVertexOffset = (UINT)box.Vertices.size();
	UINT sphereVertexOffset = gridVertexOffset + (UINT)grid.Vertices.size();
	UINT cylinderVertexOffset = sphereVertexOffset + (UINT)sphere.Vertices.size();
    UINT quadVertexOffset = cylinderVertexOffset + (UINT)cylinder.Vertices.size();

	// Cache the starting index for each object in the concatenated index buffer.
	UINT boxIndexOffset = 0;
	UINT gridIndexOffset = (UINT)box.Indices32.size();
	UINT sphereIndexOffset = gridIndexOffset + (UINT)grid.Indices32.size();
	UINT cylinderIndexOffset = sphereIndexOffset + (UINT)sphere.Indices32.size();
    UINT quadIndexOffset = cylinderIndexOffset + (UINT)cylinder.Indices32.size();

	SubmeshGeometry boxSubmesh;
	boxSubmesh.IndexCount = (UINT)box.Indices32.size();
	boxSubmesh.StartIndexLocation = boxIndexOffset;
	boxSubmesh.BaseVertexLocation = boxVertexOffset;

	SubmeshGeometry gridSubmesh;
	gridSubmesh.IndexCount = (UINT)grid.Indices32.size();
	gridSubmesh.StartIndexLocation = gridIndexOffset;
	gridSubmesh.BaseVertexLocation = gridVertexOffset;

	SubmeshGeometry sphereSubmesh;
	sphereSubmesh.IndexCount = (UINT)sphere.Indices32.size();
	sphereSubmesh.StartIndexLocation = sphereIndexOffset;
	sphereSubmesh.BaseVertexLocation = sphereVertexOffset;

	SubmeshGeometry cylinderSubmesh;
	cylinderSubmesh.IndexCount = (UINT)cylinder.Indices32.size();
	cylinderSubmesh.StartIndexLocation = cylinderIndexOffset;
	cylinderSubmesh.BaseVertexLocation = cylinderVertexOffset;

    SubmeshGeometry quadSubmesh;
    quadSubmesh.IndexCount = (UINT)quad.Indices32.size();
    quadSubmesh.StartIndexLocation = quadIndexOffset;
    quadSubmesh.BaseVertexLocation = quadVertexOffset;

	//
	// Extract the vertex elements we are interested in and pack the
	// vertices of all the meshes into one vertex buffer.
	//

	auto totalVertexCount =
		box.Vertices.size() +
		grid.Vertices.size() +
		sphere.Vertices.size() +
		cylinder.Vertices.size() + 
        quad.Vertices.size();

	std::vector<Vertex> vertices(totalVertexCount);

	UINT k = 0;
	for(size_t i = 0; i < box.Vertices.size(); ++i, ++k)
	{
		vertices[k].Pos = box.Vertices[i].Position;
		vertices[k].Normal = box.Vertices[i].Normal;
		vertices[k].TexC = box.Vertices[i].TexC;
		vertices[k].TangentU = box.Vertices[i].TangentU;
	}

	for(size_t i = 0; i < grid.Vertices.size(); ++i, ++k)
	{
		vertices[k].Pos = grid.Vertices[i].Position;
		vertices[k].Normal = grid.Vertices[i].Normal;
		vertices[k].TexC = grid.Vertices[i].TexC;
		vertices[k].TangentU = grid.Vertices[i].TangentU;
	}

	for(size_t i = 0; i < sphere.Vertices.size(); ++i, ++k)
	{
		vertices[k].Pos = sphere.Vertices[i].Position;
		vertices[k].Normal = sphere.Vertices[i].Normal;
		vertices[k].TexC = sphere.Vertices[i].TexC;
		vertices[k].TangentU = sphere.Vertices[i].TangentU;
	}

	for(size_t i = 0; i < cylinder.Vertices.size(); ++i, ++k)
	{
		vertices[k].Pos = cylinder.Vertices[i].Position;
		vertices[k].Normal = cylinder.Vertices[i].Normal;
		vertices[k].TexC = cylinder.Vertices[i].TexC;
		vertices[k].TangentU = cylinder.Vertices[i].TangentU;
	}

    for(int i = 0; i < quad.Vertices.size(); ++i, ++k)
    {
        vertices[k].Pos = quad.Vertices[i].Position;
        vertices[k].Normal = quad.Vertices[i].Normal;
        vertices[k].TexC = quad.Vertices[i].TexC;
        vertices[k].TangentU = quad.Vertices[i].TangentU;
    }

	std::vector<std::uint16_t> indices;
	indices.insert(indices.end(), std::begin(box.GetIndices16()), std::end(box.GetIndices16()));
	indices.insert(indices.end(), std::begin(grid.GetIndices16()), std::end(grid.GetIndices16()));
	indices.insert(indices.end(), std::begin(sphere.GetIndices16()), std::end(sphere.GetIndices16()));
	indices.insert(indices.end(), std::begin(cylinder.GetIndices16()), std::end(cylinder.GetIndices16()));
    indices.insert(indices.end(), std::begin(quad.GetIndices16()), std::end(quad.GetIndices16()));

    const UINT vbByteSize = (UINT)vertices.size() * sizeof(Vertex);
    const UINT ibByteSize = (UINT)indices.size()  * sizeof(std::uint16_t);

	auto geo = std::make_unique<MeshGeometry>();
	geo->Name = "shapeGeo";

	ThrowIfFailed(D3DCreateBlob(vbByteSize, &geo->VertexBufferCPU));
	CopyMemory(geo->VertexBufferCPU->GetBufferPointer(), vertices.data(), vbByteSize);

	ThrowIfFailed(D3DCreateBlob(ibByteSize, &geo->IndexBufferCPU));
	CopyMemory(geo->IndexBufferCPU->GetBufferPointer(), indices.data(), ibByteSize);

	std::shared_ptr<Graphics::GpuDefaultBuffer> m_VertexBuffer = std::make_shared<Graphics::GpuDefaultBuffer>((UINT)vertices.size(), sizeof(Vertex), (void*)vertices.data());

	std::shared_ptr<Graphics::GpuDefaultBuffer> m_IndexBuffer = std::make_shared<Graphics::GpuDefaultBuffer>((UINT)indices.size(), sizeof(std::uint16_t), (void*)indices.data());

	geo->VertexBufferGPU = m_VertexBuffer->GetResource();

	geo->IndexBufferGPU = m_IndexBuffer->GetResource();

	geo->VertexByteStride = sizeof(Vertex);
	geo->VertexBufferByteSize = vbByteSize;
	geo->IndexFormat = DXGI_FORMAT_R16_UINT;
	geo->IndexBufferByteSize = ibByteSize;

	geo->DrawArgs["box"] = boxSubmesh;
	geo->DrawArgs["grid"] = gridSubmesh;
	geo->DrawArgs["sphere"] = sphereSubmesh;
	geo->DrawArgs["cylinder"] = cylinderSubmesh;
    geo->DrawArgs["quad"] = quadSubmesh;

	mGeometries[geo->Name] = std::move(geo);
}

void Engine::LoadSkinnedModel()
{
	std::vector<Animation::SkinnedVertex> vertices;
	std::vector<std::uint16_t> indices;	
	int animation_num = Animation_Num;


	FBXLoader fbxLoader;
	fbxLoader.LoadFBX(bind_pose_filename, vertices, indices,
		mSkinnedSubsets, mSkinnedMats, character.db);
	
	for (int i = 0; i < animation_num; ++i) {
		fbxLoader.LoadFBXClip(mAnimationFilename[i], character.db);
	}

	character.db.convert_to_fps(60.0f);

	for (int i = 0; i < animation_num; ++i)
	{
		std::string clipName = character.db.GetAnimationClipName(i);
		mSamplers[i] = character.db.GetAnimationClipByName(clipName);
		Debug::Log(LOG_LEVEL::LOG_LEVEL_INFO, "Analyze", "MotionAnalyzer", 794, clipName);
	}

	character.Initialize();

	// Initialize analyzer
	for (int i = 0; i < animation_num; ++i)
	{
		analyzer[i].legC = &character.leg_controller;
		analyzer[i].animation = mSamplers[i];
		analyzer[i].Analyze();
	}

	// Prepare gradient band interpolator
	std::vector<Vector2> velocities;
	for (size_t i = 0; i < animation_num; ++i)
	{
		Vector3 velocity = analyzer[i].mCycleSpeed * analyzer[i].mCycleDirection;
		if (i == 0) velocity = Vector3::Zero;
		velocities.push_back(Vector2(velocity.x, velocity.z));
	}
	interpolator = std::make_shared<PolarGradientBandInterpolator>(velocities);
	character.character_controller.simulation_run_fwrd_speed = -interpolator->minVy;
	character.character_controller.simulation_run_side_speed = interpolator->maxVx;
	character.character_controller.simulation_run_back_speed = interpolator->maxVy;
	//character.character_controller_.simulation_run_fwrd_speed = 4.0f;
	//character.character_controller_.simulation_run_side_speed = 3.0f;
	//character.character_controller_.simulation_run_back_speed = 2.5f;

	// Calculate weight
	std::vector<float> weights = interpolator->Interpolate(Vector2(0.0, 0.0));
	//std::vector<float> weights = interpolator->Interpolate(Vector2(velocities[0].x, velocities[0].y));

	// Prepares blending layers
	std::vector<BlendingJob::Layer> layers;
	for (size_t i = 0; i < animation_num; i++)
	{
		BlendingJob::Layer layer;
		layer.weight = weights[i];
		layer.t = 0.0f;
		layer.animation = analyzer[i].animation;
		layer.Nk = 4;
		layer.K[0] = 0.0f * analyzer[i].animation->get_duration_in_second();
		layer.K[1] = analyzer[i].cycles[0].postliftTime * analyzer[i].animation->get_duration_in_second();
		layer.K[2] = analyzer[i].cycles[1].postliftTime * analyzer[i].animation->get_duration_in_second();
		layer.K[3] = 1.0f * analyzer[i].animation->get_duration_in_second();
		qsort(layer.K,
			layer.Nk,
			sizeof(float),
			[](const void* x, const void* y) {
				return *(int*)x - *(int*)y;
			});
		layers.push_back(layer);
	}

	// Prepares playback controller
	mController.SetTimeRatio(0.0f);

	// Setups blending job
	blender.layers = layers;
	blender.interpolator = interpolator;

	const UINT vbByteSize = static_cast<UINT>(vertices.size()) * sizeof(Animation::SkinnedVertex);
    const UINT ibByteSize = static_cast<UINT>(indices.size()) * sizeof(std::uint16_t);

	auto geo = std::make_unique<MeshGeometry>();
	geo->Name = mSkeletonName;

	ThrowIfFailed(D3DCreateBlob(vbByteSize, &geo->VertexBufferCPU));
	CopyMemory(geo->VertexBufferCPU->GetBufferPointer(), vertices.data(), vbByteSize);

	ThrowIfFailed(D3DCreateBlob(ibByteSize, &geo->IndexBufferCPU));
	CopyMemory(geo->IndexBufferCPU->GetBufferPointer(), indices.data(), ibByteSize);

	std::shared_ptr<Graphics::GpuDefaultBuffer> m_VertexBuffer = std::make_shared<Graphics::GpuDefaultBuffer>((UINT)vertices.size(), sizeof(Animation::SkinnedVertex), (void*)vertices.data());

	std::shared_ptr<Graphics::GpuDefaultBuffer> m_IndexBuffer = std::make_shared<Graphics::GpuDefaultBuffer>((UINT)indices.size(), sizeof(std::uint16_t), (void*)indices.data());

	geo->VertexBufferGPU = m_VertexBuffer->GetResource();

	geo->IndexBufferGPU = m_IndexBuffer->GetResource();

	geo->VertexByteStride = sizeof(Animation::SkinnedVertex);
	geo->VertexBufferByteSize = vbByteSize;
	geo->IndexFormat = DXGI_FORMAT_R16_UINT;
	geo->IndexBufferByteSize = ibByteSize;

	for(UINT i = 0; i < (UINT)mSkinnedSubsets.size(); ++i)
	{
		SubmeshGeometry submesh;
		std::string name = "sm_" + std::to_string(i);
		submesh.IndexCount = (UINT)mSkinnedSubsets[i].FaceCount * 3;
        submesh.StartIndexLocation = mSkinnedSubsets[i].FaceStart * 3;
        submesh.BaseVertexLocation = 0;

		geo->DrawArgs[name] = submesh;
	}

	mGeometries[geo->Name] = std::move(geo);
}

void Engine::BuildMaterials()
{
	TexMetadata metadata;
	ScratchImage image;
	HRESULT hr = LoadFromWICFile(L"Contents/Textures/GreyWhiteBlock.jpg", WIC_FLAGS::WIC_FLAGS_NONE, &metadata, image);
	const Image* pImages = image.GetImages();
	std::shared_ptr<Graphics::GpuTexture2D> tileTex = std::make_shared<Graphics::GpuTexture2D>(pImages[0].width, pImages[0].height, pImages[0].format, pImages[0].rowPitch, pImages[0].pixels);


	DirectX::XMFLOAT4 oneVector = DirectX::XMFLOAT4(1.0f, 1.0f, 1.0f, 1.0f);
	auto tile0 = std::make_unique<Material>(0.5f, 0.5f, oneVector, oneVector, tileTex,
		nullptr, nullptr, nullptr, nullptr);
	mMaterials["tile0"] = std::move(tile0);

	for (UINT i = 0; i < mSkinnedMats.size(); ++i)
	{
		auto mat = std::make_unique<Material>(0.5f, mSkinnedMats[i].Roughness, mSkinnedMats[i].DiffuseAlbedo, oneVector, ResourceManager::GetSingleton().GetDefaultWhiteTex(),
			nullptr, nullptr, nullptr, nullptr);

		mMaterials[mSkinnedMats[i].Name] = std::move(mat);
	}

	//mMaterialConstants[0] = PBRMaterialConstants(Vector4(1.0, 0.0, 0.0f, 1.0f), Vector4(1.0, 0.0, 0.0f, 1.0f), 1.0f, 1.0f);
	//mMaterialConstants[1] = PBRMaterialConstants(Vector4(0.0, 1.0, 0.0f, 1.0f), Vector4(1.0, 0.0, 0.0f, 1.0f), 1.0f, 1.0f);
	//mMaterialConstants[2] = PBRMaterialConstants(Vector4(1.0, 1.0, 1.0f, 1.0f), Vector4(1.0, 0.0, 0.0f, 1.0f), 1.0f, 1.0f);

}

void Engine::BuildRenderItems()
{
	UINT objCBIndex = 0;

    auto gridRitem = std::make_unique<RenderItem>();
    gridRitem->World = MathHelper::Identity4x4();
	XMStoreFloat4x4(&gridRitem->World, XMMatrixScaling(100.0f, 1.0f, 100.0f) * XMMatrixTranslation(0.0, 0.0, 0.0));
	XMStoreFloat4x4(&gridRitem->TexTransform, XMMatrixScaling(1.0f, 1.0f, 1.0f));
	gridRitem->ObjCBIndex = objCBIndex++;
	gridRitem->Mat = mMaterials["tile0"].get();
	gridRitem->Geo = mGeometries["shapeGeo"].get();
	gridRitem->PrimitiveType = D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST;
    gridRitem->IndexCount = gridRitem->Geo->DrawArgs["grid"].IndexCount;
    gridRitem->StartIndexLocation = gridRitem->Geo->DrawArgs["grid"].StartIndexLocation;
    gridRitem->BaseVertexLocation = gridRitem->Geo->DrawArgs["grid"].BaseVertexLocation;
	gridRitem->Color = Colors::White;
	mRitemLayer[(int)RenderLayer::Checkboard].push_back(gridRitem.get());
	mAllRitems.push_back(std::move(gridRitem));

	//// Foot IK Target
	//auto targetRitem = std::make_unique<RenderItem>();
	//targetRitem->World = MathHelper::Identity4x4();
	//XMStoreFloat4x4(&targetRitem->World, XMMatrixTranslation(target.x, target.y, target.z));
	//XMStoreFloat4x4(&targetRitem->TexTransform, Matrix::Identity);
	//targetRitem->ObjCBIndex = objCBIndex++;
	//targetRitem->Mat = mMaterials["tile0"].get();
	//targetRitem->Geo = mGeometries["shapeGeo"].get();
	//targetRitem->PrimitiveType = D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST;
	//targetRitem->IndexCount = targetRitem->Geo->DrawArgs["sphere"].IndexCount;
	//targetRitem->StartIndexLocation = targetRitem->Geo->DrawArgs["sphere"].StartIndexLocation;
	//targetRitem->BaseVertexLocation = targetRitem->Geo->DrawArgs["sphere"].BaseVertexLocation;


	//mRitemLayer[(int)RenderLayer::Opaque].push_back(targetRitem.get());
	//pTargetRitem = targetRitem.get();
	//mAllRitems.push_back(std::move(targetRitem));

	//// Head IK Target
	//auto head_target_ritem = std::make_unique<RenderItem>();
	//head_target_ritem->World = MathHelper::Identity4x4();
	//XMStoreFloat4x4(&head_target_ritem->World, XMMatrixTranslation(head_target.x, head_target.y, head_target.z));
	//XMStoreFloat4x4(&head_target_ritem->TexTransform, Matrix::Identity);
	//head_target_ritem->ObjCBIndex = objCBIndex++;
	//head_target_ritem->Mat = mMaterials["tile0"].get();
	//head_target_ritem->Geo = mGeometries["shapeGeo"].get();
	//head_target_ritem->PrimitiveType = D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST;
	//head_target_ritem->IndexCount = head_target_ritem->Geo->DrawArgs["sphere"].IndexCount;
	//head_target_ritem->StartIndexLocation = head_target_ritem->Geo->DrawArgs["sphere"].StartIndexLocation;
	//head_target_ritem->BaseVertexLocation = head_target_ritem->Geo->DrawArgs["sphere"].BaseVertexLocation;

	//mRitemLayer[(int)RenderLayer::Opaque].push_back(head_target_ritem.get());
	//pHeadAimTargetRitem = head_target_ritem.get();
	//mAllRitems.push_back(std::move(head_target_ritem));
	
	// Trajectory
	for (int i = 0; i < 4; i++) {
		auto trajectoryPointRitem = std::make_unique<RenderItem>();
		trajectoryPointRitem->World = MathHelper::Identity4x4();
		XMStoreFloat4x4(&trajectoryPointRitem->World, XMMatrixTranslation(0.0f, 0.0f, 0.0f));
		XMStoreFloat4x4(&trajectoryPointRitem->TexTransform, Matrix::Identity);
		trajectoryPointRitem->ObjCBIndex = objCBIndex++;
		trajectoryPointRitem->Mat = mMaterials["tile0"].get();
		trajectoryPointRitem->Geo = mGeometries["shapeGeo"].get();
		trajectoryPointRitem->PrimitiveType = D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST;
		trajectoryPointRitem->IndexCount = trajectoryPointRitem->Geo->DrawArgs["sphere"].IndexCount;
		trajectoryPointRitem->StartIndexLocation = trajectoryPointRitem->Geo->DrawArgs["sphere"].StartIndexLocation;
		trajectoryPointRitem->BaseVertexLocation = trajectoryPointRitem->Geo->DrawArgs["sphere"].BaseVertexLocation;
		trajectoryPointRitem->Color = Colors::Yellow;

		mRitemLayer[(int)RenderLayer::Opaque].push_back(trajectoryPointRitem.get());
		pTrajectoryPointRitems[i] = trajectoryPointRitem.get();
		mAllRitems.push_back(std::move(trajectoryPointRitem));

	}

	// Matched
	for (int i = 0; i < 4; i++) {
		auto matchedPointRitem = std::make_unique<RenderItem>();
		matchedPointRitem->World = MathHelper::Identity4x4();
		XMStoreFloat4x4(&matchedPointRitem->World, XMMatrixTranslation(0.0f, 0.0f, 0.0f));
		XMStoreFloat4x4(&matchedPointRitem->TexTransform, Matrix::Identity);
		matchedPointRitem->ObjCBIndex = objCBIndex++;
		matchedPointRitem->Mat = mMaterials["tile0"].get();
		matchedPointRitem->Geo = mGeometries["shapeGeo"].get();
		matchedPointRitem->PrimitiveType = D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST;
		matchedPointRitem->IndexCount = matchedPointRitem->Geo->DrawArgs["sphere"].IndexCount;
		matchedPointRitem->StartIndexLocation = matchedPointRitem->Geo->DrawArgs["sphere"].StartIndexLocation;
		matchedPointRitem->BaseVertexLocation = matchedPointRitem->Geo->DrawArgs["sphere"].BaseVertexLocation;
		matchedPointRitem->Color = Colors::LightGreen;

		mRitemLayer[(int)RenderLayer::Opaque].push_back(matchedPointRitem.get());
		pMatchedPointRitems[i] = matchedPointRitem.get();
		mAllRitems.push_back(std::move(matchedPointRitem));

	}

	if (mGeometries[mSkeletonName]!=NULL)
	{
		for (UINT i = 0; i < mGeometries[mSkeletonName]->DrawArgs.size(); ++i)
		{
			std::string submeshName = "sm_" + std::to_string(i);

			auto ritem = std::make_unique<RenderItem>();

			// Reflect to change coordinate system from the RHS the data was exported out as.
			XMStoreFloat4x4(&ritem->World, character.scale);

			ritem->TexTransform = MathHelper::Identity4x4();
			ritem->ObjCBIndex = objCBIndex++;
			ritem->Mat = mMaterials[mSkinnedMats[i].Name].get();
			ritem->Geo = mGeometries[mSkeletonName].get();
			ritem->PrimitiveType = D3D_PRIMITIVE_TOPOLOGY_TRIANGLELIST;
			ritem->IndexCount = ritem->Geo->DrawArgs[submeshName].IndexCount;
			ritem->StartIndexLocation = ritem->Geo->DrawArgs[submeshName].StartIndexLocation;
			ritem->BaseVertexLocation = ritem->Geo->DrawArgs[submeshName].BaseVertexLocation;
			ritem->Color = Colors::White;

			// All render items for this solider.m3d instance share
			// the same skinned model instance.
			ritem->SkinnedCBIndex = 0;
			ritem->SkinnedModelInst = mSamplers[0];

			mRitemLayer[(int)RenderLayer::SkinnedOpaque].push_back(ritem.get());
			mAllRitems.push_back(std::move(ritem));
		}
	}
}

void Engine::BuildGUI()
{
	// Setup Dear ImGui context
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO(); (void)io;
	//io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;           // Enable Docking
	//io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;         // Enable Multi-Viewport / Platform Windows
	//io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
	//io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

	// Setup Dear ImGui style
	ImGui::StyleColorsDark();
	//ImGui::StyleColorsClassic();

	// When viewports are enabled we tweak WindowRounding/WindowBg so platform windows can look identical to regular ones.
	//ImGuiStyle& style = ImGui::GetStyle();
	//if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
	//{
	//	//style.WindowRounding = 0.0f;
	//	//style.Colors[ImGuiCol_WindowBg].w = 1.0f;
	//}


	// Setup Platform/Renderer backends
	Graphics::GPUDescriptorHeap& cbvsrvuavHeap = Graphics::RenderDevice::GetSingleton().GetGPUDescriptorHeap(D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV);
	Graphics::DescriptorHeapAllocation allocation = cbvsrvuavHeap.Allocate(3);
	ImGui_ImplWin32_Init(mhMainWnd);
	ImGui_ImplDX12_Init(m_RenderDevice->GetD3D12Device(), 3,
		DXGI_FORMAT_R8G8B8A8_UNORM, cbvsrvuavHeap.GetD3D12DescriptorHeap(),
		allocation.GetCpuHandle(),
		allocation.GetGpuHandle());

}

void Engine::DrawRenderItems(Graphics::GraphicsContext& graphicsContext, Graphics::PipelineState* pipelineState, Graphics::GpuDynamicBuffer* perDrawCB, const std::vector<RenderItem*>& ritems, bool useMaterial)
{
	// For each render item...
	for (size_t i = 0; i < ritems.size(); ++i)
	{
		auto ri = ritems[i];

		if (useMaterial)
			ri->Mat->CreateSRB(pipelineState);

		graphicsContext.SetVertexBuffer(0, ri->Geo->VertexBufferView());
		graphicsContext.SetIndexBuffer(ri->Geo->IndexBufferView());
		graphicsContext.SetPrimitiveTopology(ri->PrimitiveType);
		if(useMaterial)
			graphicsContext.SetShaderResourceBinding(ri->Mat->GetSRB());

		ObjectConstants objConstants;
		XMMATRIX world = XMLoadFloat4x4(&ri->World);
		XMMATRIX object = XMMatrixInverse(nullptr, world);
		XMStoreFloat4x4(&objConstants.World, XMMatrixTranspose(world));
		XMStoreFloat4x4(&objConstants.Obejct, XMMatrixTranspose(object));
		XMStoreFloat4(&objConstants.Color, XMLoadFloat4(&ri->Color));
		void* pPerDrawCB = perDrawCB->Map(graphicsContext, 256);
		memcpy(pPerDrawCB, &objConstants, sizeof(ObjectConstants));

		graphicsContext.DrawIndexedInstanced(ri->IndexCount, 1, ri->StartIndexLocation, ri->BaseVertexLocation, 0);
	}
}