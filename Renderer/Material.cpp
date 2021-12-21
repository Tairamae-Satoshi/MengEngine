#include "../pch.h"
#include "Material.h"
#include "../Graphics/GpuTexture2D.h"
#include "../Graphics/GpuResourceDescriptor.h"
#include "../Graphics/ShaderResourceBinding.h"
#include "../Graphics/PipelineState.h"
#include "../Graphics/GpuBuffer.h"

Material::Material(float metallicFactor, float roughnessFactor, DirectX::XMFLOAT4 baseColorFactor, DirectX::XMFLOAT4 emissiveFactor,
	std::shared_ptr<Graphics::GpuTexture2D> baseColorTex, std::shared_ptr<Graphics::GpuTexture2D> metallicRoughnessTex,
	std::shared_ptr<Graphics::GpuTexture2D> normalTex, std::shared_ptr<Graphics::GpuTexture2D> occlusionTex, std::shared_ptr<Graphics::GpuTexture2D> emissiveTex) :
	m_ConstantsData{ baseColorFactor, emissiveFactor, metallicFactor, roughnessFactor },
	m_BaseColorTexture(baseColorTex),
	m_MetallicRoughnessTexture(metallicRoughnessTex),
	m_NormalTexture(normalTex),
	m_OcclusionTexture(occlusionTex),
	m_EmissiveTexture(emissiveTex)
{
}

void Material::CreateSRB(Graphics::PipelineState* PSO)
{
	assert(PSO != nullptr);

	//if (m_SRB == nullptr)
	{
		m_ConstantBuffer = std::make_shared<Graphics::GpuDefaultBuffer>(1, sizeof(PBRMaterialConstants), &m_ConstantsData);
		m_BaseColorTextureDescriptor = m_BaseColorTexture != nullptr ? m_BaseColorTexture->CreateSRV() : nullptr;
		m_MetallicRoughnessTextureDescriptor = m_MetallicRoughnessTexture != nullptr ? m_MetallicRoughnessTexture->CreateSRV() : nullptr;
		m_NormalTextureDescriptor = m_NormalTexture != nullptr ? m_NormalTexture->CreateSRV() : nullptr;
		m_OcclusionTextureDescriptor = m_OcclusionTexture != nullptr ? m_OcclusionTexture->CreateSRV() : nullptr;
		m_EmissiveTextureDescriptor = m_EmissiveTexture != nullptr ? m_EmissiveTexture->CreateSRV() : nullptr;

		m_SRB = PSO->CreateShaderResourceBinding();

		Graphics::ShaderVariable* materialCB = m_SRB->GetVariableByName(Graphics::SHADER_TYPE_PIXEL, "cbMaterial");
		if (materialCB != nullptr)
		{
			materialCB->Set(m_ConstantBuffer);
		}
		/*else
		{
			LOG_ERROR("Shader variable: cbMaterial not found.");
		}*/
		
		Graphics::ShaderVariable* baseColorTexVar = m_SRB->GetVariableByName(Graphics::SHADER_TYPE_PIXEL, "BaseColorTex");
		Graphics::ShaderVariable* metallicRoughnessTexVar = m_SRB->GetVariableByName(Graphics::SHADER_TYPE_PIXEL, "MetallicRoughnessTex");
		Graphics::ShaderVariable* normalTexVar = m_SRB->GetVariableByName(Graphics::SHADER_TYPE_PIXEL, "NormalTex");
		Graphics::ShaderVariable* occlusionTexVar = m_SRB->GetVariableByName(Graphics::SHADER_TYPE_PIXEL, "OcclusionTex");
		Graphics::ShaderVariable* emissiveTexVar = m_SRB->GetVariableByName(Graphics::SHADER_TYPE_PIXEL, "EmissiveTex");

		int i = 0;
		auto bindTex = [](Graphics::ShaderVariable* variable, std::shared_ptr<Graphics::GpuResourceDescriptor> texDescriptor)
		{
			if (variable != nullptr)
			{
				variable->Set(texDescriptor);
			}
			/*else
			{
				LOG_ERROR("Shader variable not found");
			}*/
		};

		if (m_BaseColorTextureDescriptor != nullptr)
			bindTex(baseColorTexVar, m_BaseColorTextureDescriptor);
		if (m_MetallicRoughnessTextureDescriptor != nullptr)
			bindTex(metallicRoughnessTexVar, m_MetallicRoughnessTextureDescriptor);
		if (m_NormalTextureDescriptor != nullptr)
			bindTex(normalTexVar, m_NormalTextureDescriptor);
		if (m_OcclusionTextureDescriptor != nullptr)
			bindTex(occlusionTexVar, m_OcclusionTextureDescriptor);
		if (m_EmissiveTextureDescriptor != nullptr)
			bindTex(emissiveTexVar, m_EmissiveTextureDescriptor);
	}
}