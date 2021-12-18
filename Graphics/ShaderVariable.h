#pragma once
#include "ShaderResourceLayout.h"
#include "ShaderResourceCache.h"
#include "GpuBuffer.h"
#include "GpuResourceDescriptor.h"
#include "ShaderResourceBindingUtility.h"

namespace Graphics 
{

    /**
   * ��ʾShader�е�һ���������ⲿ����ͨ�������������Դ(һ����������һ����ͼ)
   */
    class ShaderVariable
    {
		friend class ShaderVariableCollection;

    public:
        ShaderVariable(ShaderResourceCache* cache,
            const ShaderResourceLayout::Resource& resource) :
            m_ResourceCache(cache),
            m_Resource(resource)
        {
        }

        ShaderVariable(const ShaderVariable&) = delete;
        ShaderVariable(ShaderVariable&&) = delete;
        ShaderVariable& operator=(const ShaderVariable&) = delete;
        ShaderVariable& operator=(ShaderVariable&&) = delete;

        /*  ֻ��CBV������Buffer��SRV/UAV������ΪRoot Descriptor�󶨣���Ϊ����ID3D12GraphicsCommandList::SetGraphicsRootConstantBufferView()����ʱ��
        *   �ǲ�ָ��Buffer�Ĵ�С�ģ�Buffer�Ĵ�С��Shaderȷ������ΪTexture����Ҫ�ܶ���Ϣ�������ģ�����ֻ�ṩһ����ַ�ǲ�����
        *   Bufferֱ�Ӱ󶨵�Root Descriptorʱ��������������
        *   Ŀǰֻ��CBV����ΪRoot Descriptor�󶨵�
        *   һ������Ҳ��һ��ShaderVariable��ʾ���������а���Դʱ��Ҫ���������е�����
        */
        void Set(std::shared_ptr<GpuBuffer> buffer, UINT32 arrayIndex = 0);
        void Set(std::shared_ptr<GpuResourceDescriptor> view, UINT32 arrayIndex = 0);
        bool IsBound(UINT32 arrayIndex) const;

    private:
        ShaderResourceCache* m_ResourceCache;
        const ShaderResourceLayout::Resource& m_Resource;
    };

    /*
    * ShaderVariableCollection�����ض����͵�ShaderVariable�б���
    * PipelineStateʹ��Manager������Static��Դ��ShaderResourceBindingʹ��Manager������Mutable��Dynamic��Դ
    * ��ShaderResourceLayout��ShaderResourceCache����������������
    */
    class ShaderVariableCollection
    {
    public:
        // ΪShaderResourceLayout�е�ÿ��Shader��Դ����һ��ShaderVariable
        ShaderVariableCollection(ShaderResourceCache* resourceCache,
            const ShaderResourceLayout& srcLayout,
            const SHADER_RESOURCE_VARIABLE_TYPE* allowedVarTypes,
            UINT32 allowedTypeNum);
        
        ShaderVariableCollection(const ShaderVariableCollection&) = delete;
        ShaderVariableCollection(ShaderVariableCollection&&) = delete;
        ShaderVariableCollection& operator=(const ShaderVariableCollection&) = delete;
        ShaderVariableCollection& operator=(ShaderVariableCollection&&) = delete;

        ShaderVariable* GetVariable(const std::string& name);
        ShaderVariable* GetVariable(UINT32 index);
        UINT32 GetVariableCount() const { return m_Variables.size(); }


    private:
        friend ShaderVariable;

        ShaderResourceCache* m_ResourceCache;

        // ��ΪShaderVariable���ܿ��������ƶ������Ա�������ָ��
        std::vector<std::unique_ptr<ShaderVariable>> m_Variables;
    };

   
}