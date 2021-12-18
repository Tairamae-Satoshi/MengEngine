#pragma once

namespace Graphics
{
	class GpuTexture2D;
}

class ResourceManager : public Singleton<ResourceManager>
{
public:
	ResourceManager();

	std::shared_ptr<Graphics::GpuTexture2D> GetDefaultWhiteTex() { return m_DefaultWhiteTex; }
	std::shared_ptr<Graphics::GpuTexture2D> GetDefaultBlackTex() { return m_DefaultBlackTex; }
private:
	// Default Texture
	std::shared_ptr<Graphics::GpuTexture2D> m_DefaultWhiteTex = nullptr;
	std::shared_ptr<Graphics::GpuTexture2D> m_DefaultBlackTex = nullptr;
};