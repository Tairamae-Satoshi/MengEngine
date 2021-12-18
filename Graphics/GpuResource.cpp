#include "../pch.h"
#include "GpuResource.h"
#include "RenderDevice.h"


namespace Graphics
{

	GpuResource::~GpuResource()
	{
		RenderDevice::GetSingleton().SafeReleaseDeviceObject(m_pResource);
	}

}
