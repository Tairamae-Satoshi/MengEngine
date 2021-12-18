#pragma once

#include "GpuTexture.h"

namespace Graphics 
{
    /**
    * 
    */
    class GpuTexture2D : public GpuTexture
    {
    public:

        GpuTexture2D(UINT32 width, UINT32 height, DXGI_FORMAT format, UINT64 RowPitchBytes, const void* InitialData);

        std::shared_ptr<GpuResourceDescriptor> CreateSRV();

    protected:

    };

}