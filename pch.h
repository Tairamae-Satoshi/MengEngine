#pragma once

// pch.h: 这是预编译标头文件。
// 下方列出的文件仅编译一次，提高了将来生成的生成性能。
// 这还将影响 IntelliSense 性能，包括代码完成和许多代码浏览功能。
// 但是，如果此处列出的文件中的任何一个在生成之间有更新，它们全部都将被重新编译。
// 请勿在此处添加要频繁更新的文件，这将使得性能优势无效。

#ifndef PCH_H
#define PCH_H

// 添加要在此处预编译的标头
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif

// Windows
#include <windows.h>
#include <WindowsX.h>
#include <wrl.h>


// Direct3d12
#include <d3d12.h>
#include <D3d12SDKLayers.h>
#include <dxgi1_4.h>
#include <D3Dcompiler.h>
#include <DirectXMath.h>
#include <DirectXColors.h>
#include <DirectXPackedVector.h>

#pragma comment(lib, "d3d12.lib")
#pragma comment(lib, "dxgi.lib")
#pragma comment(lib,"d3dcompiler.lib")
#pragma comment(lib,"dxguid.lib")

// Utility
#include "Common/d3dx12.h"
#include "Common/SimpleMath.h"
//#include "Common/DxException.h"
//#include "Common/PathUtil.h"
#include "Common/d3dUtil.h"
#include "Common/MathHelper.h"
#include "Common/Singleton.hpp"
#include "Common/Debug.h"
#include "Common/HashUtils.hpp"
// C++
#include <array>
#include <vector>
#include <queue>
#include <deque>
#include <set>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <memory>
#include <string>
#include <exception>
#include <shellapi.h>
#include <cassert>
#include <iostream>
#include <limits>
#include <mutex>
#include <cmath>

// Meng
#include "Graphics/GraphicsEnums.h"
//#include "Math/Common.h"
//#include "Math/VectorMath.h"
#include "Common/Align.h"
#include "Renderer/FrameResource.h"

// imhui
#include "Vendor/GUI/imgui.h"
#include "Vendor/GUI/imgui_impl_win32.h"
#include "Vendor/GUI/imgui_impl_dx12.h"

//#include "Renderer/ConstantBufferDefinition.h"


#define D3D12_GPU_VIRTUAL_ADDRESS_NULL      ((D3D12_GPU_VIRTUAL_ADDRESS)0)
#define D3D12_GPU_VIRTUAL_ADDRESS_UNKNOWN   ((D3D12_GPU_VIRTUAL_ADDRESS)-1)

// Dynamic Resource的Page大小，单位字节，1M
#define DYNAMIC_RESOURCE_PAGE_SIZE 1048576

#endif //PCH_H
