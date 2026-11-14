# Pacai CloudCompare Target Architecture

## Fixed decisions

Windows x64; WinUI 3; C++/WinRT; C++20 application/core code; OpenGL renderer; CCCoreLib reuse; Qt forbidden in Pacai targets and runtime.

## Layers

```text
WinUI 3 / XAML / C++/WinRT
            |
     Pacai Application API
            |
  +---------+----------+-------------+
  |                    |             |
Project Services   Task Runtime   Plugin Host
  |                    |             |
  +---------------- Pacai Core API --+
                       |
          +------------+------------+
          |            |            |
       CCCoreLib    Pacai IO    Pacai Renderer
       algorithms   adapters    OpenGL/WGL
          |            |            |
          +------ Windows Platform--+
```

## Boundary rules

- No Qt headers, Qt types, Qt signals/slots, Qt meta-object macros, Qt resource system, or Qt plugin loading in Pacai targets.
- No WinUI/WinRT types in CCCoreLib or the renderer's portable core.
- UI thread owns XAML objects. Worker threads operate on immutable snapshots or explicitly synchronized core objects.
- C++ exceptions must not cross the plugin C ABI.
- No STL containers, `std::string`, allocator-owned objects, or compiler-specific C++ classes cross the plugin DLL ABI.
- Cancellation is cooperative. Long-running algorithms need cancellation checkpoints; never terminate worker threads forcibly.
- GPU resources are owned by the renderer/context thread and released before the WGL context is destroyed.

## CCCoreLib strategy

- Build CCCoreLib as a separate CMake subproject.
- Force `CCCORELIB_USE_QT_CONCURRENT=OFF`.
- Prefer oneTBB where available and validated. A serial build without TBB is acceptable for the first dependency-isolation check, but does not establish parallel-performance parity.
- Do not assume all CloudCompare `qCC_db` or `qCC_io` code is independent merely because CCCoreLib is.
- Keep upstream code isolated and add Pacai adapters in separate directories.

## Renderer strategy

First POC: WinUI 3 host + native child HWND or another explicitly validated Win32 surface + WGL context. Verify DPI, resize, minimize/restore, focus, input routing, device/context teardown, and multi-window behavior before choosing the final hosting mechanism.

Renderer modules: GraphicsDevice, GLContext, RenderSurface, Scene, Camera, PointCloudRenderer, MeshRenderer, ShaderLibrary, Picking, Selection, RenderScheduler.

## Plugin ABI

Use a versioned C ABI with:
- ABI version and struct size
- explicit ownership/free callbacks
- stable error codes
- host function table
- documented callback threading
- capability declarations
- dependency/version checks

C ABI compatibility is not a security sandbox. Untrusted plugins require process isolation.
