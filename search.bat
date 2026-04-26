@echo off
findstr /N /C:"Pass_Depth_Prepass" /C:"Pass_GBuffer" /C:"Pass_Depth_Light" /C:"IndirectCull" /C:"DummyInstance" /C:"GetInstanceBuffer" /C:"geometry_instances" /C:"GeometryBuffer::Get" /C:"IsCpuDrivenDraw" "%~1"
