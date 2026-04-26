/*
Copyright(c) 2015-2026 Panos Karabelas

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and / or sell
copies of the Software, and to permit persons to whom the Software is furnished
to do so, subject to the following conditions :
The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

//= INCLUDES =========
#include "common.hlsl"
//====================

// per-triangle cull, dispatched indirectly with one workgroup per surviving meshlet
// each thread handles one triangle, survivors are appended into visible_triangles and the final draw's vertex_count is bumped by 3
// flags bit 0 also opts out of backface cull (skinned / hw-instanced fallback, normals are unreliable)
// flags bit 3 is the explicit two-sided flag for materials that disable backface culling

[numthreads(MESHLET_MAX_TRIANGLES, 1, 1)]
void main_cs(uint3 gid : SV_GroupID, uint3 lid : SV_GroupThreadID)
{
    uint mi_idx       = gid.x;
    uint triangle_idx = lid.x;

    // f4_value.x = max_meshlet_instances cap (matches the meshlet cull cap so we do not read garbage)
    // f4_value.y = max_visible_triangles cap, the cull shader stops appending past this
    uint max_meshlet_instances = (uint)pass_get_f4_value().x;
    uint max_visible_triangles = (uint)pass_get_f4_value().y;
    if (mi_idx >= max_meshlet_instances)
        return;

    MeshletInstance mi = meshlet_instances[mi_idx];
    DrawData draw      = indirect_draw_data[mi.draw_index];
    MeshletBounds mb   = meshlet_bounds[mi.meshlet_index];

    uint triangle_count = mb.index_count / 3u;
    if (triangle_idx >= triangle_count)
        return;

    // load the three vertex indices for this triangle from the global index buffer
    uint base_index_pos = draw.lod_first_index + mb.first_index + triangle_idx * 3u;
    uint i0 = geometry_indices[base_index_pos + 0u] + draw.lod_vertex_offset;
    uint i1 = geometry_indices[base_index_pos + 1u] + draw.lod_vertex_offset;
    uint i2 = geometry_indices[base_index_pos + 2u] + draw.lod_vertex_offset;

    PulledVertex v0 = geometry_vertices[i0];
    PulledVertex v1 = geometry_vertices[i1];
    PulledVertex v2 = geometry_vertices[i2];

    // transform to world space, engine convention places the vector on the left of mul
    float3 p0_world = mul(float4(v0.position, 1.0f), draw.transform).xyz;
    float3 p1_world = mul(float4(v1.position, 1.0f), draw.transform).xyz;
    float3 p2_world = mul(float4(v2.position, 1.0f), draw.transform).xyz;

    // backface cull, world-space normal vs view direction, skipped for two-sided / skinned / hw-instanced draws
    bool skip_backface = (draw.flags & 1u) != 0u || (draw.flags & 8u) != 0u;
    if (!skip_backface)
    {
        // ccw front, the cross of two edges points along the front-face normal
        float3 face_normal = cross(p1_world - p0_world, p2_world - p0_world);
        float3 view_dir    = p0_world - buffer_frame.camera_position;
        if (dot(face_normal, view_dir) > 0.0f)
            return;
    }

    // transform to clip space for frustum + sub-pixel tests
    float4 p0_clip = mul(float4(p0_world, 1.0f), buffer_frame.view_projection);
    float4 p1_clip = mul(float4(p1_world, 1.0f), buffer_frame.view_projection);
    float4 p2_clip = mul(float4(p2_world, 1.0f), buffer_frame.view_projection);

    // drop triangles fully behind the camera, they would produce garbage post-projection
    if (p0_clip.w <= 0.0f && p1_clip.w <= 0.0f && p2_clip.w <= 0.0f)
        return;

    // frustum cull only when all three vertices are in front of the camera, the half-space tests fail with negative w
    // half-space form keeps the test stable across reverse-z and avoids the inverted compare when w flips sign
    if (p0_clip.w > 0.0f && p1_clip.w > 0.0f && p2_clip.w > 0.0f)
    {
        bool out_left   = (p0_clip.x + p0_clip.w < 0.0f) && (p1_clip.x + p1_clip.w < 0.0f) && (p2_clip.x + p2_clip.w < 0.0f);
        bool out_right  = (p0_clip.w - p0_clip.x < 0.0f) && (p1_clip.w - p1_clip.x < 0.0f) && (p2_clip.w - p2_clip.x < 0.0f);
        bool out_bottom = (p0_clip.y + p0_clip.w < 0.0f) && (p1_clip.y + p1_clip.w < 0.0f) && (p2_clip.y + p2_clip.w < 0.0f);
        bool out_top    = (p0_clip.w - p0_clip.y < 0.0f) && (p1_clip.w - p1_clip.y < 0.0f) && (p2_clip.w - p2_clip.y < 0.0f);
        bool out_near   = (p0_clip.w - p0_clip.z < 0.0f) && (p1_clip.w - p1_clip.z < 0.0f) && (p2_clip.w - p2_clip.z < 0.0f);
        bool out_far    = (p0_clip.z          < 0.0f)    && (p1_clip.z          < 0.0f)    && (p2_clip.z          < 0.0f);
        if (out_left || out_right || out_bottom || out_top || out_near || out_far)
            return;

        // sub-pixel cull only on the fully-in-front path, behind-camera vertices break the perspective divide
        float2 s0 = p0_clip.xy / p0_clip.w;
        float2 s1 = p1_clip.xy / p1_clip.w;
        float2 s2 = p2_clip.xy / p2_clip.w;
        float2 min_ndc = min(s0, min(s1, s2));
        float2 max_ndc = max(s0, max(s1, s2));
        float2 px      = buffer_frame.resolution_render * buffer_frame.resolution_scale * 0.5f;
        int2   min_px  = int2(floor(min_ndc * px));
        int2   max_px  = int2(floor(max_ndc * px));
        if (all(min_px == max_px))
            return;
    }

    // append survivor, bounds-check first then commit, dropping past the cap is preferable to writing oob
    // also clamp the draw's vertex_count via interlocked min so the indirect draw does not read past the buffer
    uint slot;
    InterlockedAdd(indirect_draw_args[0].index_count, 3u, slot);
    uint triangle_slot = slot / 3u;
    if (triangle_slot >= max_visible_triangles)
    {
        InterlockedMin(indirect_draw_args[0].index_count, max_visible_triangles * 3u);
        return;
    }
    visible_triangles[triangle_slot] = VISIBLE_TRI_PACK(mi_idx, triangle_idx);
}
