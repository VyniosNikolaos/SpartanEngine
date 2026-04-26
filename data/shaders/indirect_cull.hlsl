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

// gpu-driven meshlet cull, per-meshlet backface cone + per-renderable hi-z occlusion
// dispatches over the cpu-built cull task array, each task is one (renderable, meshlet, instance) tuple
// survivors are atomically compacted into meshlet_instances, triangle_dispatch_args.group_count_x is bumped
// the triangle cull pass then dispatches one workgroup per surviving meshlet to perform per-triangle culling

[numthreads(256, 1, 1)]
void main_cs(uint3 dispatch_thread_id : SV_DispatchThreadID)
{
    uint task_index = dispatch_thread_id.x;
    uint task_count = (uint)pass_get_f4_value().x;
    if (task_index >= task_count)
        return;

    CullTask task = cull_tasks[task_index];
    DrawData draw = indirect_draw_data[task.draw_index];

    bool skip_cull       = (draw.flags & 1u) != 0u;
    bool is_per_instance = (draw.flags & 2u) != 0u;

    bool is_visible = true;

    // per-meshlet backface cone culling, skipped for per-instance draws since instances may carry independent rotations
    if (!skip_cull && !is_per_instance)
    {
        MeshletBounds mb = meshlet_bounds[task.meshlet_index];

        uint packed   = mb.cone_axis_cutoff;
        int axis_x_s8 = (int)((packed      ) & 0xffu); axis_x_s8 = (axis_x_s8 << 24) >> 24;
        int axis_y_s8 = (int)((packed >> 8 ) & 0xffu); axis_y_s8 = (axis_y_s8 << 24) >> 24;
        int axis_z_s8 = (int)((packed >> 16) & 0xffu); axis_z_s8 = (axis_z_s8 << 24) >> 24;
        int cutoff_s8 = (int)((packed >> 24) & 0xffu); cutoff_s8 = (cutoff_s8 << 24) >> 24;

        // cutoff_s8 == 127 marks a degenerate cone, do not cull
        if (cutoff_s8 < 127)
        {
            float3 cone_axis_local = float3(axis_x_s8, axis_y_s8, axis_z_s8) / 127.0f;
            float  cone_cutoff     = (float)cutoff_s8 / 127.0f;

            // engine convention places the vector on the left of mul
            float3 cone_axis_world = normalize(mul(cone_axis_local, (float3x3)draw.transform));
            float3 center_world    = mul(float4(mb.center, 1.0f), draw.transform).xyz;
            float3 to_meshlet      = center_world - buffer_frame.camera_position;
            float  to_meshlet_len  = length(to_meshlet);

            // perspective formula from meshoptimizer, the radius term keeps near meshlets visible when the bounding sphere overlaps the camera
            float lhs = dot(to_meshlet, cone_axis_world);
            float rhs = cone_cutoff * to_meshlet_len + mb.radius;
            if (to_meshlet_len > 0.0f && lhs >= rhs)
            {
                is_visible = false;
            }
        }
    }

    // per-renderable frustum + hi-z occlusion against the world-space aabb
    if (is_visible && !skip_cull)
    {
        Aabb current_aabb = aabbs[draw.aabb_index];

        float3 corners_world[8] =
        {
            current_aabb.min,
            float3(current_aabb.min.x, current_aabb.min.y, current_aabb.max.z),
            float3(current_aabb.min.x, current_aabb.max.y, current_aabb.min.z),
            float3(current_aabb.min.x, current_aabb.max.y, current_aabb.max.z),
            float3(current_aabb.max.x, current_aabb.min.y, current_aabb.min.z),
            float3(current_aabb.max.x, current_aabb.min.y, current_aabb.max.z),
            float3(current_aabb.max.x, current_aabb.max.y, current_aabb.min.z),
            current_aabb.max
        };

        float2 min_ndc      = float2(1e30f, 1e30f);
        float2 max_ndc      = float2(-1e30f, -1e30f);
        float closest_box_z = -1e30f;
        bool any_behind     = false;
        bool any_front      = false;
        for (int i = 0; i < 8; ++i)
        {
            float4 clip = mul(float4(corners_world[i], 1.0), buffer_frame.view_projection);
            if (clip.w <= 0.0f)
            {
                any_behind = true;
            }
            else
            {
                float3 ndc    = clip.xyz / clip.w;
                min_ndc       = min(min_ndc, ndc.xy);
                max_ndc       = max(max_ndc, ndc.xy);
                closest_box_z = max(closest_box_z, ndc.z);
                any_front     = true;
            }
        }

        if (!any_front)
        {
            is_visible = false;
        }
        else if (!any_behind)
        {
            if (max_ndc.x < -1.0f || min_ndc.x > 1.0f || max_ndc.y < -1.0f || min_ndc.y > 1.0f)
                is_visible = false;

            if (is_visible)
            {
                float2 uv_a   = saturate(ndc_to_uv(min_ndc));
                float2 uv_b   = saturate(ndc_to_uv(max_ndc));
                float2 min_uv = min(uv_a, uv_b);
                float2 max_uv = max(uv_a, uv_b);

                float2 render_size;
                tex.GetDimensions(render_size.x, render_size.y);
                float4 box_uvs = float4(min_uv, max_uv);

                float2 uv_extent     = max_uv - min_uv;
                int2   size          = uv_extent * render_size;
                float  mip           = ceil(log2(max(max(size.x, size.y), 1)));
                float  max_mip_level = pass_get_f4_value().y;
                mip                  = clamp(mip, 0, max_mip_level);

                float  level_lower = max(mip - 1, 0);
                float2 scale_mip   = exp2(-level_lower);
                float2 a           = floor(box_uvs.xy * scale_mip * render_size);
                float2 b           = ceil(box_uvs.zw * scale_mip * render_size);
                float2 dims        = b - a;

                if (dims.x <= 2 && dims.y <= 2)
                    mip = level_lower;

                float2 mip_texel = exp2(mip) / render_size;
                box_uvs.xy = saturate(box_uvs.xy - mip_texel);
                box_uvs.zw = saturate(box_uvs.zw + mip_texel);

                float4 scaled_uvs = box_uvs * buffer_frame.resolution_scale;
                float2 center_uv  = (scaled_uvs.xy + scaled_uvs.zw) * 0.5f;
                float4 depth = float4(
                    tex.SampleLevel(GET_SAMPLER(sampler_point_clamp), scaled_uvs.xy, mip).r,
                    tex.SampleLevel(GET_SAMPLER(sampler_point_clamp), scaled_uvs.zy, mip).r,
                    tex.SampleLevel(GET_SAMPLER(sampler_point_clamp), scaled_uvs.xw, mip).r,
                    tex.SampleLevel(GET_SAMPLER(sampler_point_clamp), scaled_uvs.zw, mip).r
                );
                float depth_center = tex.SampleLevel(GET_SAMPLER(sampler_point_clamp), center_uv, mip).r;

                float furthest_z = min(min(min(min(depth.x, depth.y), depth.z), depth.w), depth_center);
                is_visible       = closest_box_z > furthest_z - 0.01;
            }
        }
    }

    if (!is_visible)
        return;

    // atomically reserve a contiguous range, normally one entry but the hw-instancing fallback fans out into instance_count entries
    // group_count_x of the triangle cull's indirect dispatch is bumped in lockstep, one workgroup per meshlet survivor
    uint instance_count        = max(task.instance_count, 1u);
    uint max_meshlet_instances = (uint)pass_get_f4_value().z;
    uint slot;
    InterlockedAdd(triangle_dispatch_args[0].group_count_x, instance_count, slot);

    // bounds-check the destination range, dropping survivors past the cap is preferable to writing oob and corrupting other buffers
    if (slot >= max_meshlet_instances)
        return;
    uint write_count = min(instance_count, max_meshlet_instances - slot);

    for (uint i = 0; i < write_count; i++)
    {
        MeshletInstance mi;
        mi.draw_index     = task.draw_index;
        mi.meshlet_index  = task.meshlet_index;
        mi.instance_index = task.instance_index + i;
        mi.padding0       = 0;
        meshlet_instances[slot + i] = mi;
    }
}
