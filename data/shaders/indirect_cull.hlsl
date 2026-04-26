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

// gpu-driven per-renderable hi-z occlusion culling, deterministic 1:1 task to draw mapping
// each thread writes its draw at output_index = task_index, culled draws are written with instance_count = 0
// the cpu primes indirect_draw_count[0] to the cull task count so vkcmddrawindexedindirectcount sees every slot
// this removes a former interlockedadd-based compaction that was racing args and data across draws

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
    bool is_hw_instanced = (draw.flags & 4u) != 0u;

    // every task owns slot task_index in the output buffers, no compaction
    IndirectDrawArgs out_args = indirect_draw_args[task.draw_index];
    if (!is_hw_instanced)
    {
        out_args.instance_count = 1;
        out_args.first_instance = 0;
    }

    DrawData out_draw       = draw;
    out_draw.instance_index = task.instance_index;

    bool is_visible = true;

    if (!skip_cull)
    {
        // read the world-space aabb for this renderable
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

    // culled draws survive in the array as zero-instance draws so the slot is harmless
    if (!is_visible)
    {
        out_args.instance_count = 0;
    }

    indirect_draw_args_out[task_index] = out_args;
    indirect_draw_data_out[task_index] = out_draw;
}
