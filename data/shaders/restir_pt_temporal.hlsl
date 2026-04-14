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

//= INCLUDES ===================
#include "common.hlsl"
#include "restir_reservoir.hlsl"
//==============================

static const float TEMPORAL_MIN_CONFIDENCE = 0.1f;

bool check_temporal_visibility(float3 shading_pos, float3 shading_normal, float3 sample_hit_pos, float3 sample_hit_normal, float3 prev_shading_pos)
{
    float3 dir = sample_hit_pos - shading_pos;
    float dist = length(dir);

    if (dist < RESTIR_VIS_PLANE_MIN)
        return true;

    dir /= dist;

    // reject samples behind the surface
    float cos_theta = dot(dir, shading_normal);
    if (cos_theta <= 0.05f)
        return false;

    // reject backfacing samples
    float cos_back = dot(sample_hit_normal, -dir);
    if (cos_back <= RESTIR_VIS_COS_BACK)
        return false;

    // reject samples below surface plane
    float plane_dist = dot(sample_hit_pos - shading_pos, shading_normal);
    if (plane_dist < RESTIR_VIS_PLANE_MIN)
        return false;

    // direction similarity check prevents interior/exterior sample bleeding
    float3 dir_from_prev = normalize(sample_hit_pos - prev_shading_pos);
    float direction_similarity = dot(dir, dir_from_prev);
    float3 prev_to_current = shading_pos - prev_shading_pos;
    float prev_current_dist = length(prev_to_current);
    float relative_shift = prev_current_dist / max(dist, 0.01f);
    float min_similarity = lerp(0.9f, 0.99f, saturate(relative_shift * 2.0f));
    if (direction_similarity < min_similarity)
        return false;

    if (dist < RESTIR_VIS_MIN_DIST)
        return true;

    // trace shadow ray to verify visibility
    RayDesc ray;
    ray.Origin    = shading_pos + shading_normal * RESTIR_RAY_NORMAL_OFFSET;
    ray.Direction = dir;
    ray.TMin      = RESTIR_RAY_T_MIN;
    ray.TMax      = max(dist - RESTIR_RAY_NORMAL_OFFSET, RESTIR_RAY_T_MIN);

    RayQuery<RAY_FLAG_ACCEPT_FIRST_HIT_AND_END_SEARCH | RAY_FLAG_SKIP_CLOSEST_HIT_SHADER> query;
    query.TraceRayInline(tlas, RAY_FLAG_NONE, 0xFF, ray);
    query.Proceed();

    return query.CommittedStatus() == COMMITTED_NOTHING;
}

float2 reproject_to_previous_frame(float2 current_uv)
{
    float2 velocity_ndc = tex_velocity.SampleLevel(GET_SAMPLER(sampler_point_clamp), current_uv, 0).xy;
    float2 velocity_uv  = velocity_ndc * float2(0.5f, -0.5f);
    return current_uv - velocity_uv;
}

bool is_temporal_sample_valid(float2 current_uv, float2 prev_uv, float3 current_pos, float3 current_normal, float current_depth, float2 screen_resolution, out float confidence)
{
    confidence = 0.0f;

    if (!is_valid_uv(prev_uv))
        return false;

    // verify reprojection accuracy against the previous frame transform
    float4 prev_clip        = mul(float4(current_pos, 1.0f), buffer_frame.view_projection_previous);
    float3 prev_ndc         = prev_clip.xyz / prev_clip.w;
    float2 expected_prev_uv = prev_ndc.xy * float2(0.5f, -0.5f) + 0.5f;
    float2 reproj_diff = abs(prev_uv - expected_prev_uv) * screen_resolution;
    float reproj_dist  = length(reproj_diff);

    float2 motion       = (current_uv - prev_uv) * screen_resolution;
    float motion_length = length(motion);

    float motion_factor = saturate(motion_length / 32.0f);
    float reproj_tolerance = lerp(1.5f, 0.75f, motion_factor);
    if (reproj_dist > reproj_tolerance)
        return false;

    float normal_threshold = lerp(0.9f, 0.97f, motion_factor);
    float3 prev_uv_normal   = get_normal(prev_uv);
    float normal_similarity = dot(current_normal, prev_uv_normal);
    if (normal_similarity < normal_threshold)
        return false;

    // detect depth edges
    float2 texel_size    = 1.0f / screen_resolution;
    float depth_left     = linearize_depth(tex_depth.SampleLevel(GET_SAMPLER(sampler_point_clamp), current_uv + float2(-texel_size.x, 0), 0).r);
    float depth_right    = linearize_depth(tex_depth.SampleLevel(GET_SAMPLER(sampler_point_clamp), current_uv + float2(texel_size.x, 0), 0).r);
    float depth_up       = linearize_depth(tex_depth.SampleLevel(GET_SAMPLER(sampler_point_clamp), current_uv + float2(0, -texel_size.y), 0).r);
    float depth_down     = linearize_depth(tex_depth.SampleLevel(GET_SAMPLER(sampler_point_clamp), current_uv + float2(0, texel_size.y), 0).r);
    float depth_gradient = abs(depth_left - depth_right) + abs(depth_up - depth_down);
    bool is_depth_edge   = depth_gradient > current_depth * 0.05f;
    if (is_depth_edge)
    {
        reproj_tolerance *= 0.75f;
        normal_threshold = max(normal_threshold, 0.95f);
    }

    if (reproj_dist > reproj_tolerance || normal_similarity < normal_threshold)
        return false;

    float edge_penalty = is_depth_edge ? saturate(1.0f - motion_length / 12.0f) : 1.0f;

    // combine confidence factors conservatively so moving pixels shed stale history quickly
    float reproj_confidence = saturate(1.0f - reproj_dist / reproj_tolerance);
    float normal_confidence = saturate((normal_similarity - normal_threshold) / (1.0f - normal_threshold));
    float motion_confidence = saturate(1.0f - motion_length / 32.0f);
    confidence = reproj_confidence * normal_confidence * motion_confidence * edge_penalty;

    if (confidence < TEMPORAL_MIN_CONFIDENCE)
        return false;

    return true;
}

[numthreads(THREAD_GROUP_COUNT_X, THREAD_GROUP_COUNT_Y, 1)]
void main_cs(uint3 dispatch_id : SV_DispatchThreadID)
{
    uint2 pixel = dispatch_id.xy;
    uint resolution_x, resolution_y;
    tex_uav.GetDimensions(resolution_x, resolution_y);
    float2 resolution = float2(resolution_x, resolution_y);

    if (pixel.x >= resolution_x || pixel.y >= resolution_y)
        return;

    float2 uv = (pixel + 0.5f) / resolution;

    float depth = tex_depth.SampleLevel(GET_SAMPLER(sampler_point_clamp), uv, 0).r;
    if (depth <= 0.0f)
        return;

    // gather surface properties
    float3 pos_ws    = get_position(uv);
    float3 normal_ws = get_normal(uv);
    float3 view_dir  = normalize(buffer_frame.camera_position - pos_ws);
    float4 material = tex_material.SampleLevel(GET_SAMPLER(sampler_point_clamp), uv, 0);
    float3 albedo   = saturate(tex_albedo.SampleLevel(GET_SAMPLER(sampler_point_clamp), uv, 0).rgb);
    float roughness = max(material.r, 0.04f);
    float metallic  = material.g;

    // load current reservoir
    Reservoir current = unpack_reservoir(
        tex_reservoir0[pixel],
        tex_reservoir1[pixel],
        tex_reservoir2[pixel],
        tex_reservoir3[pixel],
        tex_reservoir4[pixel]
    );

    if (!is_reservoir_valid(current))
        current = create_empty_reservoir();

    uint seed = create_seed_for_pass(pixel, buffer_frame.frame, 1);
    Reservoir combined = create_empty_reservoir();
    float confidence_weight_sum   = 0.0f;
    float confidence_weight_total = 0.0f;

    float target_pdf_current;
    if (is_sky_sample(current.sample))
    {
        target_pdf_current = calculate_target_pdf_sky(current.sample.radiance,
            current.sample.direction, normal_ws, view_dir, albedo, roughness, metallic);
    }
    else
    {
        target_pdf_current = calculate_target_pdf_with_geometry(current.sample.radiance,
            pos_ws, normal_ws, view_dir, current.sample.hit_position, current.sample.hit_normal,
            albedo, roughness, metallic);
    }
    if (target_pdf_current <= 0.0f)
        target_pdf_current = calculate_target_pdf(current.sample.radiance);

    // initialize combined reservoir with the current stream contribution
    float weight_current     = target_pdf_current * current.W * current.M;
    combined.weight_sum      = weight_current;
    combined.M               = current.M;
    combined.sample          = current.sample;
    combined.target_pdf      = target_pdf_current;
    confidence_weight_sum    = current.confidence * max(weight_current, 0.0f);
    confidence_weight_total  = max(weight_current, 0.0f);

    // temporal reuse
    float2 prev_uv = reproject_to_previous_frame(uv);
    float temporal_confidence = 0.0f;
    float linear_depth = linearize_depth(depth);

    if (is_temporal_sample_valid(uv, prev_uv, pos_ws, normal_ws, linear_depth, buffer_frame.resolution_render, temporal_confidence))
    {
        float2 prev_pixel_f = prev_uv * resolution;
        bool in_bounds = prev_pixel_f.x >= 0.5f && prev_pixel_f.x < resolution.x - 0.5f &&
                         prev_pixel_f.y >= 0.5f && prev_pixel_f.y < resolution.y - 0.5f;

        if (in_bounds && temporal_confidence > 0.0f)
        {
            int2 prev_pixel = int2(prev_pixel_f);

            Reservoir temporal = unpack_reservoir(
                tex_reservoir_prev0[prev_pixel],
                tex_reservoir_prev1[prev_pixel],
                tex_reservoir_prev2[prev_pixel],
                tex_reservoir_prev3[prev_pixel],
                tex_reservoir_prev4[prev_pixel]
            );

            if (is_reservoir_valid(temporal) && temporal.M > 0 && temporal.W > 0)
            {
                float temp_lum = dot(temporal.sample.radiance, float3(0.299f, 0.587f, 0.114f));
                if (temp_lum > 50.0f)
                    temporal.sample.radiance *= 50.0f / temp_lum;

                // decay low-confidence history before it can dominate the new frame
                float temporal_scale = RESTIR_TEMPORAL_DECAY * temporal_confidence;
                temporal.M          *= temporal_scale;
                temporal.weight_sum *= temporal_scale;
                temporal.age        += 1.0f;

                // cap temporal mass based on confidence and staleness
                float staleness_factor = saturate(1.0f - temporal.age / 64.0f);
                float effective_M_cap  = max(1.0f, RESTIR_M_CAP * temporal_confidence * staleness_factor);
                clamp_reservoir_M(temporal, effective_M_cap);

                if (temporal.M > 0.0f && temporal.weight_sum > 0.0f && is_sky_sample(temporal.sample))
                {
                    float n_dot_sky = dot(normal_ws, temporal.sample.direction);
                    if (n_dot_sky > 0.0f)
                    {
                        float target_pdf_temporal = calculate_target_pdf_sky(temporal.sample.radiance,
                            temporal.sample.direction, normal_ws, view_dir, albedo, roughness, metallic);

                        if (target_pdf_temporal > 0.0f)
                        {
                            float weight_temporal = target_pdf_temporal * temporal.W * temporal.M;

                            combined.weight_sum += weight_temporal;
                            combined.M += temporal.M;
                            confidence_weight_sum   += temporal.confidence * max(weight_temporal, 0.0f);
                            confidence_weight_total += max(weight_temporal, 0.0f);

                            if (random_float(seed) * combined.weight_sum < weight_temporal)
                            {
                                combined.sample     = temporal.sample;
                                combined.target_pdf = target_pdf_temporal;
                            }
                        }
                    }
                }
                else if (temporal.M > 0.0f && temporal.weight_sum > 0.0f)
                {
                    float3 reference_shading_pos = pos_ws;

                    bool temporal_visible = temporal.sample.path_length == 0 ||
                                            all(temporal.sample.radiance <= 0.0f) ||
                                            check_temporal_visibility(pos_ws, normal_ws, temporal.sample.hit_position, temporal.sample.hit_normal, reference_shading_pos);

                    if (temporal_visible)
                    {
                        float jacobian = compute_jacobian(temporal.sample.hit_position, reference_shading_pos, pos_ws, temporal.sample.hit_normal, normal_ws);

                        if (jacobian > 0.0f)
                        {
                            float3 dir_to_hit = normalize(temporal.sample.hit_position - pos_ws);
                            float n_dot_l = dot(normal_ws, dir_to_hit);

                            if (n_dot_l > 0.0f)
                            {
                                float target_pdf_temporal = calculate_target_pdf_with_geometry(temporal.sample.radiance,
                                    pos_ws, normal_ws, view_dir, temporal.sample.hit_position, temporal.sample.hit_normal,
                                    albedo, roughness, metallic);

                                if (target_pdf_temporal > 0.0f)
                                {
                                    float weight_temporal = target_pdf_temporal * jacobian * temporal.W * temporal.M;

                                    combined.weight_sum += weight_temporal;
                                    combined.M += temporal.M;
                                    confidence_weight_sum   += temporal.confidence * max(weight_temporal, 0.0f);
                                    confidence_weight_total += max(weight_temporal, 0.0f);

                                    if (random_float(seed) * combined.weight_sum < weight_temporal)
                                    {
                                        combined.sample     = temporal.sample;
                                        combined.target_pdf = target_pdf_temporal;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    clamp_reservoir_M(combined, RESTIR_M_CAP);

    float final_target_pdf;
    if (is_sky_sample(combined.sample))
    {
        final_target_pdf = calculate_target_pdf_sky(combined.sample.radiance,
            combined.sample.direction, normal_ws, view_dir, albedo, roughness, metallic);
    }
    else
    {
        final_target_pdf = calculate_target_pdf_with_geometry(combined.sample.radiance,
            pos_ws, normal_ws, view_dir, combined.sample.hit_position, combined.sample.hit_normal,
            albedo, roughness, metallic);
    }
    if (final_target_pdf <= 0.0f)
        final_target_pdf = calculate_target_pdf(combined.sample.radiance);
    combined.target_pdf = final_target_pdf;

    if (final_target_pdf > 0 && combined.M > 0)
        combined.W = combined.weight_sum / (final_target_pdf * combined.M);
    else
        combined.W = 0;

    float w_clamp = get_w_clamp_for_sample(combined.sample);
    combined.W = min(combined.W, w_clamp);

    combined.confidence = confidence_weight_total > 0.0f ? saturate(confidence_weight_sum / confidence_weight_total) : 0.0f;

    // store reservoir
    float4 t0, t1, t2, t3, t4;
    pack_reservoir(combined, t0, t1, t2, t3, t4);
    tex_reservoir0[pixel] = t0;
    tex_reservoir1[pixel] = t1;
    tex_reservoir2[pixel] = t2;
    tex_reservoir3[pixel] = t3;
    tex_reservoir4[pixel] = t4;

    // output GI with soft clamp
    float3 gi = combined.sample.radiance * combined.W;
    gi = soft_clamp_gi(gi, combined.sample);

    tex_uav[pixel] = float4(gi, 1.0f);
}
