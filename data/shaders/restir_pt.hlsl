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

static const uint INITIAL_CANDIDATE_SAMPLES   = 8;
static const float RUSSIAN_ROULETTE_PROB      = 0.85f;
static const uint RUSSIAN_ROULETTE_START      = 3;
static const float MIN_AREA_LIGHT_SOLID_ANGLE = 1e-4f;
static const float SKY_MIP_LEVEL              = 2.0f;
// half-angle (radians) of the cone sampled around the dominant directional light
static const float SUN_CONE_HALF_ANGLE        = 0.015f;
static const float SUN_SAMPLE_PROBABILITY     = 0.5f;

struct [raypayload] PathPayload
{
    float3 hit_position   : read(caller) : write(closesthit);
    float3 hit_normal     : read(caller) : write(closesthit);
    float3 geometric_normal : read(caller) : write(closesthit);
    float3 albedo         : read(caller) : write(closesthit);
    float3 emission       : read(caller) : write(closesthit, miss);
    float  roughness      : read(caller) : write(closesthit);
    float  metallic       : read(caller) : write(closesthit);
    float  triangle_area  : read(caller) : write(closesthit);
    bool   hit            : read(caller) : write(closesthit, miss);
};

float compute_spec_probability(float roughness, float metallic, float n_dot_v)
{
    float fresnel_factor   = pow(1.0f - n_dot_v, 5.0f);
    float base_spec        = lerp(0.04f, 1.0f, metallic);
    float spec_response    = lerp(base_spec, 1.0f, fresnel_factor);
    float roughness_factor = 1.0f - roughness * roughness;
    float spec_prob        = lerp(0.1f, 0.9f, spec_response * roughness_factor + metallic * 0.5f);
    return clamp(spec_prob, 0.1f, 0.9f);
}

float3 evaluate_brdf(float3 albedo, float roughness, float metallic, float3 n, float3 v, float3 l, out float pdf)
{
    float3 h_unnorm = v + l;
    float h_len_sq  = dot(h_unnorm, h_unnorm);

    if (h_len_sq < 1e-6f)
    {
        pdf = 0.0f;
        return float3(0, 0, 0);
    }

    float3 h      = h_unnorm * rsqrt(h_len_sq);
    float n_dot_l = max(dot(n, l), 0.0f);
    float n_dot_v = max(dot(n, v), 0.001f);
    float n_dot_h = max(dot(n, h), 0.0f);
    float v_dot_h = max(dot(v, h), 0.0f);

    if (n_dot_l <= 0.0f)
    {
        pdf = 0.0f;
        return float3(0, 0, 0);
    }

    // diffuse term
    float3 diffuse = albedo * (1.0f / PI);

    // GGX specular distribution
    float alpha   = max(roughness * roughness, 0.001f);
    float alpha2  = alpha * alpha;
    float d_denom = n_dot_h * n_dot_h * (alpha2 - 1.0f) + 1.0f;
    float d       = alpha2 / (PI * d_denom * d_denom + 1e-6f);

    // geometry term
    float r_plus_1 = roughness + 1.0f;
    float k   = (r_plus_1 * r_plus_1) / 8.0f;
    float g_v = n_dot_v / (n_dot_v * (1.0f - k) + k + 1e-6f);
    float g_l = n_dot_l / (n_dot_l * (1.0f - k) + k + 1e-6f);
    float g   = g_v * g_l;

    // fresnel term
    float3 f0 = lerp(float3(0.04f, 0.04f, 0.04f), albedo, metallic);
    float3 f  = f0 + (1.0f - f0) * pow(1.0f - v_dot_h, 5.0f);

    // combine specular with energy compensation
    float3 specular = (d * g * f) / (4.0f * n_dot_v * n_dot_l + 1e-6f);
    float3 f_avg       = f0 + (1.0f - f0) / 21.0f;
    float energy_bias  = lerp(0.0f, 0.5f, roughness);
    float3 energy_comp = 1.0f + f_avg * energy_bias;
    specular *= energy_comp;

    // final brdf
    float3 kd   = (1.0f - f) * (1.0f - metallic);
    float3 brdf = kd * diffuse + specular;

    // combined pdf
    float diffuse_pdf = n_dot_l / PI;
    float spec_pdf    = d * n_dot_h / (4.0f * v_dot_h + 1e-6f);
    float spec_prob   = compute_spec_probability(roughness, metallic, n_dot_v);
    pdf = (1.0f - spec_prob) * diffuse_pdf + spec_prob * spec_pdf;

    return brdf * n_dot_l;
}

float3 sample_brdf(float3 albedo, float roughness, float metallic, float3 n, float3 v, float2 xi, out float pdf)
{
    float3 t, b;
    build_orthonormal_basis_fast(n, t, b);

    float n_dot_v      = max(dot(n, v), 0.001f);
    float spec_prob    = compute_spec_probability(roughness, metallic, n_dot_v);
    float prob_diffuse = 1.0f - spec_prob;

    // sample diffuse or specular lobe
    float3 l;
    if (xi.x < prob_diffuse)
    {
        xi.x = xi.x / prob_diffuse;
        float pdf_diffuse;
        float3 local_dir = sample_cosine_hemisphere(xi, pdf_diffuse);
        l = local_to_world(local_dir, n);
    }
    else
    {
        xi.x = (xi.x - prob_diffuse) / (1.0f - prob_diffuse);
        float pdf_h;
        float3 h       = sample_ggx(xi, max(roughness, 0.04f), pdf_h);
        float3 h_world = local_to_world(h, n);
        l = reflect(-v, h_world);
    }

    // compute combined pdf
    float n_dot_l     = max(dot(n, l), 0.001f);
    float diffuse_pdf = n_dot_l / PI;

    float3 h_unnorm = v + l;
    float h_len_sq  = dot(h_unnorm, h_unnorm);

    if (h_len_sq < 1e-6f)
    {
        pdf = diffuse_pdf * prob_diffuse;
        return l;
    }

    float3 h       = h_unnorm * rsqrt(h_len_sq);
    float n_dot_h  = max(dot(n, h), 0.001f);
    float v_dot_h  = max(dot(v, h), 0.001f);
    float alpha    = max(roughness * roughness, 0.001f);
    float alpha2   = alpha * alpha;
    float d_denom  = n_dot_h * n_dot_h * (alpha2 - 1.0f) + 1.0f;
    float d        = alpha2 / (PI * d_denom * d_denom + 1e-6f);
    float spec_pdf = d * n_dot_h / (4.0f * v_dot_h + 1e-6f);

    pdf = prob_diffuse * diffuse_pdf + spec_prob * spec_pdf;
    return l;
}

bool trace_shadow_ray(float3 origin, float3 direction, float max_dist)
{
    float epsilon = max(RESTIR_RAY_T_MIN, compute_ray_offset(origin));

    RayDesc ray;
    ray.Origin    = origin;
    ray.Direction = direction;
    ray.TMin      = epsilon;
    ray.TMax      = max(max_dist - epsilon, epsilon);

    RayQuery<RAY_FLAG_ACCEPT_FIRST_HIT_AND_END_SEARCH | RAY_FLAG_SKIP_CLOSEST_HIT_SHADER> query;
    query.TraceRayInline(tlas, RAY_FLAG_NONE, 0xFF, ray);
    query.Proceed();

    return query.CommittedStatus() == COMMITTED_NOTHING;
}

// approximation used when the closest-hit shader is bypassed (rayquery), since the
// emissive texture can't be sampled from here
float3 probe_emission_estimate(MaterialParameters mat)
{
    if (mat.emissive_from_albedo() || mat.has_texture_emissive())
        return mat.color.rgb * 10.0f;
    return float3(0.0f, 0.0f, 0.0f);
}

PathSample trace_path(float3 origin, float3 direction, inout uint seed)
{
    PathSample sample;
    sample.direction   = direction;
    sample.radiance    = float3(0, 0, 0);
    sample.path_length = 0;
    sample.flags       = 0;
    sample.pdf         = 1.0f;

    float3 ray_origin   = origin;
    float3 ray_dir      = direction;
    float3 throughput   = float3(1, 1, 1);
    bool first_hit      = true;
    float prev_brdf_pdf = 1.0f;
    bool prev_specular  = false;
    bool did_env_sample = false;
    float prev_env_pdf = 0.0f;
    float3 last_bounce_normal = float3(0, 1, 0);
    uint light_count = (uint)buffer_frame.restir_pt_light_count;

    for (uint bounce = 0; bounce < RESTIR_MAX_PATH_LENGTH; bounce++)
    {
        RayDesc ray;
        ray.Origin    = ray_origin;
        ray.Direction = ray_dir;
        ray.TMin      = RESTIR_RAY_T_MIN;
        ray.TMax      = 1000.0f;

        PathPayload payload;
        payload.hit = false;

        TraceRay(tlas, RAY_FLAG_NONE, 0xFF, 0, 1, 0, ray, payload);

        // sky hit handling
        if (!payload.hit)
        {
            float2 sky_uv       = direction_sphere_uv(ray_dir);
            float3 sky_radiance = tex3.SampleLevel(GET_SAMPLER(sampler_trilinear_clamp), sky_uv, SKY_MIP_LEVEL).rgb;
            sky_radiance = clamp_sky_radiance(sky_radiance);

            if (bounce == 0 || prev_specular)
            {
                sample.radiance += throughput * sky_radiance;
            }
            else
            {
                float env_pdf    = max(prev_env_pdf, RESTIR_MIN_PDF);
                float mis_weight = did_env_sample ? power_heuristic(prev_brdf_pdf, env_pdf) : 1.0f;
                sample.radiance += throughput * sky_radiance * mis_weight;
            }

            if (bounce == 0)
            {
                sample.flags |= PATH_FLAG_SKY;
                sample.direction    = ray_dir;
                sample.hit_position = float3(RESTIR_SKY_DISTANCE, RESTIR_SKY_DISTANCE, RESTIR_SKY_DISTANCE);
                sample.hit_normal   = -ray_dir;
            }
            break;
        }

        // store the geometric normal so visibility/jacobian checks aren't fooled by normal maps
        if (first_hit)
        {
            sample.hit_position = payload.hit_position;
            sample.hit_normal   = payload.geometric_normal;
            sample.flags        = (payload.roughness < RESTIR_SPECULAR_THRESHOLD) ? PATH_FLAG_SPECULAR : PATH_FLAG_DIFFUSE;
            first_hit = false;
        }

        sample.path_length = bounce + 1;
        float3 view_dir = -ray_dir;
        last_bounce_normal = payload.hit_normal;

        if (luminance(payload.emission) > 0.0f)
        {
            sample.radiance += throughput * payload.emission;
        }

        float shading_offset = compute_ray_offset(payload.hit_position);
        float3 shading_pos   = payload.hit_position + payload.geometric_normal * shading_offset;

        // direct lighting
        for (uint light_idx = 0; light_idx < light_count; light_idx++)
        {
            LightParameters light = light_parameters[light_idx];

            if (light.intensity <= 0.0f)
                continue;

            uint light_flags    = light.flags;
            bool is_directional = (light_flags & (1u << 0)) != 0;
            bool is_point       = (light_flags & (1u << 1)) != 0;
            bool is_spot        = (light_flags & (1u << 2)) != 0;
            bool is_area        = (light_flags & (1u << 6)) != 0;

            float3 light_color = light.color.rgb;
            float3 light_dir;
            float  light_dist;
            float  light_pdf    = 1.0f;
            float  attenuation  = 1.0f;

            if (is_directional)
            {
                light_dir  = -light.direction;
                light_dist = 1000.0f;
                light_pdf  = 1.0f;
            }
            else if (is_area && light.area_width > 0.0f && light.area_height > 0.0f)
            {
                // area light sampling with solid angle approximation
                float3 light_normal = light.direction;
                float3 light_right, light_up;
                build_orthonormal_basis_fast(light_normal, light_right, light_up);

                float half_width  = light.area_width * 0.5f;
                float half_height = light.area_height * 0.5f;

                float3 light_center = light.position;
                float3 p0 = light_center - light_right * half_width - light_up * half_height;
                float3 p1 = light_center + light_right * half_width - light_up * half_height;
                float3 p2 = light_center + light_right * half_width + light_up * half_height;
                float3 p3 = light_center - light_right * half_width + light_up * half_height;

                float3 v0 = normalize(p0 - shading_pos);
                float3 v1 = normalize(p1 - shading_pos);
                float3 v2 = normalize(p2 - shading_pos);
                float3 v3 = normalize(p3 - shading_pos);

                float solid_angle_approx = 0.0f;
                {
                    float a1 = acos(clamp(dot(v0, v1), -1.0f, 1.0f));
                    float a2 = acos(clamp(dot(v1, v2), -1.0f, 1.0f));
                    float a3 = acos(clamp(dot(v2, v0), -1.0f, 1.0f));
                    float s  = (a1 + a2 + a3) * 0.5f;
                    float excess1 = 4.0f * atan(sqrt(max(0.0f, tan(s * 0.5f) * tan((s - a1) * 0.5f) * tan((s - a2) * 0.5f) * tan((s - a3) * 0.5f))));

                    float b1 = acos(clamp(dot(v0, v2), -1.0f, 1.0f));
                    float b2 = acos(clamp(dot(v2, v3), -1.0f, 1.0f));
                    float b3 = acos(clamp(dot(v3, v0), -1.0f, 1.0f));
                    float t  = (b1 + b2 + b3) * 0.5f;
                    float excess2 = 4.0f * atan(sqrt(max(0.0f, tan(t * 0.5f) * tan((t - b1) * 0.5f) * tan((t - b2) * 0.5f) * tan((t - b3) * 0.5f))));

                    solid_angle_approx = excess1 + excess2;
                }

                float2 xi = random_float2(seed);
                float3 light_sample_pos = light_center
                    + light_right * (xi.x - 0.5f) * light.area_width
                    + light_up * (xi.y - 0.5f) * light.area_height;

                float3 to_light = light_sample_pos - shading_pos;
                light_dist      = length(to_light);
                light_dir       = to_light / light_dist;

                float cos_light = dot(-light_dir, light_normal);
                if (cos_light <= 0.0f)
                    continue;

                float area = light.area_width * light.area_height;
                if (solid_angle_approx > MIN_AREA_LIGHT_SOLID_ANGLE)
                {
                    light_pdf = 1.0f / solid_angle_approx;
                }
                else
                {
                    float solid_angle = (area * cos_light) / (light_dist * light_dist);
                    solid_angle       = max(solid_angle, MIN_AREA_LIGHT_SOLID_ANGLE);
                    light_pdf         = 1.0f / solid_angle;
                }
                attenuation = 1.0f / (1.0f + light_dist * light_dist * 0.01f);
            }
            else if (is_point || is_spot)
            {
                float3 to_light = light.position - shading_pos;
                light_dist      = length(to_light);
                light_dir       = to_light / light_dist;
                light_pdf       = 1.0f;

                float range_factor = saturate(1.0f - light_dist / max(light.range, 0.01f));
                attenuation = range_factor * range_factor / (1.0f + light_dist * light_dist * 0.1f);

                if (is_spot)
                {
                    float cos_angle = dot(-light_dir, light.direction);
                    float cos_outer = cos(light.angle);
                    float cos_inner = cos(light.angle * 0.8f);
                    attenuation *= saturate((cos_angle - cos_outer) / (cos_inner - cos_outer));
                }
            }
            else
            {
                continue;
            }

            float n_dot_l = dot(payload.hit_normal, light_dir);
            if (n_dot_l <= 0.0f)
                continue;

            if (!trace_shadow_ray(shading_pos, light_dir, light_dist))
                continue;

            float brdf_pdf;
            float3 brdf = evaluate_brdf(payload.albedo, payload.roughness, payload.metallic,
                                        payload.hit_normal, view_dir, light_dir, brdf_pdf);

            float mis_weight = is_area ? power_heuristic(light_pdf, brdf_pdf) : 1.0f;

            float3 Li = light_color * light.intensity * attenuation;
            float3 light_contribution = throughput * brdf * Li * mis_weight / max(light_pdf, 1e-6f);

            sample.radiance += light_contribution;
        }

        // emissive / environment probe with sun-aware mixture sampling
        did_env_sample = false;
        prev_env_pdf   = 0.0f;
        {
            float2 env_xi = random_float2(seed);

            float3 sun_dir = float3(0, 1, 0);
            bool   has_sun = false;
            if (light_count > 0)
            {
                LightParameters primary_light = light_parameters[0];
                if ((primary_light.flags & (1u << 0)) != 0 && primary_light.intensity > 0.0f)
                {
                    sun_dir = -primary_light.direction;
                    has_sun = true;
                }
            }

            float sun_cos_max   = cos(SUN_CONE_HALF_ANGLE);
            float sun_cone_pdf  = 1.0f / (2.0f * PI * (1.0f - sun_cos_max));
            float sun_prob      = has_sun ? SUN_SAMPLE_PROBABILITY : 0.0f;

            float3 env_dir;
            float  env_pdf_cos;
            float  env_pdf_sun;
            float  strategy_xi = random_float(seed);

            if (has_sun && strategy_xi < sun_prob)
            {
                // uniform on the spherical cap of the sun cone
                float phi     = 2.0f * PI * env_xi.x;
                float cos_th  = lerp(sun_cos_max, 1.0f, env_xi.y);
                float sin_th  = sqrt(max(0.0f, 1.0f - cos_th * cos_th));
                float3 local  = float3(cos(phi) * sin_th, sin(phi) * sin_th, cos_th);
                env_dir       = local_to_world(local, sun_dir);

                float cos_to_sun = dot(env_dir, sun_dir);
                env_pdf_sun = (cos_to_sun >= sun_cos_max) ? sun_cone_pdf : 0.0f;
                env_pdf_cos = max(dot(env_dir, payload.hit_normal), 0.0f) / PI;
            }
            else
            {
                float3 env_local = sample_cosine_hemisphere(env_xi, env_pdf_cos);
                env_dir = local_to_world(env_local, payload.hit_normal);

                float cos_to_sun = has_sun ? dot(env_dir, sun_dir) : -1.0f;
                env_pdf_sun = (has_sun && cos_to_sun >= sun_cos_max) ? sun_cone_pdf : 0.0f;
            }

            // mixture pdf
            float env_pdf = (1.0f - sun_prob) * env_pdf_cos + sun_prob * env_pdf_sun;

            float env_n_dot_l = dot(payload.hit_normal, env_dir);
            if (env_n_dot_l > 0.0f && env_pdf > RESTIR_MIN_PDF)
            {
                float probe_offset = compute_ray_offset(payload.hit_position);
                RayDesc probe_ray;
                probe_ray.Origin    = shading_pos;
                probe_ray.Direction = env_dir;
                probe_ray.TMin      = probe_offset;
                probe_ray.TMax      = 10000.0f;

                RayQuery<RAY_FLAG_SKIP_CLOSEST_HIT_SHADER> probe_query;
                probe_query.TraceRayInline(tlas, RAY_FLAG_NONE, 0xFF, probe_ray);
                probe_query.Proceed();

                if (probe_query.CommittedStatus() == COMMITTED_TRIANGLE_HIT)
                {
                    uint probe_instance = probe_query.CommittedInstanceID();
                    MaterialParameters probe_mat = material_parameters[probe_instance];
                    float3 probe_emission = probe_emission_estimate(probe_mat);

                    if (luminance(probe_emission) > 0.0f)
                    {
                        float brdf_pdf_probe;
                        float3 brdf_probe = evaluate_brdf(payload.albedo, payload.roughness, payload.metallic,
                                                          payload.hit_normal, view_dir, env_dir, brdf_pdf_probe);

                        float mis_weight = power_heuristic(env_pdf, brdf_pdf_probe);
                        sample.radiance += throughput * brdf_probe * probe_emission * mis_weight / env_pdf;
                    }
                }
                else
                {
                    // missed all geometry - sky contribution
                    did_env_sample = true;
                    prev_env_pdf   = env_pdf;

                    float2 env_uv       = direction_sphere_uv(env_dir);
                    float3 env_radiance = tex3.SampleLevel(GET_SAMPLER(sampler_trilinear_clamp), env_uv, SKY_MIP_LEVEL).rgb;
                    env_radiance = clamp_sky_radiance(env_radiance);

                    float brdf_pdf_env;
                    float3 brdf_env = evaluate_brdf(payload.albedo, payload.roughness, payload.metallic,
                                                    payload.hit_normal, view_dir, env_dir, brdf_pdf_env);

                    float mis_weight_env = power_heuristic(env_pdf, brdf_pdf_env);
                    sample.radiance += throughput * brdf_env * env_radiance * mis_weight_env / env_pdf;
                }
            }
        }

        // russian roulette path termination
        if (bounce >= RUSSIAN_ROULETTE_START)
        {
            float continuation_prob = min(luminance(throughput), RUSSIAN_ROULETTE_PROB);
            if (random_float(seed) > continuation_prob)
                break;
            throughput /= continuation_prob;
        }

        // sample next direction
        float2 xi = random_float2(seed);
        float pdf;
        float3 new_dir = sample_brdf(payload.albedo, payload.roughness, payload.metallic,
                                      payload.hit_normal, view_dir, xi, pdf);

        if (pdf < 1e-6f || dot(new_dir, payload.hit_normal) <= 0 || any(isnan(new_dir)))
            break;

        float unused_pdf;
        float3 brdf = evaluate_brdf(payload.albedo, payload.roughness, payload.metallic,
                                     payload.hit_normal, view_dir, new_dir, unused_pdf);

        throughput *= brdf / pdf;
        prev_brdf_pdf = pdf;
        prev_specular = (payload.roughness < RESTIR_SPECULAR_THRESHOLD);

        ray_origin = payload.hit_position + payload.geometric_normal * shading_offset;
        ray_dir    = new_dir;
    }

    sample.radiance = soft_saturate_radiance(sample.radiance, RESTIR_FIREFLY_LUMA);
    return sample;
}

[shader("raygeneration")]
void ray_gen()
{
    uint2 launch_id   = DispatchRaysIndex().xy;
    uint2 launch_size = DispatchRaysDimensions().xy;
    float2 uv = (launch_id + 0.5f) / launch_size;

    if (geometry_infos[0].vertex_offset == 0xFFFFFFFF)
        return;

    // early out for sky pixels
    float depth = tex_depth.SampleLevel(GET_SAMPLER(sampler_point_clamp), uv, 0).r;
    if (depth <= 0.0f)
    {
        Reservoir empty = create_empty_reservoir();
        float4 t0, t1, t2, t3, t4;
        pack_reservoir(empty, t0, t1, t2, t3, t4);
        tex_reservoir0[launch_id] = t0;
        tex_reservoir1[launch_id] = t1;
        tex_reservoir2[launch_id] = t2;
        tex_reservoir3[launch_id] = t3;
        tex_reservoir4[launch_id] = t4;
        tex_uav[launch_id] = float4(0, 0, 0, 1);
        return;
    }

    uint seed = create_seed_for_pass(launch_id, buffer_frame.frame, 0);

    // gather surface properties
    float3 pos_ws    = get_position(uv);
    float3 normal_ws = get_normal(uv);
    float3 view_dir  = normalize(buffer_frame.camera_position - pos_ws);
    float4 material = tex_material.SampleLevel(GET_SAMPLER(sampler_point_clamp), uv, 0);
    float3 albedo   = saturate(tex_albedo.SampleLevel(GET_SAMPLER(sampler_point_clamp), uv, 0).rgb);
    float roughness = max(material.r, 0.04f);
    float metallic  = material.g;

    float primary_offset = compute_ray_offset(pos_ws);

    Reservoir reservoir = create_empty_reservoir();

    // cosine-hemisphere sampling for diffuse-only gi; specular is handled by the reflection pipeline
    for (uint i = 0; i < INITIAL_CANDIDATE_SAMPLES; i++)
    {
        float stratum = float(i) / float(INITIAL_CANDIDATE_SAMPLES);
        float2 jitter = random_float2(seed);
        float2 xi = float2(
            frac(stratum + jitter.x / float(INITIAL_CANDIDATE_SAMPLES)),
            jitter.y
        );

        float pdf;
        float3 local_dir = sample_cosine_hemisphere(xi, pdf);
        float3 ray_dir   = local_to_world(local_dir, normal_ws);

        if (dot(ray_dir, normal_ws) <= 0 || pdf < RESTIR_MIN_PDF)
            continue;

        float3 ray_origin     = pos_ws + normal_ws * primary_offset;
        PathSample candidate  = trace_path(ray_origin, ray_dir, seed);
        candidate.direction   = ray_dir;
        candidate.pdf         = pdf;

        float target_pdf = calculate_target_pdf_diffuse(candidate.radiance, ray_dir, normal_ws, albedo, metallic);
        float weight     = target_pdf / max(pdf, RESTIR_MIN_PDF);

        update_reservoir(reservoir, candidate, weight, random_float(seed));
    }

    {
        float3 final_dir = is_sky_sample(reservoir.sample)
            ? reservoir.sample.direction
            : normalize(reservoir.sample.hit_position - pos_ws);

        float target = calculate_target_pdf_diffuse(reservoir.sample.radiance, final_dir, normal_ws, albedo, metallic);

        reservoir.target_pdf = target;
        if (target > 0 && reservoir.M > 0)
            reservoir.W = reservoir.weight_sum / (target * reservoir.M);
        else
            reservoir.W = 0;

        float w_clamp = get_w_clamp_for_sample(reservoir.sample);
        reservoir.W = min(reservoir.W, w_clamp);
    }

    // confidence tracks sample stability, not brightness
    float sample_count_quality = saturate(reservoir.M / float(INITIAL_CANDIDATE_SAMPLES));
    float pdf_quality          = saturate(reservoir.sample.pdf * 4.0f);
    float sample_luma          = dot(reservoir.sample.radiance, float3(0.299f, 0.587f, 0.114f));
    float low_light_factor     = saturate(1.0f - sample_luma / 0.2f);
    float confidence_base      = sample_count_quality * lerp(0.55f, 1.0f, pdf_quality);
    reservoir.confidence       = (reservoir.target_pdf > 0.0f) ? saturate(confidence_base * lerp(1.15f, 1.0f, 1.0f - low_light_factor)) : 0.0f;
    reservoir.age              = 0.0f;

    // store reservoir
    float4 t0, t1, t2, t3, t4;
    pack_reservoir(reservoir, t0, t1, t2, t3, t4);
    tex_reservoir0[launch_id] = t0;
    tex_reservoir1[launch_id] = t1;
    tex_reservoir2[launch_id] = t2;
    tex_reservoir3[launch_id] = t3;
    tex_reservoir4[launch_id] = t4;

    // pre-shaded gi = diffuse_brdf * cos * radiance * W
    float3 gi = shade_reservoir_diffuse(reservoir, pos_ws, normal_ws, albedo, metallic);
    gi = soft_clamp_gi(gi, reservoir.sample);

    tex_uav[launch_id] = float4(gi, 1.0f);
}

[shader("closesthit")]
void closest_hit(inout PathPayload payload : SV_RayPayload, in BuiltInTriangleIntersectionAttributes attribs : SV_IntersectionAttributes)
{
    payload.hit = true;

    uint material_index    = InstanceID();
    MaterialParameters mat = material_parameters[material_index];

    uint instance_index = InstanceIndex();
    GeometryInfo geo    = geometry_infos[instance_index];

    // fetch triangle indices from the global index buffer
    uint primitive_index = PrimitiveIndex();
    uint index_base      = geo.index_offset + primitive_index * 3;
    uint i0 = geometry_indices[index_base + 0];
    uint i1 = geometry_indices[index_base + 1];
    uint i2 = geometry_indices[index_base + 2];

    // fetch vertex data from the global vertex buffer
    PulledVertex pv0 = geometry_vertices[geo.vertex_offset + i0];
    PulledVertex pv1 = geometry_vertices[geo.vertex_offset + i1];
    PulledVertex pv2 = geometry_vertices[geo.vertex_offset + i2];

    // interpolate vertex attributes
    float3 bary = float3(1.0f - attribs.barycentrics.x - attribs.barycentrics.y,
                         attribs.barycentrics.x, attribs.barycentrics.y);

    float3 normal_object  = normalize(pv0.normal * bary.x + pv1.normal * bary.y + pv2.normal * bary.z);
    float3 tangent_object = normalize(pv0.tangent * bary.x + pv1.tangent * bary.y + pv2.tangent * bary.z);
    float2 texcoord       = pv0.uv * bary.x + pv1.uv * bary.y + pv2.uv * bary.z;

    // transform to world space
    float3x3 obj_to_world  = (float3x3)ObjectToWorld4x3();
    float3x3 world_to_obj  = (float3x3)WorldToObject4x3();
    float3 normal_world    = normalize(mul(normal_object, transpose(world_to_obj)));
    float3 tangent_world   = normalize(mul(tangent_object, obj_to_world));

    if (any(mat.world_space_uv))
    {
        float3 hit_position = WorldRayOrigin() + WorldRayDirection() * RayTCurrent();
        texcoord            = compute_world_space_uv(hit_position, normal_world);
        texcoord            = texcoord * mat.tiling + mat.offset;
    }
    else
    {
        texcoord = texcoord * mat.tiling + mat.offset;
    }

    if (mat.uv_rotation != 0.0f)
        texcoord = rotate_uv_90(texcoord, mat.uv_rotation);

    float dist      = RayTCurrent();
    float mip_level = clamp(log2(max(dist * 0.5f, 1.0f)), 0.0f, 4.0f);

    // sample albedo texture
    float3 albedo = mat.color.rgb;
    if (mat.has_texture_albedo())
    {
        uint albedo_texture_index = material_index + material_texture_index_albedo;
        float4 sampled = material_textures[albedo_texture_index].SampleLevel(
            GET_SAMPLER(sampler_bilinear_wrap), texcoord, mip_level);
        albedo = sampled.rgb * mat.color.rgb;
    }
    albedo = saturate(albedo);

    // sample roughness texture
    float roughness = mat.roughness;
    if (mat.has_texture_roughness())
    {
        uint roughness_texture_index = material_index + material_texture_index_roughness;
        roughness *= material_textures[roughness_texture_index].SampleLevel(
            GET_SAMPLER(sampler_bilinear_wrap), texcoord, mip_level).g;
    }
    roughness = max(roughness, 0.04f);

    // sample metalness texture
    float metallic = mat.metallness;
    if (mat.has_texture_metalness())
    {
        uint metalness_texture_index = material_index + material_texture_index_metalness;
        metallic *= material_textures[metalness_texture_index].SampleLevel(
            GET_SAMPLER(sampler_bilinear_wrap), texcoord, mip_level).r;
    }

    // apply normal mapping
    float3x3 obj_to_world_3x3 = (float3x3)ObjectToWorld4x3();
    float3 edge1_world   = mul(pv1.position - pv0.position, obj_to_world_3x3);
    float3 edge2_world   = mul(pv2.position - pv0.position, obj_to_world_3x3);
    float triangle_area  = 0.5f * length(cross(edge1_world, edge2_world));
    float3 geometric_normal = normalize(cross(edge1_world, edge2_world));

    // keep the shading normal in the same hemisphere as the traced side of the triangle.
    if (dot(geometric_normal, WorldRayDirection()) > 0.0f)
        geometric_normal = -geometric_normal;
    if (dot(normal_world, geometric_normal) < 0.0f)
        normal_world = -normal_world;

    float3 tangent_projected = tangent_world - geometric_normal * dot(tangent_world, geometric_normal);
    if (dot(tangent_projected, tangent_projected) > 1e-6f)
    {
        tangent_world = normalize(tangent_projected);
    }
    else
    {
        float3 fallback_bitangent;
        build_orthonormal_basis_fast(geometric_normal, tangent_world, fallback_bitangent);
    }

    if (mat.has_texture_normal())
    {
        uint normal_texture_index = material_index + material_texture_index_normal;
        float3 normal_sample = material_textures[normal_texture_index].SampleLevel(
            GET_SAMPLER(sampler_bilinear_wrap), texcoord, mip_level).rgb;

        normal_sample = normal_sample * 2.0f - 1.0f;
        normal_sample.xy *= mat.normal;

        float3 bitangent = normalize(cross(geometric_normal, tangent_world));
        float3x3 tbn     = float3x3(tangent_world, bitangent, geometric_normal);

        normal_world = normalize(mul(normal_sample, tbn));
        if (dot(normal_world, geometric_normal) < 0.0f)
            normal_world = -normal_world;
    }

    // sample emissive texture
    float3 emission = float3(0.0f, 0.0f, 0.0f);
    if (mat.has_texture_emissive())
    {
        uint emissive_texture_index = material_index + material_texture_index_emission;
        emission = material_textures[emissive_texture_index].SampleLevel(
            GET_SAMPLER(sampler_bilinear_wrap), texcoord, mip_level).rgb;
    }
    if (mat.emissive_from_albedo())
        emission += albedo;

    // scale to match rasterization path intensity (light_composition uses *10)
    emission *= 10.0f;

    // compute hit position and triangle area
    float3 hit_position = WorldRayOrigin() + WorldRayDirection() * dist;

    // populate payload
    payload.hit_position    = hit_position;
    payload.hit_normal      = normal_world;
    payload.geometric_normal = geometric_normal;
    payload.albedo          = albedo;
    payload.emission        = emission;
    payload.roughness       = roughness;
    payload.metallic        = metallic;
    payload.triangle_area   = triangle_area;
}

[shader("miss")]
void miss(inout PathPayload payload : SV_RayPayload)
{
    payload.hit      = false;
    payload.emission = float3(0.0f, 0.0f, 0.0f);
}
