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

#ifndef SPARTAN_RESTIR_RESERVOIR
#define SPARTAN_RESTIR_RESERVOIR

// core parameters
static const uint RESTIR_MAX_PATH_LENGTH     = 5;
static const uint RESTIR_M_CAP               = 200;
static const uint RESTIR_SPATIAL_SAMPLES     = 16;
static const float RESTIR_SPATIAL_RADIUS     = 32.0f;
static const float RESTIR_DEPTH_THRESHOLD    = 0.05f;
static const float RESTIR_NORMAL_THRESHOLD   = 0.75f;
static const float RESTIR_TEMPORAL_DECAY     = 0.95f;
static const float RESTIR_RAY_NORMAL_OFFSET  = 0.05f;
static const float RESTIR_RAY_T_MIN          = 0.01f;

// sky/environment
static const float RESTIR_SKY_RADIANCE_CLAMP = 5.0f;
static const float RESTIR_SKY_W_CLAMP        = 15.0f;
static const float RESTIR_SKY_DIR_THRESHOLD  = 0.95f;
static const float RESTIR_ENV_SAMPLE_PROB    = 0.3f;
static const float RESTIR_SKY_DISTANCE       = 1e10f;

// visibility/geometry
static const float RESTIR_VIS_COS_FRONT      = 0.1f;
static const float RESTIR_VIS_COS_BACK       = 0.05f;
static const float RESTIR_VIS_MIN_DIST       = 0.02f;
static const float RESTIR_VIS_PLANE_MIN      = 0.001f;

// BRDF thresholds
static const float RESTIR_MIN_ROUGHNESS      = 0.04f;
static const float RESTIR_MIN_PDF            = 1e-6f;
static const float RESTIR_W_CLAMP_DEFAULT    = 50.0f;
static const float RESTIR_SPECULAR_THRESHOLD = 0.2f;
static const float RESTIR_MATERIAL_ROUGHNESS_THRESHOLD = 0.2f;
static const float RESTIR_MATERIAL_METALLIC_THRESHOLD  = 0.25f;
static const float RESTIR_MATERIAL_ALBEDO_THRESHOLD    = 0.35f;
static const float RESTIR_MATERIAL_SIMILARITY_MIN      = 0.25f;

// firefly suppression
static const float RESTIR_SOFT_CLAMP_SKY     = 10.0f;
static const float RESTIR_SOFT_CLAMP_DEFAULT = 12.0f;

struct PathSample
{
    float3 hit_position;
    float3 hit_normal;
    float3 direction;
    float3 radiance;
    uint   path_length;
    uint   flags;
    float  pdf;
};

struct Reservoir
{
    PathSample sample;
    float      weight_sum;
    float      M;
    float      W;
    float      target_pdf;
    float      age;
    float      confidence;
};

static const uint PATH_FLAG_SPECULAR = 1 << 0;
static const uint PATH_FLAG_DIFFUSE  = 1 << 1;
static const uint PATH_FLAG_CAUSTIC  = 1 << 2;
static const uint PATH_FLAG_DELTA    = 1 << 3;
static const uint PATH_FLAG_SKY      = 1 << 4;

static const uint RESTIR_DEBUG_MODE_NONE                  = 0;
static const uint RESTIR_DEBUG_MODE_CONFIDENCE            = 1;
static const uint RESTIR_DEBUG_MODE_M                     = 2;
static const uint RESTIR_DEBUG_MODE_W                     = 3;
static const uint RESTIR_DEBUG_MODE_REUSE                 = 4;
static const uint RESTIR_DEBUG_MODE_TEMPORAL_REJECTION    = 5;

static const uint RESTIR_TEMPORAL_REASON_ACCEPTED             = 0;
static const uint RESTIR_TEMPORAL_REASON_INVALID_UV           = 1;
static const uint RESTIR_TEMPORAL_REASON_REPROJECTION         = 2;
static const uint RESTIR_TEMPORAL_REASON_NORMAL               = 3;
static const uint RESTIR_TEMPORAL_REASON_MATERIAL             = 4;
static const uint RESTIR_TEMPORAL_REASON_LOW_CONFIDENCE       = 5;
static const uint RESTIR_TEMPORAL_REASON_OUT_OF_BOUNDS        = 6;
static const uint RESTIR_TEMPORAL_REASON_INVALID_HISTORY      = 7;
static const uint RESTIR_TEMPORAL_REASON_EMPTY_HISTORY        = 8;
static const uint RESTIR_TEMPORAL_REASON_VISIBILITY           = 9;
static const uint RESTIR_TEMPORAL_REASON_JACOBIAN            = 10;
static const uint RESTIR_TEMPORAL_REASON_TARGET_PDF          = 11;

float2 octahedral_encode(float3 n)
{
    n /= (abs(n.x) + abs(n.y) + abs(n.z));
    if (n.z < 0.0f)
    {
        float2 sign_not_zero = float2(n.x >= 0.0f ? 1.0f : -1.0f, n.y >= 0.0f ? 1.0f : -1.0f);
        n.xy = (1.0f - abs(n.yx)) * sign_not_zero;
    }
    return n.xy;
}

float3 octahedral_decode(float2 e)
{
    float3 n = float3(e.xy, 1.0f - abs(e.x) - abs(e.y));
    if (n.z < 0.0f)
    {
        float2 sign_not_zero = float2(n.x >= 0.0f ? 1.0f : -1.0f, n.y >= 0.0f ? 1.0f : -1.0f);
        n.xy = (1.0f - abs(n.yx)) * sign_not_zero;
    }
    return normalize(n);
}

uint pack_path_info(uint path_length, uint flags)
{
    return (path_length & 0xFFFF) | ((flags & 0xFFFF) << 16);
}

void unpack_path_info(uint packed, out uint path_length, out uint flags)
{
    path_length = packed & 0xFFFF;
    flags = (packed >> 16) & 0xFFFF;
}

void pack_reservoir(Reservoir r, out float4 tex0, out float4 tex1, out float4 tex2, out float4 tex3, out float4 tex4)
{
    float2 normal_oct    = octahedral_encode(r.sample.hit_normal);
    float2 direction_oct = octahedral_encode(r.sample.direction);

    tex0 = float4(r.sample.hit_position, normal_oct.x);
    tex1 = float4(normal_oct.y, direction_oct.xy, r.sample.radiance.x);
    tex2 = float4(r.sample.radiance.yz, r.sample.pdf, r.weight_sum);
    tex3 = float4(r.M, r.W, r.target_pdf, asfloat(pack_path_info(r.sample.path_length, r.sample.flags)));
    tex4 = float4(r.age, r.confidence, 0, 0);
}

Reservoir unpack_reservoir(float4 tex0, float4 tex1, float4 tex2, float4 tex3, float4 tex4)
{
    Reservoir r;
    r.sample.hit_position = tex0.xyz;
    r.sample.hit_normal   = octahedral_decode(float2(tex0.w, tex1.x));
    r.sample.direction    = octahedral_decode(tex1.yz);
    r.sample.radiance     = float3(tex1.w, tex2.xy);
    r.sample.pdf          = tex2.z;
    r.weight_sum          = tex2.w;
    r.M                   = tex3.x;
    r.W                   = tex3.y;
    r.target_pdf          = tex3.z;
    r.age                 = tex4.x;
    r.confidence          = tex4.y;

    uint packed_info = asuint(tex3.w);
    unpack_path_info(packed_info, r.sample.path_length, r.sample.flags);

    return r;
}

bool is_reservoir_valid(Reservoir r)
{
    if (any(isnan(r.sample.hit_position)) || any(isinf(r.sample.hit_position)))
        return false;
    if (any(isnan(r.sample.radiance)) || any(isinf(r.sample.radiance)))
        return false;
    if (any(isnan(r.sample.hit_normal)) || any(isinf(r.sample.hit_normal)))
        return false;
    if (any(isnan(r.sample.direction)) || any(isinf(r.sample.direction)))
        return false;
    if (isnan(r.weight_sum) || isinf(r.weight_sum) || r.weight_sum < 0)
        return false;
    if (isnan(r.W) || isinf(r.W) || r.W < 0)
        return false;
    if (isnan(r.M) || r.M < 0)
        return false;
    if (isnan(r.target_pdf) || isinf(r.target_pdf) || r.target_pdf < 0)
        return false;
    if (isnan(r.age) || isinf(r.age) || r.age < 0)
        return false;
    if (isnan(r.confidence) || isinf(r.confidence))
        return false;

    float normal_len = length(r.sample.hit_normal);
    if (normal_len < 0.5f || normal_len > 1.5f)
        return false;

    return true;
}

Reservoir create_empty_reservoir()
{
    Reservoir r;
    r.sample.hit_position = float3(0, 0, 0);
    r.sample.hit_normal   = float3(0, 1, 0);
    r.sample.direction    = float3(0, 0, 1);
    r.sample.radiance     = float3(0, 0, 0);
    r.sample.path_length  = 0;
    r.sample.flags        = 0;
    r.sample.pdf          = 0;
    r.weight_sum          = 0;
    r.M                   = 0;
    r.W                   = 0;
    r.target_pdf          = 0;
    r.age                 = 0;
    r.confidence          = 0;
    return r;
}

float calculate_target_pdf(float3 radiance)
{
    float lum = dot(radiance, float3(0.299, 0.587, 0.114));
    return max(lum, 1e-6f);
}

float calculate_target_pdf_with_brdf(float3 radiance, float3 sample_dir, float3 shading_normal, float3 view_dir,
                                      float3 albedo, float roughness, float metallic)
{
    float base_target = calculate_target_pdf(radiance);

    float n_dot_l = dot(shading_normal, sample_dir);
    if (n_dot_l <= 0.0f)
        return 0.0f;

    float3 h      = normalize(view_dir + sample_dir);
    float n_dot_v = max(dot(shading_normal, view_dir), 0.001f);
    float n_dot_h = max(dot(shading_normal, h), 0.0f);
    float v_dot_h = max(dot(view_dir, h), 0.0f);

    float3 diffuse_response = albedo * (1.0f - metallic) * n_dot_l;

    float alpha   = max(roughness * roughness, 0.001f);
    float alpha2  = alpha * alpha;
    float d_denom = n_dot_h * n_dot_h * (alpha2 - 1.0f) + 1.0f;
    float d       = alpha2 / (3.14159265f * d_denom * d_denom + 1e-6f);

    float3 f0 = lerp(float3(0.04f, 0.04f, 0.04f), albedo, metallic);
    float3 f  = f0 + (1.0f - f0) * pow(1.0f - v_dot_h, 5.0f);

    float spec_weight = d * dot(f, float3(0.299, 0.587, 0.114)) * n_dot_l;
    float diff_weight = dot(diffuse_response, float3(0.299, 0.587, 0.114));

    float brdf_weight = clamp(diff_weight + spec_weight, 0.0f, 10.0f);

    return base_target * max(brdf_weight, 0.01f);
}

float calculate_target_pdf_sky(float3 radiance, float3 sample_dir, float3 shading_normal, float3 view_dir,
                                float3 albedo, float roughness, float metallic)
{
    float base_target = calculate_target_pdf(radiance);

    float n_dot_l = dot(shading_normal, sample_dir);
    if (n_dot_l <= 0.0f)
        return 0.0f;

    float3 h      = normalize(view_dir + sample_dir);
    float n_dot_v = max(dot(shading_normal, view_dir), 0.001f);
    float n_dot_h = max(dot(shading_normal, h), 0.0f);
    float v_dot_h = max(dot(view_dir, h), 0.0f);

    float3 diffuse_response = albedo * (1.0f - metallic) * n_dot_l;

    float alpha   = max(roughness * roughness, 0.001f);
    float alpha2  = alpha * alpha;
    float d_denom = n_dot_h * n_dot_h * (alpha2 - 1.0f) + 1.0f;
    float d       = alpha2 / (3.14159265f * d_denom * d_denom + 1e-6f);

    float3 f0 = lerp(float3(0.04f, 0.04f, 0.04f), albedo, metallic);
    float3 f  = f0 + (1.0f - f0) * pow(1.0f - v_dot_h, 5.0f);

    float spec_weight = d * dot(f, float3(0.299, 0.587, 0.114)) * n_dot_l;
    float diff_weight = dot(diffuse_response, float3(0.299, 0.587, 0.114));

    float brdf_weight = clamp(diff_weight + spec_weight, 0.0f, 10.0f);

    return base_target * max(brdf_weight, 0.01f);
}

float calculate_target_pdf_with_geometry(float3 radiance, float3 shading_pos, float3 shading_normal, float3 view_dir,
                                          float3 sample_hit_pos, float3 sample_hit_normal,
                                          float3 albedo, float roughness, float metallic)
{
    float3 to_sample = sample_hit_pos - shading_pos;
    float dist_sq = dot(to_sample, to_sample);
    if (dist_sq < 1e-6f)
        return 0.0f;

    float dist = sqrt(dist_sq);
    float3 sample_dir = to_sample / dist;

    // the radiance stored in the sample already accounts for the geometry term (1/r^2, cosines)
    // from the path tracer, so we only need the BRDF-weighted luminance as the target PDF
    // including a geometry term here would double-count it and cause corners to be overbright
    return calculate_target_pdf_with_brdf(radiance, sample_dir, shading_normal, view_dir,
                                           albedo, roughness, metallic);
}

bool is_sky_sample(PathSample s)
{
    return (s.flags & PATH_FLAG_SKY) != 0;
}

bool has_path_sample(PathSample sample)
{
    return is_sky_sample(sample) || sample.path_length > 0 || any(sample.radiance > 0.0f);
}

float calculate_target_pdf_for_sample(PathSample sample, float3 shading_pos, float3 shading_normal, float3 view_dir,
                                      float3 albedo, float roughness, float metallic)
{
    float target_pdf = 0.0f;

    if (is_sky_sample(sample))
    {
        target_pdf = calculate_target_pdf_sky(sample.radiance, sample.direction, shading_normal, view_dir, albedo, roughness, metallic);
    }
    else if (has_path_sample(sample))
    {
        target_pdf = calculate_target_pdf_with_geometry(sample.radiance, shading_pos, shading_normal, view_dir, sample.hit_position, sample.hit_normal,
                                                        albedo, roughness, metallic);
    }

    if (target_pdf <= 0.0f)
        target_pdf = calculate_target_pdf(sample.radiance);

    return target_pdf;
}

float compute_reservoir_stream_weight(float target_pdf, float W, float M)
{
    if (target_pdf <= 0.0f || W <= 0.0f || M <= 0.0f)
        return 0.0f;

    return target_pdf * W * M;
}

float get_w_clamp_for_sample(PathSample s);

void finalize_reservoir_with_target(inout Reservoir reservoir, float target_pdf)
{
    reservoir.target_pdf = max(target_pdf, 0.0f);

    if (reservoir.target_pdf > 0.0f && reservoir.M > 0.0f)
        reservoir.W = reservoir.weight_sum / max(reservoir.target_pdf * reservoir.M, RESTIR_MIN_PDF);
    else
        reservoir.W = 0.0f;

    float w_clamp = get_w_clamp_for_sample(reservoir.sample);
    reservoir.W = min(reservoir.W, w_clamp);
}

float compute_material_similarity(float3 albedo_a, float roughness_a, float metallic_a, float3 albedo_b, float roughness_b, float metallic_b)
{
    float roughness_similarity = saturate(1.0f - abs(roughness_a - roughness_b) / RESTIR_MATERIAL_ROUGHNESS_THRESHOLD);
    float metallic_similarity  = saturate(1.0f - abs(metallic_a - metallic_b) / RESTIR_MATERIAL_METALLIC_THRESHOLD);
    float3 albedo_delta        = abs(albedo_a - albedo_b);
    float albedo_max_delta     = max(albedo_delta.r, max(albedo_delta.g, albedo_delta.b));
    float albedo_similarity    = saturate(1.0f - albedo_max_delta / RESTIR_MATERIAL_ALBEDO_THRESHOLD);

    return roughness_similarity * metallic_similarity * albedo_similarity;
}

bool are_materials_compatible(float3 albedo_a, float roughness_a, float metallic_a, float3 albedo_b, float roughness_b, float metallic_b)
{
    return compute_material_similarity(albedo_a, roughness_a, metallic_a, albedo_b, roughness_b, metallic_b) >= RESTIR_MATERIAL_SIMILARITY_MIN;
}

uint get_restir_pt_debug_mode()
{
    return (uint)buffer_frame.restir_pt_debug_mode;
}

float compute_sky_direction_similarity(float3 dir1, float3 dir2)
{
    float cos_angle = dot(dir1, dir2);
    if (cos_angle < RESTIR_SKY_DIR_THRESHOLD)
        return 0.0f;
    return saturate((cos_angle - RESTIR_SKY_DIR_THRESHOLD) / (1.0f - RESTIR_SKY_DIR_THRESHOLD));
}

float get_w_clamp_for_sample(PathSample s)
{
    if (is_sky_sample(s))
        return RESTIR_SKY_W_CLAMP;
    return RESTIR_W_CLAMP_DEFAULT;
}

bool update_reservoir(inout Reservoir reservoir, PathSample new_sample, float weight, float random_value)
{
    reservoir.weight_sum += weight;
    reservoir.M += 1.0f;

    if (random_value * reservoir.weight_sum < weight)
    {
        reservoir.sample = new_sample;
        reservoir.age    = 0.0f;
        return true;
    }
    return false;
}

bool merge_reservoir(inout Reservoir dst, Reservoir src, float target_pdf_at_dst, float random_value)
{
    float weight = compute_reservoir_stream_weight(target_pdf_at_dst, src.W, src.M);

    dst.weight_sum += weight;
    dst.M += src.M;

    if (random_value * dst.weight_sum < weight)
    {
        dst.sample     = src.sample;
        dst.target_pdf = target_pdf_at_dst;
        dst.age        = src.age;
        return true;
    }
    return false;
}

void finalize_reservoir(inout Reservoir reservoir)
{
    finalize_reservoir_with_target(reservoir, calculate_target_pdf(reservoir.sample.radiance));
}

void clamp_reservoir_M(inout Reservoir reservoir, float max_M)
{
    if (reservoir.M > max_M)
    {
        float scale = max_M / reservoir.M;
        reservoir.weight_sum *= scale;
        reservoir.M = max_M;
        finalize_reservoir_with_target(reservoir, reservoir.target_pdf);
    }
}

uint pcg_hash(uint seed)
{
    uint state = seed * 747796405u + 2891336453u;
    uint word  = ((state >> ((state >> 28u) + 4u)) ^ state) * 277803737u;
    return (word >> 22u) ^ word;
}

uint xxhash32(uint seed)
{
    const uint PRIME1 = 2654435761u;
    const uint PRIME2 = 2246822519u;
    const uint PRIME3 = 3266489917u;

    uint h = seed + PRIME3;
    h = (h ^ (h >> 15)) * PRIME2;
    h = (h ^ (h >> 13)) * PRIME3;
    return h ^ (h >> 16);
}

float random_float(inout uint seed)
{
    seed = pcg_hash(seed);
    return float(seed) / 4294967295.0f;
}

float2 random_float2(inout uint seed)
{
    return float2(random_float(seed), random_float(seed));
}

float3 random_float3(inout uint seed)
{
    return float3(random_float(seed), random_float(seed), random_float(seed));
}

uint create_seed(uint2 pixel, uint frame)
{
    uint h = xxhash32(pixel.x);
    h = pcg_hash(h ^ xxhash32(pixel.y));
    h = pcg_hash(h ^ xxhash32(frame));
    return h;
}

uint create_seed_for_pass(uint2 pixel, uint frame, uint pass_id)
{
    const uint GOLDEN_RATIO = 0x9E3779B9u;
    const uint PASS_PRIMES[4] = { 0x85EBCA77u, 0xC2B2AE3Du, 0x27D4EB2Fu, 0x165667B1u };

    uint pass_salt = (pass_id < 4) ? PASS_PRIMES[pass_id] : xxhash32(pass_id * GOLDEN_RATIO);

    uint h = xxhash32(pixel.x ^ pass_salt);
    h = pcg_hash(h ^ xxhash32(pixel.y));
    h = pcg_hash(h ^ xxhash32(frame ^ (pass_salt >> 16)));
    return h;
}

float3 sample_cosine_hemisphere(float2 xi, out float pdf)
{
    float phi       = 2.0f * 3.14159265f * xi.x;
    float cos_theta = sqrt(xi.y);
    float sin_theta = sqrt(1.0f - xi.y);

    pdf = cos_theta / 3.14159265f;
    return float3(cos(phi) * sin_theta, sin(phi) * sin_theta, cos_theta);
}

float3 sample_ggx(float2 xi, float roughness, out float pdf)
{
    float a  = roughness * roughness;
    float a2 = a * a;

    float phi       = 2.0f * 3.14159265f * xi.x;
    float cos_theta = sqrt((1.0f - xi.y) / (1.0f + (a2 - 1.0f) * xi.y));
    float sin_theta = sqrt(1.0f - cos_theta * cos_theta);

    float3 h = float3(cos(phi) * sin_theta, sin(phi) * sin_theta, cos_theta);

    float d = (a2 - 1.0f) * cos_theta * cos_theta + 1.0f;
    pdf = a2 * cos_theta / (3.14159265f * d * d);

    return h;
}

void build_orthonormal_basis_fast(float3 n, out float3 t, out float3 b)
{
    if (n.z < -0.9999999f)
    {
        t = float3(0, -1, 0);
        b = float3(-1, 0, 0);
    }
    else
    {
        float a = 1.0f / (1.0f + n.z);
        float d = -n.x * n.y * a;
        t = float3(1.0f - n.x * n.x * a, d, -n.x);
        b = float3(d, 1.0f - n.y * n.y * a, -n.y);
    }
}

float3 local_to_world(float3 local_dir, float3 n)
{
    float3 t, b;
    build_orthonormal_basis_fast(n, t, b);
    return normalize(t * local_dir.x + b * local_dir.y + n * local_dir.z);
}

bool surface_similarity_check(float3 pos1, float3 normal1, float depth1, float3 pos2, float3 normal2, float depth2)
{
    if (dot(normal1, normal2) < RESTIR_NORMAL_THRESHOLD)
        return false;

    float depth_ratio = depth1 / max(depth2, 1e-6f);
    if (abs(depth_ratio - 1.0f) > RESTIR_DEPTH_THRESHOLD)
        return false;

    return true;
}

float compute_jacobian(float3 sample_pos, float3 original_shading_pos, float3 new_shading_pos, float3 sample_normal, float3 new_receiver_normal)
{
    float3 dir_original = sample_pos - original_shading_pos;
    float3 dir_new      = sample_pos - new_shading_pos;

    float dist_original_sq = dot(dir_original, dir_original);
    float dist_new_sq      = dot(dir_new, dir_new);

    static const float MIN_DIST_SQ = 0.01f;
    if (dist_original_sq < MIN_DIST_SQ || dist_new_sq < MIN_DIST_SQ)
        return 0.0f;

    float dist_original = sqrt(dist_original_sq);
    float dist_new      = sqrt(dist_new_sq);

    dir_original /= dist_original;
    dir_new      /= dist_new;

    float cos_at_receiver = dot(new_receiver_normal, dir_new);
    if (cos_at_receiver < 0.05f)
        return 0.0f;

    float cos_original = dot(sample_normal, -dir_original);
    float cos_new      = dot(sample_normal, -dir_new);

    static const float MIN_COS_ANGLE = 0.05f;
    if (cos_original < MIN_COS_ANGLE || cos_new < MIN_COS_ANGLE)
        return 0.0f;

    // tighten similarity for large distances to prevent artifacts on flat surfaces
    float avg_dist = (dist_original + dist_new) * 0.5f;
    if (avg_dist > 50.0f)
    {
        float direction_similarity = dot(dir_original, dir_new);
        float min_similarity = lerp(0.95f, 0.99f, saturate((avg_dist - 50.0f) / 150.0f));
        if (direction_similarity < min_similarity)
            return 0.0f;
    }

    float jacobian = (cos_new * dist_original_sq) / max(cos_original * dist_new_sq, 1e-4f);

    return clamp(jacobian, 0.0f, 10.0f);
}

float power_heuristic(float pdf_a, float pdf_b)
{
    float a2 = pdf_a * pdf_a;
    float b2 = pdf_b * pdf_b;
    return a2 / max(a2 + b2, 1e-6f);
}

float compute_environment_pdf(float3 direction)
{
    float cos_theta = abs(direction.y);
    return max(cos_theta / PI, RESTIR_MIN_PDF);
}

float3 sample_environment_direction(float2 xi, out float pdf)
{
    float phi       = 2.0f * PI * xi.x;
    float cos_theta = sqrt(xi.y);
    float sin_theta = sqrt(1.0f - xi.y);

    float3 dir = float3(cos(phi) * sin_theta, cos_theta, sin(phi) * sin_theta);

    pdf = max(cos_theta / PI, RESTIR_MIN_PDF);
    return dir;
}

float3 sample_environment_direction_uniform(float2 xi, out float pdf)
{
    float z   = 1.0f - 2.0f * xi.x;
    float r   = sqrt(max(0.0f, 1.0f - z * z));
    float phi = 2.0f * PI * xi.y;

    pdf = 1.0f / (4.0f * PI);
    return float3(r * cos(phi), r * sin(phi), z);
}

float3 clamp_sky_radiance(float3 radiance)
{
    float lum = dot(radiance, float3(0.299f, 0.587f, 0.114f));
    if (lum > RESTIR_SKY_RADIANCE_CLAMP)
        radiance *= RESTIR_SKY_RADIANCE_CLAMP / lum;
    return radiance;
}

float3 soft_clamp_gi(float3 gi, PathSample sample)
{
    if (any(isnan(gi)) || any(isinf(gi)))
        return float3(0.0f, 0.0f, 0.0f);

    float lum = dot(gi, float3(0.299f, 0.587f, 0.114f));

    float soft_clamp = is_sky_sample(sample) ? RESTIR_SOFT_CLAMP_SKY : RESTIR_SOFT_CLAMP_DEFAULT;

    if (lum > soft_clamp)
    {
        float excess = lum - soft_clamp;
        float scale  = soft_clamp + excess / (1.0f + excess / soft_clamp);
        gi *= scale / lum;
    }

    // hard clamp
    float max_lum = 100.0f;
    float final_lum = dot(gi, float3(0.299f, 0.587f, 0.114f));
    if (final_lum > max_lum)
        gi *= max_lum / final_lum;

    return gi;
}

float3 restir_debug_heat(float value)
{
    float x = saturate(value);
    return saturate(float3(
        1.5f * x,
        1.5f * (1.0f - abs(x - 0.5f) * 2.0f),
        1.5f * (1.0f - x)
    ));
}

float3 get_restir_debug_visualization(Reservoir reservoir, float reuse_ratio)
{
    uint debug_mode = get_restir_pt_debug_mode();

    if (debug_mode == RESTIR_DEBUG_MODE_CONFIDENCE)
        return reservoir.confidence.xxx;

    if (debug_mode == RESTIR_DEBUG_MODE_M)
        return restir_debug_heat(reservoir.M / RESTIR_M_CAP);

    if (debug_mode == RESTIR_DEBUG_MODE_W)
        return restir_debug_heat(reservoir.W / get_w_clamp_for_sample(reservoir.sample));

    if (debug_mode == RESTIR_DEBUG_MODE_REUSE)
        return float3(1.0f - reuse_ratio, reuse_ratio, 0.0f);

    return soft_clamp_gi(reservoir.sample.radiance * reservoir.W, reservoir.sample);
}

float3 get_temporal_rejection_debug_visualization(uint rejection_reason)
{
    switch (rejection_reason)
    {
        case RESTIR_TEMPORAL_REASON_ACCEPTED:        return float3(0.0f, 1.0f, 0.0f);
        case RESTIR_TEMPORAL_REASON_INVALID_UV:      return float3(0.25f, 0.0f, 0.0f);
        case RESTIR_TEMPORAL_REASON_REPROJECTION:    return float3(1.0f, 0.0f, 1.0f);
        case RESTIR_TEMPORAL_REASON_NORMAL:          return float3(1.0f, 1.0f, 0.0f);
        case RESTIR_TEMPORAL_REASON_MATERIAL:        return float3(0.0f, 1.0f, 1.0f);
        case RESTIR_TEMPORAL_REASON_LOW_CONFIDENCE:  return float3(1.0f, 0.5f, 0.0f);
        case RESTIR_TEMPORAL_REASON_OUT_OF_BOUNDS:   return float3(0.0f, 0.0f, 1.0f);
        case RESTIR_TEMPORAL_REASON_INVALID_HISTORY: return float3(0.6f, 0.0f, 0.8f);
        case RESTIR_TEMPORAL_REASON_EMPTY_HISTORY:   return float3(0.3f, 0.3f, 0.3f);
        case RESTIR_TEMPORAL_REASON_VISIBILITY:      return float3(1.0f, 0.0f, 0.0f);
        case RESTIR_TEMPORAL_REASON_JACOBIAN:        return float3(1.0f, 1.0f, 1.0f);
        case RESTIR_TEMPORAL_REASON_TARGET_PDF:      return float3(0.0f, 0.4f, 0.9f);
        default:                                     return float3(0.0f, 0.0f, 0.0f);
    }
}

#endif // SPARTAN_RESTIR_RESERVOIR
