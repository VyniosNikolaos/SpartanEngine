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

//= includes =========
#include "common.hlsl"
//====================

[numthreads(1, 1, 1)]
void main_cs(uint3 thread_id : SV_DispatchThreadID)
{
    // 1. compute average luminance (in watts for now)
    float avg_watts = 0.0f;
    {
        uint w, h, mip_count;
        tex.GetDimensions(0, w, h, mip_count);

        // we use the lowest mip (1x1 or 2x2) for performance
        uint mip = max(0, mip_count - 1); // safety check
        tex.GetDimensions(mip, w, h, mip_count);

        float sum_watts = 0.0f;
        for (uint y = 0; y < h; y++)
        {
            for (uint x = 0; x < w; x++)
            {
                float3 col = tex.Load(int3(x, y, mip)).rgb;
                
                // get luminance (watts)
                float lum = dot(col, float3(0.2126f, 0.7152f, 0.0722f));
                
                // [sun protection]
                // if we look at the sun, the average skyrockets, causing the rest of 
                // the world to become pitch black. we clamp the max sample value 
                // to prevent the sun from dominating the average.
                // 100.0f watts ~ 68,000 nits (very bright highlight, but not the sun)
                lum = min(lum, 100.0f); 

                sum_watts += lum;
            }
        }
        avg_watts = sum_watts / float(w * h);
    }

    // 2. physics conversion (watts -> nits)
    const float luminous_efficacy = 683.0f;
    float avg_nits = avg_watts * luminous_efficacy;

    // 3. auto exposure works on top of the manual camera exposure, so the target is in post-camera scene-linear space
    float camera_exposure   = max(buffer_frame.camera_exposure, 0.000001f);
    float current_brightness = avg_nits * camera_exposure;

    // 4. target a brighter average than strict middle gray
    const float target_luminance = 0.40f;

    // 5. compute the ae multiplier needed to bring the current average toward middle gray
    float desired_exposure = target_luminance / max(current_brightness, 0.000001f);

    // 6. clamp the ae multiplier to a wide but still bounded range
    const float min_ev = -12.0f; 
    const float max_ev =  12.0f;

    float min_exposure = exp2(min_ev);
    float max_exposure = exp2(max_ev);

    desired_exposure = clamp(desired_exposure, min_exposure, max_exposure);

    // 7. temporal adaptation in ev space so large changes remain stable
    float prev_exposure = tex2.Load(int3(0, 0, 0)).r;
    float adaptation_speed = pass_get_f3_value().x;
    
    // start from the current target to avoid a first-frame flash
    if (isnan(prev_exposure) || prev_exposure <= 0.0f)
    {
        prev_exposure = desired_exposure;
    }

    // higher values should adapt faster, so the response scales with the user-controlled speed directly
    float prev_ev          = log2(max(prev_exposure, 0.000001f));
    float desired_ev       = log2(max(desired_exposure, 0.000001f));
    float adaptation_alpha = 1.0f - exp(-max(adaptation_speed, 0.0f) * 4.0f * buffer_frame.delta_time);
    float exposure         = exp2(lerp(prev_ev, desired_ev, adaptation_alpha));

    // write output
    tex_uav[uint2(0, 0)] = float4(exposure, exposure, exposure, 1.0f);
}
