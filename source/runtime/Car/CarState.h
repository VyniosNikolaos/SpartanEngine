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

//= INCLUDES ===============================
#pragma once

#ifdef DEBUG
    #define _DEBUG 1
    #undef NDEBUG
#else
    #define NDEBUG 1
    #undef _DEBUG
#endif
#define PX_PHYSX_STATIC_LIB
#include <physx/PxPhysicsAPI.h>
#include <vector>
#include <cstring>
#include "../Logging/Log.h"
#include "../Core/Engine.h"
#include "CarPresets.h"
//==========================================

// vehicle dynamics state: structs, globals, telemetry, debug helpers.
// this is the shared substrate every other car sim header pulls in; it owns no physics
// logic of its own - that lives in CarPacejka / CarAero / CarSuspension / CarDrivetrain /
// CarTires / CarSimulation.

namespace car
{
    using namespace physx;

    //= tuning namespace ===========================================================

    // swap active car spec at runtime
    inline void load_car(const car_preset& new_spec);

    namespace tuning
    {
        // active car specification - swap this to change the car
        inline car_preset spec = laferrari_preset();

        // simulation-level parameters (not part of car spec)
        constexpr float air_density                  = 1.225f;
        constexpr float road_bump_amplitude          = 0.002f;
        constexpr float road_bump_frequency          = 0.5f;
        constexpr float surface_friction_asphalt     = 1.0f;
        constexpr float surface_friction_concrete    = 0.95f;
        constexpr float surface_friction_wet_asphalt = 0.7f;
        constexpr float surface_friction_gravel      = 0.6f;
        constexpr float surface_friction_grass       = 0.4f;
        constexpr float surface_friction_ice         = 0.1f;

        // debug
        inline bool draw_raycasts   = true;
        inline bool draw_suspension = true;
        inline bool log_pacejka     = false;
        inline bool log_telemetry   = false;
        inline bool log_to_file     = false;
    }

    inline void load_car(const car_preset& new_spec) { tuning::spec = new_spec; }

    struct aero_debug_data
    {
        PxVec3 position         = PxVec3(0);
        PxVec3 velocity         = PxVec3(0);
        PxVec3 drag_force       = PxVec3(0);
        PxVec3 front_downforce  = PxVec3(0);
        PxVec3 rear_downforce   = PxVec3(0);
        PxVec3 side_force       = PxVec3(0);
        PxVec3 front_aero_pos   = PxVec3(0);
        PxVec3 rear_aero_pos    = PxVec3(0);
        float  ride_height      = 0.0f;
        float  yaw_angle        = 0.0f;
        float  ground_effect_factor = 1.0f;
        bool   valid            = false;
    };
    inline static aero_debug_data aero_debug;

    // stored shape data for visualization (2D projections of convex hull)
    struct shape_2d
    {
        std::vector<std::pair<float, float>> side_profile;   // (z, y) points for side view
        std::vector<std::pair<float, float>> front_profile;  // (x, y) points for front view
        float min_x = 0, max_x = 0;
        float min_y = 0, max_y = 0;
        float min_z = 0, max_z = 0;
        bool valid = false;
    };

    // function-local static for odr safety
    inline shape_2d& shape_data_ref()
    {
        static shape_2d instance;
        return instance;
    }

    enum wheel_id { front_left = 0, front_right = 1, rear_left = 2, rear_right = 3, wheel_count = 4 };
    inline constexpr const char* wheel_names[] = { "FL", "FR", "RL", "RR" };
    enum surface_type { surface_asphalt = 0, surface_concrete, surface_wet_asphalt, surface_gravel, surface_grass, surface_ice, surface_count };

    struct config
    {
        float length              = 4.5f;
        float width               = 2.0f;
        float height              = 0.5f;
        float mass                = 1500.0f;
        float front_wheel_radius  = 0.34f;
        float rear_wheel_radius   = 0.35f;
        float front_wheel_width   = 0.245f;
        float rear_wheel_width    = 0.305f;
        float wheel_mass          = 20.0f;
        float suspension_travel   = 0.20f;
        float suspension_height   = 0.35f;

        // cached geometry filled by compute_constants, source of truth for all consumers
        float wheelbase           = 0.0f; // z distance between axles
        float track_front         = 0.0f; // x distance between front wheels
        float track_rear          = 0.0f; // x distance between rear wheels

        float wheel_radius_for(int i) const { return (i == front_left || i == front_right) ? front_wheel_radius : rear_wheel_radius; }
        float wheel_width_for(int i) const  { return (i == front_left || i == front_right) ? front_wheel_width  : rear_wheel_width;  }
    };

    // 3-zone surface temperature + core temperature
    struct tire_thermal
    {
        float surface[3] = { 50.0f, 50.0f, 50.0f }; // inside, middle, outside
        float core        = 50.0f;

        float avg_surface() const { return (surface[0] + surface[1] + surface[2]) / 3.0f; }
    };

    struct wheel
    {
        float        compression          = 0.0f;
        float        target_compression   = 0.0f;
        float        prev_compression     = 0.0f;
        float        compression_velocity = 0.0f;
        bool         grounded             = false;
        PxVec3       contact_point        = PxVec3(0);
        PxVec3       contact_normal       = PxVec3(0, 1, 0);
        float        angular_velocity     = 0.0f;
        float        rotation             = 0.0f;
        float        tire_load            = 0.0f;
        float        slip_angle           = 0.0f;
        float        slip_ratio           = 0.0f;
        float        lateral_force        = 0.0f;
        float        longitudinal_force   = 0.0f;
        float        net_torque           = 0.0f;
        tire_thermal thermal;
        float        brake_temp           = tuning::spec.brake_ambient_temp;
        float        wear                 = 0.0f;
        surface_type contact_surface      = surface_asphalt;
    };

    struct input_state
    {
        float throttle  = 0.0f;
        float brake     = 0.0f;
        float steering  = 0.0f;
        float handbrake = 0.0f;
    };

    inline static PxRigidDynamic* body             = nullptr;
    inline static PxMaterial*     material         = nullptr;
    inline static PxConvexMesh*   wheel_sweep_mesh = nullptr;
    inline static config          cfg;
    inline static wheel           wheels[wheel_count];
    inline static input_state     input;
    inline static input_state     input_target;
    inline static PxVec3          wheel_offsets[wheel_count];
    inline static float           wheel_moi[wheel_count];
    inline static float           spring_stiffness[wheel_count];
    inline static float           spring_damping[wheel_count];
    inline static float           abs_phase               = 0.0f;
    inline static bool            abs_active[wheel_count] = {};
    inline static float           tc_reduction            = 0.0f;
    inline static bool            tc_active               = false;
    inline static float           engine_rpm              = tuning::spec.engine_idle_rpm;
    // gear encoding: 0 = reverse, 1 = neutral (gear_ratios[1] == 0 acts as the sentinel),
    // 2..8 = forward gears. use the helpers below rather than literal-comparing the index.
    inline static int             current_gear            = 2;

    inline bool is_in_reverse()      { return current_gear == 0; }
    inline bool is_in_neutral()      { return current_gear == 1; }
    inline bool is_in_forward_gear() { return current_gear >= 2; }
    inline static float           shift_timer             = 0.0f;
    inline static bool            is_shifting             = false;
    inline static float           clutch                  = 1.0f;
    inline static float           shift_cooldown          = 0.0f;
    // cooldown after a shift completes before the next auto-shift can occur
    inline static constexpr float shift_cooldown_time     = 0.5f;
    // per-wheel chassis reaction force cap from the suspension, expressed in g's
    inline static constexpr float chassis_force_cap_g     = 6.0f;
    // "large" static friction gain applied under the low-slip static-friction model,
    // units are (N per m/s) per kg of chassis mass
    inline static constexpr float static_friction_gain_per_kg = 10.0f;
    inline static int             last_shift_direction    = 0;
    inline static float           redline_hold_timer      = 0.0f;
    inline static float           boost_pressure          = 0.0f;
    inline static bool            rev_limiter_active      = false;
    inline static float           downshift_blip_timer    = 0.0f;
    inline static float           driveshaft_twist        = 0.0f;
    inline static bool            drs_active              = false;
    inline static float           longitudinal_accel      = 0.0f;
    inline static float           lateral_accel           = 0.0f;
    inline static float           road_bump_phase         = 0.0f;
    inline static PxVec3          prev_velocity           = PxVec3(0);

    // telemetry: writes a per-tick csv of the rear-wheel state + gearbox. opens the file
    // lazily, closes it when tuning::log_to_file is turned off, and flushes periodically.
    struct telemetry_dump
    {
        FILE* file          = nullptr;
        int   frame_counter = 0;

        ~telemetry_dump()      { close(); }
        void close()
        {
            if (file)
            {
                fclose(file);
                file = nullptr;
            }
            frame_counter = 0;
        }

        bool open_if_needed()
        {
            if (file) return true;
            fopen_s(&file, "car_telemetry.csv", "w");
            if (!file) return false;
            fprintf(file,
                "frame,dt,"
                "engine_rpm,speed_kmh,forward_speed_ms,"
                "gear,is_shifting,shift_timer,shift_cooldown,"
                "clutch,throttle,brake,"
                "rl_ang_vel,rr_ang_vel,rl_slip_ratio,rr_slip_ratio,"
                "rl_tire_load,rr_tire_load,rl_long_force,rr_long_force,"
                "rl_grounded,rr_grounded,"
                "tc_active,tc_reduction\n");
            frame_counter = 0;
            return true;
        }

        void tick(float dt, float speed_kmh)
        {
            if (!tuning::log_to_file)
            {
                close();
                return;
            }
            if (!open_if_needed()) return;

            float fwd_speed = body->getLinearVelocity().dot(body->getGlobalPose().q.rotate(PxVec3(0, 0, 1)));
            fprintf(file,
                "%d,%.4f,"
                "%.1f,%.2f,%.3f,"
                "%d,%d,%.4f,%.4f,"
                "%.4f,%.3f,%.3f,"
                "%.3f,%.3f,%.4f,%.4f,"
                "%.1f,%.1f,%.1f,%.1f,"
                "%d,%d,"
                "%d,%.4f\n",
                frame_counter, dt,
                engine_rpm, speed_kmh, fwd_speed,
                current_gear, is_shifting ? 1 : 0, shift_timer, shift_cooldown,
                clutch, input.throttle, input.brake,
                wheels[rear_left].angular_velocity, wheels[rear_right].angular_velocity,
                wheels[rear_left].slip_ratio, wheels[rear_right].slip_ratio,
                wheels[rear_left].tire_load, wheels[rear_right].tire_load,
                wheels[rear_left].longitudinal_force, wheels[rear_right].longitudinal_force,
                wheels[rear_left].grounded ? 1 : 0, wheels[rear_right].grounded ? 1 : 0,
                tc_active ? 1 : 0, tc_reduction);

            if (frame_counter % 200 == 0)
                fflush(file);
            frame_counter++;
        }
    };
    inline static telemetry_dump  telemetry;

    struct debug_sweep_data
    {
        PxVec3 origin;
        PxVec3 hit_point;
        bool   hit;
    };
    inline static debug_sweep_data debug_sweep[wheel_count];
    inline static PxVec3           debug_suspension_top[wheel_count];
    inline static PxVec3           debug_suspension_bottom[wheel_count];

    // pre-filter that skips the car's own body during suspension sweeps/raycasts
    class SelfFilterCallback : public PxQueryFilterCallback
    {
    public:
        PxRigidActor* ignore = nullptr;

        PxQueryHitType::Enum preFilter(const PxFilterData&, const PxShape*, const PxRigidActor* actor, PxHitFlags&) override
        {
            return (actor == ignore) ? PxQueryHitType::eNONE : PxQueryHitType::eBLOCK;
        }

        PxQueryHitType::Enum postFilter(const PxFilterData&, const PxQueryHit&, const PxShape*, const PxRigidActor*) override
        {
            return PxQueryHitType::eBLOCK;
        }
    };
    inline static SelfFilterCallback self_filter;

    inline bool  is_front(int i)                { return i == front_left || i == front_right; }
    inline bool  is_rear(int i)                 { return i == rear_left || i == rear_right; }
    inline bool  is_driven(int i)
    {
        if (tuning::spec.drivetrain_type == 0) return is_rear(i);   // rwd
        if (tuning::spec.drivetrain_type == 1) return is_front(i);  // fwd
        return true;                                                // awd
    }
    inline float lerp(float a, float b, float t){ return a + (b - a) * t; }
    inline float exp_decay(float rate, float dt){ return 1.0f - expf(-rate * dt); }

    inline bool        is_valid_wheel(int i) { return i >= 0 && i < wheel_count; }
    inline const char* get_wheel_name(int i) { return is_valid_wheel(i) ? wheel_names[i] : "??"; }
}
