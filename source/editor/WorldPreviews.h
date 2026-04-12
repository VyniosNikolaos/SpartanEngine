#pragma once

#include "Game/Game.h"
#include <string>

namespace spartan
{
    class RHI_Texture;
}

class WorldPreviews
{
public:
    static void Shutdown();
    static void Tick();

    static void RequestGeneration(spartan::DefaultWorld default_world);
    static void RequestGeneration(const std::string& world_file_path);

    static std::string GetPreviewPath(spartan::DefaultWorld default_world);
    static std::string GetPreviewPath(const std::string& world_file_path);

    static spartan::RHI_Texture* GetTexture(spartan::DefaultWorld default_world);
    static spartan::RHI_Texture* GetTexture(const std::string& world_file_path);
};
