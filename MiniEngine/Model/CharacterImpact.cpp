#include "CharacterImpact.h"
#pragma warning(push)
#pragma warning(disable : 4100)
#include "json.hpp"
#pragma warning(pop)
#include <algorithm>
#include <cmath>
#include <Windows.h>
#include <cfloat>
#include <fstream>

namespace PARS
{
    namespace
    {
        std::wstring ImpactPath(const std::string& path)
        {
            const int count = MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, path.c_str(), -1, nullptr, 0);
            if (count <= 0)
            {
                throw std::runtime_error("Invalid UTF-8 impact settings path.");
            }
            std::wstring result(count, L'\0');
            MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, path.c_str(), -1, &result[0], count);
            result.resize(count - 1);
            return result;
        }
    }

    bool CharacterImpactSettings::Validate(std::string& error) const
    {
        const float values[] = {collisionMass, capsuleRadiusScale, capsuleHeightScale, contactSkin};
        for (float value : values)
        {
            if (!std::isfinite(value) || value < 0)
            {
                error = u8"밀침 설정은 유한한 0 이상의 값이어야 합니다.";
                return false;
            }
        }
        if (collisionMass < .01f || collisionMass > 10000 || capsuleRadiusScale <= 0 || capsuleHeightScale <= 0 || contactSkin > .1f)
        {
            error = u8"충돌 질량은 0.01~10000kg, 캡슐 배율은 양수, 접촉 여유는 0~0.1m여야 합니다.";
            return false;
        }
        error.clear();
        return true;
    }

    bool CharacterImpactSettings::Save(const std::string& path, std::string& error) const
    {
        if (!Validate(error))
        {
            return false;
        }
        try
        {
            const nlohmann::json data = {{"version", 6}, {"enabled", enabled}, {"collisionMassKg", collisionMass},
                {"capsuleRadiusScale", capsuleRadiusScale}, {"capsuleHeightScale", capsuleHeightScale}, {"contactSkinMeters", contactSkin}};
            std::ofstream file(ImpactPath(path), std::ios::trunc);
            file << data.dump(4) << '\n';
            file.flush();
            if (!file)
            {
                throw std::runtime_error("Could not write character impact settings.");
            }
            error.clear();
            return true;
        }
        catch (const std::exception& exception)
        {
            error = exception.what();
            return false;
        }
    }

    bool CharacterImpactSettings::Load(const std::string& path, std::string& error)
    {
        try
        {
            std::ifstream file(ImpactPath(path));
            nlohmann::json data;
            file >> data;
            const int version = data.at("version").get<int>();
            if (version < 1 || version > 6)
            {
                throw std::runtime_error("Unsupported character impact settings version.");
            }
            CharacterImpactSettings candidate;
            candidate.enabled = data.at("enabled").get<bool>();
            // 이전 골반 질량 설정은 독립 충돌체의 질량으로만 이전한다. 질량이 없는 버전은 기본값을 쓴다.
            if (version >= 6)
            {
                candidate.collisionMass = data.at("collisionMassKg").get<float>();
            }
            else if (version == 3 || version == 4)
            {
                candidate.collisionMass = data.at("bodyMassKg").get<float>();
            }
            candidate.capsuleRadiusScale = data.at("capsuleRadiusScale").get<float>();
            if (version >= 4)
            {
                candidate.capsuleHeightScale = data.at("capsuleHeightScale").get<float>();
            }
            candidate.contactSkin = data.at("contactSkinMeters").get<float>();
            if (!candidate.Validate(error))
            {
                return false;
            }
            *this = candidate;
            error.clear();
            return true;
        }
        catch (const std::exception& exception)
        {
            error = exception.what();
            return false;
        }
    }
}
