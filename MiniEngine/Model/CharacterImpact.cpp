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
        const float values[] = {referenceImpulse, referenceSpeed, strengthMultiplier, maximumImpulse,
            minimumClosingSpeed, pelvisShare, separationTime, separationDistance, cooldown, capsuleRadiusScale, contactSkin};
        for (float value : values)
        {
            if (!std::isfinite(value) || value < 0)
            {
                error = u8"밀침 설정은 유한한 0 이상의 값이어야 합니다.";
                return false;
            }
        }
        if (referenceSpeed <= 0 || pelvisShare > 1 || capsuleRadiusScale <= 0 || contactSkin > .1f)
        {
            error = u8"기준 속도·캡슐 배율은 양수, 골반 비율은 0~1, 접촉 여유는 0~0.1m여야 합니다.";
            return false;
        }
        error.clear();
        return true;
    }

    float CharacterImpactSettings::ComputeImpulse(float closingSpeed, float& requested) const
    {
        requested = 0;
        if (!enabled || !std::isfinite(closingSpeed) || closingSpeed < minimumClosingSpeed ||
            closingSpeed <= 0 || referenceSpeed <= 0)
        {
            return 0;
        }
        const double amount = double(referenceImpulse) * strengthMultiplier * closingSpeed / referenceSpeed;
        requested = static_cast<float>((std::min)(amount, double(FLT_MAX)));
        return (std::min)(requested, maximumImpulse);
    }

    bool CharacterImpactSettings::Save(const std::string& path, std::string& error) const
    {
        if (!Validate(error))
        {
            return false;
        }
        try
        {
            const nlohmann::json data = {{"version", 1}, {"enabled", enabled},
                {"referenceImpulseNs", referenceImpulse}, {"referenceSpeedMps", referenceSpeed},
                {"strengthMultiplier", strengthMultiplier}, {"maximumImpulseNs", maximumImpulse},
                {"minimumClosingSpeedMps", minimumClosingSpeed}, {"pelvisShare", pelvisShare},
                {"separationTimeSeconds", separationTime}, {"separationDistanceMeters", separationDistance}, {"cooldownSeconds", cooldown},
                {"capsuleRadiusScale", capsuleRadiusScale}, {"contactSkinMeters", contactSkin}};
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
            if (data.at("version").get<int>() != 1)
            {
                throw std::runtime_error("Unsupported character impact settings version.");
            }
            CharacterImpactSettings candidate;
            candidate.enabled = data.at("enabled").get<bool>();
            candidate.referenceImpulse = data.at("referenceImpulseNs").get<float>();
            candidate.referenceSpeed = data.at("referenceSpeedMps").get<float>();
            candidate.strengthMultiplier = data.at("strengthMultiplier").get<float>();
            candidate.maximumImpulse = data.at("maximumImpulseNs").get<float>();
            candidate.minimumClosingSpeed = data.at("minimumClosingSpeedMps").get<float>();
            candidate.pelvisShare = data.at("pelvisShare").get<float>();
            candidate.separationTime = data.at("separationTimeSeconds").get<float>();
            candidate.separationDistance = data.at("separationDistanceMeters").get<float>();
            candidate.cooldown = data.at("cooldownSeconds").get<float>();
            candidate.capsuleRadiusScale = data.at("capsuleRadiusScale").get<float>();
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
