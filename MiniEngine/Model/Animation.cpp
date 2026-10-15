//
// Copyright (c) Microsoft. All rights reserved.
// This code is licensed under the MIT License (MIT).
// THIS CODE IS PROVIDED *AS IS* WITHOUT WARRANTY OF
// ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING ANY
// IMPLIED WARRANTIES OF FITNESS FOR A PARTICULAR
// PURPOSE, MERCHANTABILITY, OR NON-INFRINGEMENT.
//
// Developed by Minigraph
//
// Author:   James Stanard
//

#include "Animation.h"
#include <cmath>
#include "Model.h"

void ModelInstance::UpdateAnimations(float deltaTime)
{
    if (!m_Model || !m_AnimGraph || !std::isfinite(deltaTime) || deltaTime < 0.0f)
        return;
    for (uint32_t i = 0; i < m_Model->m_NumAnimations; ++i)
    {
        AnimationState& state = m_AnimState[i];
        if (state.state == AnimationState::kStopped)
            continue;
        const AnimationSet& animation = m_Model->m_Animations[i];
        state.time += deltaTime;
        if (state.state == AnimationState::kLooping && animation.duration > 0.0f)
            state.time = fmodf(state.time, animation.duration);
        else if (state.time >= animation.duration)
        {
            state.time = animation.duration;
            state.state = AnimationState::kStopped;
        }
        EvaluateAnimationPose(animation, m_Model->m_CurveData.get(), m_Model->m_KeyFrameData.get(),
            m_AnimGraph.get(), m_Model->m_NumNodes, state.time);
    }
}

void ModelInstance::PlayAnimation(uint32_t animIdx, bool loop)
{
    if (animIdx < m_AnimState.size())
        m_AnimState[animIdx].state = loop ? AnimationState::kLooping : AnimationState::kPlaying;
}

void ModelInstance::PauseAnimation(uint32_t animIdx)
{
    if (animIdx < m_AnimState.size())
        m_AnimState[animIdx].state = AnimationState::kStopped;
}

void ModelInstance::ResetAnimation(uint32_t animIdx)
{
    if (animIdx < m_AnimState.size())
        m_AnimState[animIdx].time = 0.0f;
}

void ModelInstance::StopAnimation(uint32_t animIdx)
{
    if (animIdx < m_AnimState.size())
    {
        m_AnimState[animIdx].state = AnimationState::kStopped;
        m_AnimState[animIdx].time = 0.0f;
    }
}

void ModelInstance::LoopAllAnimations(void)
{
    for (auto& anim : m_AnimState)
    {
        anim.state = AnimationState::kLooping;
        anim.time = 0.0f;
    }
}
