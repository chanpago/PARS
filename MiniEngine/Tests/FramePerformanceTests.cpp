#include "FramePerformance.h"
#include "PhysicsWorld.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace
{
    void CheckTiming(bool passed, const char* message)
    {
        if (!passed)
        {
            std::fprintf(stderr, "FAILED: %s\n", message);
            std::exit(1);
        }
    }

    bool Near(double actual, double expected)
    {
        return std::abs(actual - expected) < 1e-9;
    }
}

void TestFramePerformance()
{
    using FramePerformance::Category;
    FramePerformance::Recorder recorder;
    CheckTiming(recorder.GetFrameCount() == 0 && recorder.GetStatistics(Category::PhysXStep).last == 0.0,
        "Empty timing history is zero");
    recorder.BeginFrame();
    recorder.AddSample(Category::PhysXStep, 2.0);
    recorder.AddSample(Category::PhysXStep, 3.0);
    recorder.AddSample(Category::ParsControl, 1.0);
    const Category controlStages[] = {Category::FootSupport, Category::FootstepPlanning, Category::LandingTargetSearch,
        Category::FootstepIK, Category::RecoveryTargets, Category::RagdollDrives};
    for (const Category stage : controlStages)
    {
        recorder.AddSample(stage, 0.5);
    }
    const Category trajectoryStages[] = {Category::SwingTrajectory, Category::TrajectorySetup, Category::CurrentPathSamples,
        Category::CurrentPathMidpoints, Category::CurrentPathIntervals, Category::DetourRanking, Category::DetourSamples,
        Category::DetourMidpoints, Category::DetourIntervals};
    for (const Category stage : trajectoryStages)
    {
        recorder.AddSample(stage, 0.25);
    }
    recorder.AddSample(Category::ParsPose, 0.25);
    recorder.AddSample(Category::ParsOther, 0.5);
    recorder.AddSample(Category::RenderPreparation, 4.0);
    recorder.EndFrame();
    CheckTiming(Near(recorder.GetStatistics(Category::PhysXStep).last, 5.0) &&
        recorder.GetStatistics(Category::PhysXStep).calls == 2, "Multiple steps accumulate within one frame");
    CheckTiming(Near(recorder.GetStatistics(Category::TrajectoryTotal).last, 2.25) &&
        Near(recorder.GetStatistics(Category::ControlTotal).last, 6.25) &&
        Near(recorder.GetStatistics(Category::ParsTotal).last, 7.0),
        "Each control and trajectory stage contributes once to its totals");
    recorder.BeginFrame();
    recorder.EndFrame();
    const FramePerformance::Statistics idle = recorder.GetStatistics(Category::PhysXStep);
    CheckTiming(idle.last == 0.0 && idle.calls == 0 && Near(idle.average, 2.5) && Near(idle.maximum, 5.0),
        "Zero-step frames clear current values and remain in the average");
    recorder.BeginFrame();
    recorder.AddSample(Category::DetourSamples, 3.0);
    recorder.EndFrame();
    const size_t detour = static_cast<size_t>(Category::DetourSamples);
    const FramePerformance::Sample& peak = recorder.GetPeakFrame(Category::ParsTotal);
    CheckTiming(Near(peak.milliseconds[detour], 0.25) && peak.calls[detour] == 1 &&
        peak.calls[static_cast<size_t>(Category::PhysXStep)] == 2 &&
        Near(recorder.GetStatistics(Category::DetourSamples).maximum, 3.0),
        "Peak times and calls come from the same frame even when a stage peaks in another frame");
    for (size_t frame = 0; frame < FramePerformance::HistoryLength; ++frame)
    {
        recorder.BeginFrame();
        recorder.AddSample(Category::PhysXStep, 1.0);
        recorder.EndFrame();
    }
    CheckTiming(recorder.GetFrameCount() == FramePerformance::HistoryLength &&
        Near(recorder.GetStatistics(Category::PhysXStep).average, 1.0) &&
        Near(recorder.GetStatistics(Category::PhysXStep).maximum, 1.0),
        "Rolling history expires old peaks");
    CheckTiming(recorder.GetPeakFrame(Category::ParsTotal).calls[detour] == 0,
        "An expired PARS peak cannot supply the displayed stage breakdown");

    PhysicsWorld world;
    CheckTiming(world.Initialize(false), "Initialize real PhysX timing fixture");
    const std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    FramePerformance::BeginFrame();
    {
        FramePerformance::Scope outer(Category::ParsOther);
        world.Step();
        world.Step();
    }
    FramePerformance::EndFrame();
    const double elapsed = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    const FramePerformance::Statistics physics = FramePerformance::GetState().recorder.GetStatistics(Category::PhysXStep);
    const FramePerformance::Statistics pars = FramePerformance::GetState().recorder.GetStatistics(Category::ParsTotal);
    CheckTiming(physics.calls == 2 && physics.last > 0.0 && pars.last >= 0.0,
        "Real PhysX steps are timed across translation units");
    CheckTiming(physics.last + pars.last <= elapsed,
        "Nested PhysX duration is excluded from PARS elapsed time");
    FramePerformance::BeginFrame();
    FramePerformance::EndFrame();
    CheckTiming(FramePerformance::GetState().recorder.GetStatistics(Category::PhysXStep).calls == 0 &&
        FramePerformance::GetState().recorder.GetStatistics(Category::PhysXStep).last == 0.0,
        "A paused frame does not retain the previous simulation time");
    world.Step();
    CheckTiming(FramePerformance::GetState().recorder.GetStatistics(Category::PhysXStep).last == 0.0,
        "Work outside a measured frame does not change the displayed sample");
    CheckTiming(world.GetErrorCount() == 0, "Timing scopes do not produce PhysX errors");
    std::printf("PASS: frame timing aggregation, zero frames, history, nested scopes and real PhysX (%.3f ms)\n",
        physics.last);
}
