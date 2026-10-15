#include "../Core/GameFramework/World.h"
#include "../Core/GameFramework/SceneComponent.h"
#include "../Core/GameFramework/Pawn.h"
#include <cstdio>
#include <cstdlib>
#include <limits>

using namespace PARS;

namespace
{
    void Check(bool value, const char* message)
    {
        if (!value) { std::fprintf(stderr, "FAIL: %s\n", message); std::exit(1); }
    }

    struct Record { int begins = 0, ticks = 0, ends = 0, destroyed = 0; };

    class ProbeComponent : public ActorComponent
    {
    public:
        ProbeComponent(Actor& owner, const std::string& name, Record& record)
            : ActorComponent(owner, name), m_Record(record) {}
        std::function<void(Actor&)> onBegin, onTick, onEnd;
    protected:
        void BeginPlay() override { ++m_Record.begins; if (onBegin) onBegin(GetOwner()); }
        void TickComponent(float) override { ++m_Record.ticks; if (onTick) onTick(GetOwner()); }
        void EndPlay() override { ++m_Record.ends; if (onEnd) onEnd(GetOwner()); }
    private:
        Record& m_Record;
    };

    class ProbeActor : public Actor
    {
    public:
        ProbeActor(const std::string& name, Record& record) : Actor(name), m_Record(record)
        {
            SetRootComponent(CreateDefaultSubobject<SceneComponent>("Root"));
            probe = &CreateDefaultSubobject<ProbeComponent>("Probe", record);
        }
        ~ProbeActor() override { ++m_Record.destroyed; }
        ProbeComponent* probe;
    private:
        Record& m_Record;
    };

    void TestOwnership()
    {
        Record a, b, c;
        World world, other;
        auto first = world.SpawnActor<ProbeActor>("First", a);
        auto second = world.SpawnActor<ProbeActor>("Second", b);
        auto foreign = other.SpawnActor<ProbeActor>("Foreign", c);
        const auto firstHandle = first->GetHandle(), secondHandle = second->GetHandle();
        Check(first->GetWorld() == &world && foreign->GetWorld() == &other, "Actor knows its owning World");
        Check(!world.FindActor(foreign->GetHandle()) && !world.DestroyActor(foreign->GetHandle()), "World rejects foreign handles");
        Check(world.FindActor<ProbeActor>(firstHandle) == first && !world.FindActor<Pawn>(firstHandle), "Typed handle lookup");
        Check(world.GetActorCount() == 2, "World owns multiple actors");
        first->SetActorLocation(Math::Vector3(1, 2, 3));
        second->SetActorLocation(Math::Vector3(4, 5, 6));
        Check(float(first->GetActorLocation().GetX()) == 1 && float(second->GetActorLocation().GetX()) == 4, "Actors keep independent transforms");
        world.Tick(0.1f);
        Check(a.ticks == 0, "World does not tick before BeginPlay");
        world.BeginPlay();
        world.BeginPlay();
        world.Tick(0.1f);
        Check(a.begins == 1 && b.begins == 1 && a.ticks == 1 && b.ticks == 1, "World begins and ticks every actor once");
        world.Tick(-1);
        world.Tick(std::numeric_limits<float>::quiet_NaN());
        Check(a.ticks == 1 && b.ticks == 1, "Invalid world delta rejected");
        Check(second->Destroy() && !second->Destroy(), "Destroy requested once");
        Check(!world.FindActor(secondHandle) && world.GetActorCount() == 1, "Pending actors are immediately hidden from lookup");
        Check(b.destroyed == 0 && b.ends == 0, "Actual destruction deferred to safe frame boundary");
        world.Tick(0.1f);
        Check(b.ticks == 1 && a.ticks == 2, "Pending actor no longer ticks");
        second->probe->onEnd = [&](Actor& actor) { Check(actor.GetWorld() == &world, "World remains available during EndPlay"); };
        world.CollectGarbage();
        Check(b.ends == 1 && b.destroyed == 1 && !world.HasPendingDestroy(), "EndPlay and deletion happen once");
        world.Clear();
        world.Clear();
        Check(a.ends == 1 && a.destroyed == 1 && world.GetActorCount() == 0, "Clear is idempotent");
        Check(!world.HasBegunPlay() && !world.FindActor(firstHandle), "Clear invalidates observer handles");
        auto replacement = world.SpawnActor<ProbeActor>("Replacement", a);
        Check(replacement->GetHandle() != firstHandle && replacement->GetHandle() != secondHandle, "New actors cannot reuse old handles");
        world.BeginPlay();
        world.Clear();
        Check(a.begins == 2 && a.ends == 2 && a.destroyed == 2, "World can restart after Clear");
        std::puts("PASS: World ownership, identity, independent actors and lifecycle");
    }

    void TestMutationDuringTick()
    {
        Record firstRecord, victimRecord, tailRecord;
        std::vector<Record> spawnedRecords(256);
        World world;
        auto first = world.SpawnActor<ProbeActor>("Spawner", firstRecord);
        auto victim = world.SpawnActor<ProbeActor>("Victim", victimRecord);
        first->CreateDefaultSubobject<ProbeComponent>("Tail", tailRecord);
        const auto firstHandle = first->GetHandle(), victimHandle = victim->GetHandle();
        first->probe->onTick = [&](Actor& self) {
            for (auto& record : spawnedRecords)
                Check(world.SpawnActor<ProbeActor>("Dummy", record) != nullptr, "Spawn while ticking");
            Check(self.Destroy() && world.DestroyActor(victimHandle), "Self and later actor destruction during Tick");
            world.CollectGarbage();
            world.Tick(0.1f); // Recursive dispatch is ignored.
            Check(firstRecord.destroyed == 0 && victimRecord.destroyed == 0, "Collection cannot destroy callback stack objects");
        };
        world.BeginPlay();
        world.Tick(0.1f);
        Check(!world.FindActor(firstHandle) && world.GetActorCount() == 256, "Mutation survives ownership-vector reallocation");
        Check(victimRecord.ticks == 0 && tailRecord.ticks == 0, "Skip deleted actor and remaining self components");
        for (const auto& record : spawnedRecords)
            Check(record.begins == 0 && record.ticks == 0, "Spawned actors wait until next Tick");
        world.CollectGarbage();
        Check(firstRecord.ends == 1 && victimRecord.ends == 1 && tailRecord.ends == 1, "Deferred actors and components end once");
        world.Tick(0.1f);
        for (const auto& record : spawnedRecords)
            Check(record.begins == 1 && record.ticks == 1, "Every new dummy ticks next frame");
        int visits = 0;
        world.ForEachActor([&](Actor& actor) {
            ++visits;
            actor.Destroy();
            world.CollectGarbage();
        });
        Check(visits == 256 && world.GetActorCount() == 0, "Render-style enumeration supports destruction requests");
        world.CollectGarbage();
        for (const auto& record : spawnedRecords)
            Check(record.ends == 1 && record.destroyed == 1, "All dummy lifetimes are reclaimed");
        std::puts("PASS: spawning 256 actors, deferred deletion and safe iteration");
    }

    void TestLifecycleMutations()
    {
        Record early, neverBegun, replacement, peer;
        World world;
        auto actor = world.SpawnActor<ProbeActor>("DestroyOnBegin", early);
        actor->CreateDefaultSubobject<ProbeComponent>("NeverBegun", neverBegun);
        actor->probe->onBegin = [](Actor& self) { self.Destroy(); };
        actor->probe->onEnd = [&](Actor&) { world.SpawnActor<ProbeActor>("Replacement", replacement); };
        world.BeginPlay();
        world.CollectGarbage();
        Check(early.begins == 1 && early.ends == 1 && early.destroyed == 1, "Destroy during BeginPlay is deferred");
        Check(neverBegun.begins == 0 && neverBegun.ends == 0, "EndPlay only runs for components that began");
        Check(replacement.begins == 0 && world.GetActorCount() == 1, "EndPlay may safely spawn a replacement");
        world.Tick(0.1f);
        Check(replacement.begins == 1 && replacement.ticks == 1, "Replacement begins on next tick");
        auto clearing = world.SpawnActor<ProbeActor>("ClearWorld", peer);
        clearing->probe->onTick = [&](Actor&) {
            world.Clear();
            Check(peer.destroyed == 0, "Clear during callback defers destruction");
            Check(world.SpawnActor<Actor>() == nullptr, "Spawning during Clear is rejected");
        };
        clearing->probe->onEnd = [&](Actor&) { Check(world.SpawnActor<Actor>() == nullptr, "Spawning during shutdown EndPlay is rejected"); };
        world.Tick(0.1f);
        world.CollectGarbage();
        Check(!world.HasBegunPlay() && world.GetActorCount() == 0 && !world.HasPendingDestroy(), "Clear requested from Tick completes at safe boundary");
        Check(replacement.ends == 1 && replacement.destroyed == 1 && peer.ends == 1 && peer.destroyed == 1, "Shutdown reclaims all actors exactly once");
        std::puts("PASS: BeginPlay/EndPlay mutations, replacement and world shutdown");
    }

    void TestStandaloneEndPlayReentry()
    {
        Record record;
        ProbeActor actor("Standalone", record);
        actor.probe->onEnd = [](Actor& self) {
            self.EndPlay();
            self.BeginPlay();
            bool rejected = false;
            try { self.CreateDefaultSubobject<SceneComponent>("DuringEnd"); }
            catch (const std::logic_error&) { rejected = true; }
            Check(rejected, "EndPlay cannot mutate the component ownership vector");
        };
        actor.BeginPlay();
        actor.EndPlay();
        Check(record.begins == 1 && record.ends == 1 && !actor.HasBegunPlay(), "Lifecycle hooks cannot recursively restart or end play");
    }
}

void RunWorldTests()
{
    TestOwnership();
    TestMutationDuringTick();
    TestLifecycleMutations();
    TestStandaloneEndPlayReentry();
}
