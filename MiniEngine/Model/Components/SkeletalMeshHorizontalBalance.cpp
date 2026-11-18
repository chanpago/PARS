#include "SkeletalMeshComponent.h"

using namespace PARS;
using namespace physx;

void SkeletalMeshComponent::UpdateHorizontalBalance(float deltaTime)
{
    const FootSupportSnapshot& sample = m_FootSupport.GetSnapshot();
    HorizontalBalanceInput input;
    input.enabled = IsActiveRagdoll() && m_AutomaticRootSupport && m_Ragdoll.UsesFootDrivenSupport() &&
        m_RootSupportRelease == RootSupportRelease::None && sample.hasCenterOfMass &&
        sample.hasCenterOfMassVelocity && m_FootstepIK.GetSnapshot().status == FootstepIKStatus::Applied;
    input.revision = sample.contactRevision;
    input.completedStep = sample.completedPhysicsStep;
    input.com = sample.centerOfMass;
    input.velocity = sample.centerOfMassVelocity;
    input.mass = sample.totalMass;
    input.deltaTime = deltaTime;
    input.supportWeight = m_Ragdoll.GetRootSupportWeight();
    input.stepping = sample.footstep.IsInProgress();
    for (size_t side = 0; side < input.feet.size(); ++side)
    {
        const FootSoleSample& foot = sample.feet[side];
        HorizontalBalanceFoot& target = input.feet[side];
        const Math::Vector3 position(foot.soleWorld.GetW());
        target.position = PxVec3(float(position.GetX()), float(position.GetY()), float(position.GetZ()));
        target.available = foot.available && foot.contact.hasSample;
        target.grounded = foot.contact.grounded;
        target.normal = foot.contact.normal;
        target.normalImpulse = foot.contact.normalImpulse;
        target.moving = sample.footstep.feet[side].IsInProgress();
    }
    m_HorizontalBalance.Update(input, m_HorizontalBalanceSettings);
    m_Ragdoll.SetHorizontalSupportForce(m_HorizontalBalance.GetSnapshot().force);
}
