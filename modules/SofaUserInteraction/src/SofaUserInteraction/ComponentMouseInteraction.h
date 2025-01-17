/******************************************************************************
*                 SOFA, Simulation Open-Framework Architecture                *
*                    (c) 2006 INRIA, USTL, UJF, CNRS, MGH                     *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#pragma once
#include <SofaUserInteraction/config.h>

#include <sofa/simulation/Simulation.h>
#include <sofa/simulation/DeleteVisitor.h>
#include <SofaUserInteraction/MouseInteractor.h>
#include <SofaBaseMechanics/IdentityMapping.h>
#include <sofa/core/Mapping.h>

namespace sofa::component::collision
{


class SOFA_SOFAUSERINTERACTION_API ComponentMouseInteraction
{
public:
    ComponentMouseInteraction();

    virtual ~ComponentMouseInteraction();

    virtual void createInteractionComponents(sofa::simulation::Node* parent,
            sofa::simulation::Node* current) = 0;

    void attach(simulation::Node* parentNode);

    void detach();

    void reset();

    virtual bool isCompatible( core::objectmodel::BaseContext *)const=0;

    typedef helper::Factory<std::string, ComponentMouseInteraction, core::objectmodel::BaseContext*> ComponentMouseInteractionFactory;

    template <class RealObject>
    static RealObject* create( RealObject*, core::objectmodel::BaseContext* /* context */)
    {
        return new RealObject;
    }

    //Components
    simulation::Node::SPtr nodeRayPick;
    sofa::core::behavior::BaseMechanicalState::SPtr mouseInSofa;
    sofa::component::collision::BaseMouseInteractor::SPtr mouseInteractor;
};



template <class DataTypes>
class TComponentMouseInteraction : public ComponentMouseInteraction
{
    typedef sofa::component::container::MechanicalObject< defaulttype::Vec3Types > MousePosition;
    typedef sofa::component::container::MechanicalObject< DataTypes > MouseContainer;
    typedef sofa::component::collision::MouseInteractor< DataTypes > Interactor;
    typedef sofa::component::mapping::IdentityMapping< defaulttype::Vec3Types, DataTypes > IdentityMechanicalMapping;
    typedef typename sofa::core::Mapping< defaulttype::Vec3Types, DataTypes >::SPtr MouseMapping;

public:


    void createInteractionComponents(sofa::simulation::Node* parent, sofa::simulation::Node* current) override;

    bool  isCompatible( core::objectmodel::BaseContext *context) const override;
protected :
    MouseMapping mouseMapping;
};

#if  !defined(SOFA_COMPONENT_COLLISION_COMPONENTMOUSEINTERACTION_CPP)
extern template class SOFA_SOFAUSERINTERACTION_API TComponentMouseInteraction<defaulttype::Vec2Types>;
extern template class SOFA_SOFAUSERINTERACTION_API TComponentMouseInteraction<defaulttype::Vec3Types>;
extern template class SOFA_SOFAUSERINTERACTION_API TComponentMouseInteraction<defaulttype::Rigid3Types>;

#endif
} // namespace sofa::component::collision



#if  !defined(SOFA_COMPONENT_COLLISION_COMPONENTMOUSEINTERACTION_CPP)
namespace sofa::helper
{
extern template class SOFA_SOFAUSERINTERACTION_API Factory<std::string, component::collision::ComponentMouseInteraction, core::objectmodel::BaseContext*>;
} // namespace sofa::helper
#endif
