/******************************************************************************
*                 SOFA, Simulation Open-Framework Architecture                *
*                    (c) 2006 INRIA, USTL, UJF, CNRS, MGH                     *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU General Public License as published by the Free  *
* Software Foundation; either version 2 of the License, or (at your option)   *
* any later version.                                                          *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for    *
* more details.                                                               *
*                                                                             *
* You should have received a copy of the GNU General Public License along     *
* with this program. If not, see <http://www.gnu.org/licenses/>.              *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#pragma once

#include <SofaSimulationGraph/config.h>
#include <string>
#include <sstream>
#include <map>

#include <sofa/simulation/Node.h>
#include <sofa/simulation/Simulation.h>

namespace sofa::simpleapi
{

using sofa::core::objectmodel::BaseObject;
using sofa::core::objectmodel::BaseObjectDescription;

using sofa::simulation::Simulation ;
using sofa::simulation::Node ;

bool SOFA_SOFASIMULATIONGRAPH_API importPlugin(const std::string& name) ;

Simulation::SPtr SOFA_SOFASIMULATIONGRAPH_API createSimulation(const std::string& type="DAG") ;

Node::SPtr SOFA_SOFASIMULATIONGRAPH_API createRootNode( Simulation::SPtr, const std::string& name,
    const std::map<std::string, std::string>& params = std::map<std::string, std::string>{} );

///@brief Create a sofa object in the provided node.
///The parameter "params" is for passing specific data argument to the created object including the
///object's type.
BaseObject::SPtr SOFA_SOFASIMULATIONGRAPH_API createObject(Node::SPtr node, BaseObjectDescription& params);

///@brief create a sofa object in the provided node of the given type.
///The parameter "params" is for passing specific data argument to the created object.
BaseObject::SPtr SOFA_SOFASIMULATIONGRAPH_API createObject( Node::SPtr node, const std::string& type,
    const std::map<std::string, std::string>& params = std::map<std::string, std::string>{} );

///@brief create a child to the provided nodeof given name.
///The parameter "params" is for passing specific data argument to the created object.
Node::SPtr SOFA_SOFASIMULATIONGRAPH_API createChild( Node::SPtr& node, const std::string& name,
    const std::map<std::string, std::string>& params = std::map<std::string, std::string>{} );

///@brief create a child to the provided node.
///The parameter "params" is for passing specific data argument to the created object (including the node name).
Node::SPtr SOFA_SOFASIMULATIONGRAPH_API createChild(Node::SPtr node, BaseObjectDescription& desc);

void SOFA_SOFASIMULATIONGRAPH_API dumpScene(Node::SPtr root) ;

template<class T>
std::string str(const T& t)
{
    std::stringstream s;
    s << t;
    return s.str() ;
}
} // namespace sofa::simpleapi


namespace sofa::simpleapi::components 
{

namespace BaseObject
{
    static const std::string aobjectname {"BaseObject"} ;
    namespace data{
        static const std::string name {"name"} ;
    }
}

namespace MechanicalObject
{
    static const std::string objectname {"MechanicalObject"} ;
    namespace data{
        using namespace BaseObject::data ;
        static const std::string position {"position"} ;
    }
}

namespace VisualModel
{
    static const std::string objectname {"VisualModel"} ;

    namespace data {
        using namespace BaseObject::data ;
        static const std::string filename {"filename"} ;
    }
}

} // namespace sofa::simpleapi::components

namespace sofa::meca   { using namespace sofa::simpleapi::components::MechanicalObject ; }
namespace sofa::visual { using namespace sofa::simpleapi::components::VisualModel ; }
