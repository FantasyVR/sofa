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
#include <SofaBaseTopology/config.h>

#include <sofa/core/topology/BaseTopologyEngine.h>
#include <sofa/core/topology/TopologyHandler.h>
#include <sofa/core/topology/BaseTopologyData.h>

#include <sofa/core/topology/BaseTopology.h>

#include <sofa/defaulttype/VecTypes.h>

namespace sofa::component::topology
{



////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////   Generic Topology Data Implementation   /////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////

template< class VecT>
class TopologyEngineImpl : public sofa::core::topology::TopologyEngine
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(TopologyEngineImpl,VecT), sofa::core::topology::TopologyEngine);
    typedef VecT container_type;
    typedef typename container_type::value_type value_type;
    typedef sofa::core::topology::BaseTopologyData<VecT> t_topologicalData;



    typedef core::topology::BaseMeshTopology::Point Point;
    typedef core::topology::BaseMeshTopology::Edge Edge;
    typedef core::topology::BaseMeshTopology::Triangle Triangle;
    typedef core::topology::BaseMeshTopology::Quad Quad;
    typedef core::topology::BaseMeshTopology::Tetrahedron Tetrahedron;
    typedef core::topology::BaseMeshTopology::Hexahedron Hexahedron;

protected:
    TopologyEngineImpl(t_topologicalData* _topologicalData,
            sofa::core::topology::BaseMeshTopology* _topology,
            sofa::core::topology::TopologyHandler* _topoHandler);

public:

    void init() override;

    void reinit() override;

    void doUpdate() override;

    void ApplyTopologyChanges();

    void registerTopology(sofa::core::topology::BaseMeshTopology* _topology);

    void registerTopology();

    void registerTopologicalData(t_topologicalData *topologicalData) {m_topologicalData = topologicalData;}


    /// Function to link DataEngine with Data array from topology
    void linkToPointDataArray() override;
    void linkToEdgeDataArray() override;
    void linkToTriangleDataArray() override;
    void linkToQuadDataArray() override;
    void linkToTetrahedronDataArray() override;
    void linkToHexahedronDataArray() override;

protected:
    t_topologicalData* m_topologicalData;
    sofa::core::topology::TopologyContainer* m_topology;
    sofa::core::topology::TopologyHandler* m_topoHandler;

public:
    bool m_pointsLinked;
    bool m_edgesLinked;
    bool m_trianglesLinked;
    bool m_quadsLinked;
    bool m_tetrahedraLinked;
    bool m_hexahedraLinked;

};


} //namespace sofa::component::topology
