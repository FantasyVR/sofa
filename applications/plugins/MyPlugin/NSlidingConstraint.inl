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
#ifndef SOFA_COMPONENT_CONSTRAINTSET_SLIDINGCONSTRAINT_INL
#define SOFA_COMPONENT_CONSTRAINTSET_SLIDINGCONSTRAINT_INL

#include "NSlidingConstraint.h"
#include <sofa/core/visual/VisualParams.h>
#include <SofaConstraint/BilateralInteractionConstraint.h>
#include <SofaConstraint/UnilateralInteractionConstraint.h>
#include <sofa/defaulttype/RGBAColor.h>
#include <sofa/defaulttype/Vec.h>
namespace sofa
{

namespace component
{

namespace constraintset
{

template<class DataTypes>
NSlidingConstraint<DataTypes>::NSlidingConstraint()
    : NSlidingConstraint(nullptr, nullptr)
{
}

template<class DataTypes>
NSlidingConstraint<DataTypes>::NSlidingConstraint(MechanicalState* object)
    : NSlidingConstraint(object, object)
{
}

template<class DataTypes>
NSlidingConstraint<DataTypes>::NSlidingConstraint(MechanicalState* object1, MechanicalState* object2)
    : Inherit(object1, object2)
    //, d_m1(initData(&d_m1, 0, "sliding_point","index of the spliding point on the first model"))
    //, d_m2a(initData(&d_m2a, 0, "axis_1","index of one end of the sliding axis"))
    //, d_m2b(initData(&d_m2b, 0, "axis_2","index of the other end of the sliding axis"))
    , d_force(initData(&d_force,"force","force (impulse) used to solve the constraint"))
    , m_yetIntegrated(false)
{
}

template<class DataTypes>
void NSlidingConstraint<DataTypes>::init()
{
    assert(this->mstate1);
    assert(this->mstate2);

    m_thirdConstraint = 0;
}

template<class DataTypes>
inline void NSlidingConstraint<DataTypes>::clear(int reserve)
{
    slidingContacts.clear();
    if (reserve)
        slidingContacts.reserve(reserve);
}

template<class DataTypes>
inline void NSlidingConstraint<DataTypes>::addContact(double mu, Deriv norm, Real contactDistance, int m1, int m2, long id, PersistentID localid)
{
    addContact(mu, norm,
        this->getMState2()->read(core::ConstVecCoordId::position())->getValue()[m2],
        this->getMState1()->read(core::ConstVecCoordId::position())->getValue()[m1],
        contactDistance, m1, m2,
        this->getMState2()->read(core::ConstVecCoordId::freePosition())->getValue()[m2],
        this->getMState1()->read(core::ConstVecCoordId::freePosition())->getValue()[m1],
        id, localid);
}

template<class DataTypes>
inline void NSlidingConstraint<DataTypes>::addContact(double mu, Deriv norm, Coord P, Coord Q, Real contactDistance, int m1, int m2, Coord Pfree, Coord Qfree, long id, PersistentID localid)
{
    slidingContacts.resize(slidingContacts.size() + 1);
    Contact& c = slidingContacts.back();

    c.P = P;
    c.Q = Q;
    c.m1 = m1;
    c.m2 = m2;
    c.norm = norm;
    c.t = Deriv(norm.z(), norm.x(), norm.y());
    c.s = cross(norm, c.t);
    c.s = c.s / c.s.norm();
    c.t = cross((-norm), c.s);
    c.mu = mu;
    c.contactId = id;
    c.localId = localid;
    c.contactDistance = contactDistance;
}


template<class DataTypes>
void NSlidingConstraint<DataTypes>::buildConstraintMatrix(const core::ConstraintParams*, DataMatrixDeriv &c1_d, DataMatrixDeriv &c2_d, unsigned int &cIndex
        , const DataVecCoord &x1, const DataVecCoord &x2)
{
    assert(this->mstate1);
    assert(this->mstate2);
    // for each sliding contact, construct constraint matrix
    for (auto c = slidingContacts.begin(); c != slidingContacts.end(); c++)
    {
        MatrixDeriv& c1 = *c1_d.beginEdit();
        MatrixDeriv& c2 = *c2_d.beginEdit();

        auto ele1_index = c1->m1;
        auto ele2_index = c2->m2;

        m_dirAxe = c->norm;
        m_dirAxe.normalize();

        // projection of the point on the axis
        Real r = (c->P - c->Q) * m_dirAxe;
        const Deriv proj = c->Q + m_dirAxe * r;

        // We move the constraint point onto the projection
        m_dirProj = c->P - proj;
        m_dist = m_dirProj.norm(); // constraint violation
        m_dirProj.normalize(); // direction of the constraint

        m_dirOrtho = cross(m_dirProj, m_dirAxe);
        m_dirOrtho.normalize();

        m_cid = cIndex;
        cIndex += 2;
        // Construct Local coordinate frame and write into system matrix ?
        // No thrid constraint like sliding constraint, because we didn't want to constraint the motion 
        // on m_dirAxe direction
        MatrixDerivRowIterator c1_it = c1.writeLine(m_cid);
        c1.addCol(ele1_index, m_dirProj);
        c1_it = c1.writeLine(m_cid+1);
        c1.addCol(ele1_index, m_dirOrtho);
        MatrixDerivRowIterator c2_it = c2.writeLine(m_cid);
        c2.addCol(ele2_index, -m_dirProj);
        c2_it = c2.writeLine(m_cid + 1);
        c2.addCol(ele2_index, -m_dirOrth0);

        c1_d.endEdit();
        c2_d.endEdit();
    }
}


template<class DataTypes>
void NSlidingConstraint<DataTypes>::getConstraintViolation(const core::ConstraintParams *, defaulttype::BaseVector *v, const DataVecCoord &, const DataVecCoord &
        , const DataVecDeriv &, const DataVecDeriv &)
{
    v->set(m_cid, m_dist);
    v->set(m_cid+1, 0.0);
}


template<class DataTypes>
void NSlidingConstraint<DataTypes>::getConstraintResolution(const ConstraintParams*,
                                                           std::vector<core::behavior::ConstraintResolution*>& resTab,
                                                           unsigned int& offset)
{
    resTab[offset++] = new BilateralConstraintResolution();
    resTab[offset++] = new BilateralConstraintResolution();
}


template<class DataTypes>
void NSlidingConstraint<DataTypes>::storeLambda(const ConstraintParams* /*cParams*/, sofa::core::MultiVecDerivId /*res*/, const sofa::defaulttype::BaseVector* lambda)
{
    Real lamb1,lamb2;

    lamb1 = lambda->element(m_cid);
    lamb2 = lambda->element(m_cid+1);

    d_force.setValue( m_dirProj* lamb1 + m_dirOrtho * lamb2 );
}

template<class DataTypes>
void NSlidingConstraint<DataTypes>::draw(const core::visual::VisualParams* vparams)
{
    if (!vparams->displayFlags().getShowInteractionForceFields())
        return;

    vparams->drawTool()->saveLastState();

    vparams->drawTool()->disableLighting();

    sofa::defaulttype::RGBAColor color;

    if(m_thirdConstraint<0)
        color = sofa::defaulttype::RGBAColor::yellow();
    else if(m_thirdConstraint>0)
        color = sofa::defaulttype::RGBAColor::green();
    else
        color = sofa::defaulttype::RGBAColor::magenta();

    //std::vector<sofa::defaulttype::Vector3> vertices;
    //vertices.push_back(DataTypes::getCPos((this->mstate1->read(core::ConstVecCoordId::position())->getValue())[d_m1.getValue()]));

    //vparams->drawTool()->drawPoints(vertices, 10, color);
    //vertices.clear();

    //color = sofa::defaulttype::RGBAColor::blue();
    //vertices.push_back(DataTypes::getCPos((this->mstate2->read(core::ConstVecCoordId::position())->getValue())[d_m2a.getValue()]));
    //vertices.push_back(DataTypes::getCPos((this->mstate2->read(core::ConstVecCoordId::position())->getValue())[d_m2b.getValue()]));
    //vparams->drawTool()->drawLines(vertices, 1, color);

    vparams->drawTool()->restoreLastState();
}

} // namespace constraintset

} // namespace component

} // namespace sofa

#endif
