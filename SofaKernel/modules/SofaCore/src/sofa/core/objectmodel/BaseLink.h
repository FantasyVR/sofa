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
#ifndef SOFA_CORE_OBJECTMODEL_BASELINK_H
#define SOFA_CORE_OBJECTMODEL_BASELINK_H

#include <sofa/core/config.h>
#include <sofa/core/ExecParams.h>
#include <string>

namespace sofa
{

namespace core
{

namespace objectmodel
{

class Base;
class BaseData;
class BaseClass;
class BaseObjectDescription;

/**
 *  \brief Abstract base class for all links in the scene grapn, independently of their type.
 *
 */
class SOFA_CORE_API BaseLink
{
public:
    SOFA_BEGIN_DEPRECATION_AS_ERROR
    enum LinkFlagsEnum
    {
        FLAG_NONE       = 0,
        FLAG_MULTILINK  = 1 << 0, ///< True if link is an array
        FLAG_STRONGLINK = 1 << 1, ///< True if link has ownership of linked object(s)
        FLAG_DOUBLELINK = 1 << 2, ///< True if link has a reciprocal link in linked object(s)
        FLAG_DUPLICATE  = 1 << 4, ///< True if link duplicates another one (possibly with a different/specialized DestType)
        FLAG_STOREPATH  = 1 << 5, ///< True if link requires a path string in order to be created
    };
    typedef unsigned LinkFlags;

    /// This internal class is used by the initLink() methods to store initialization parameters of a Data
    class BaseInitLink
    {
    public:
        BaseInitLink(const std::string& name, const std::string& help) : name(name), help(help) {}
        std::string name;
        std::string help;
    };

    /// This internal class is used by the initLink() methods to store initialization parameters of a Data
    template<class Owner>
    class InitLink : public BaseInitLink
    {
    public:
        InitLink(Owner* o, const std::string& n, const std::string& h) : BaseInitLink(n, h), owner(o) {}
        Owner* owner;
    };

    BaseLink(LinkFlags flags);
    BaseLink(const BaseInitLink& init, LinkFlags flags);
    virtual ~BaseLink();

    const std::string& getName() const { return m_name; }
    void setName(const std::string& name) { m_name = name; }

    /// Get help message
    const std::string& getHelp() const { return m_help; }

    /// Set help message
    void setHelp(const std::string& val) { m_help = val; }

    virtual Base* getOwnerBase() const = 0;

    [[deprecated("2020-10-03: Deprecated since PR #1503. BaseLink cannot hold Data anymore. Use DataLink instead. Please update your code. ")]]
    virtual sofa::core::objectmodel::BaseData* getOwnerData() const = 0;


    /// Set one of the flags.
    void setFlag(LinkFlagsEnum flag, bool b)
    {
        if(b) m_flags |= LinkFlags(flag);
        else m_flags &= ~LinkFlags(flag);
    }

    /// Get one flag
    bool getFlag(LinkFlagsEnum flag) const { return (m_flags&LinkFlags(flag))!=0; }

    bool isMultiLink() const { return getFlag(FLAG_MULTILINK); }

    [[deprecated("2020-10-03: Deprecated since PR #1503. BaseLink cannot hold Data anymore. Use DataLink instead. Please update your code. ")]]
    bool isDataLink() const { return false; }
    bool isStrongLink() const { return getFlag(FLAG_STRONGLINK); }
    bool isDoubleLink() const { return getFlag(FLAG_DOUBLELINK); }
    bool isDuplicate() const { return getFlag(FLAG_DUPLICATE); }
    bool storePath() const { return getFlag(FLAG_STOREPATH); }

    /// Alias to match BaseData API
    void setPersistent(bool b) { setFlag(FLAG_STOREPATH, b); }
    bool isPersistent() const { return storePath(); }

    /// Alias to match BaseData API
    bool isReadOnly() const   { return !storePath(); }

    virtual const BaseClass* getDestClass() const = 0;
    virtual const BaseClass* getOwnerClass() const = 0;

    /// Return the number of changes since creation
    /// This can be used to efficiently detect changes
    int getCounter() const { return m_counter; }

    /// Return the number of changes since creation
    /// This can be used to efficiently detect changes
    [[deprecated("2020-03-25: Aspect have been deprecated for complete removal in PR #1269. You can probably update your code by removing aspect related calls. If the feature was important to you contact sofa-dev. ")]]
    int getCounter(const core::ExecParams*) const { return getCounter(); }

    void setLinkedBase(Base* link);

    virtual size_t getSize() const = 0;
    virtual Base* getLinkedBase(std::size_t index=0) const = 0;

    [[deprecated("2020-10-03: Deprecated since PR #1503. BaseLink cannot hold Data anymore. Use DataLink instead. Please update your code. ")]]
    virtual BaseData* getLinkedData(std::size_t index=0) const = 0;

    virtual std::string getLinkedPath(std::size_t index=0) const = 0;

    /// @name Serialization API
    /// @{

    /// Read the command line
    virtual bool read( const std::string& str ) = 0;

    /// Update pointers in case the pointed-to objects have appeared
    /// @return false if there are broken links
    virtual bool updateLinks() = 0;

    /// Print the value of the associated variable
    virtual void printValue( std::ostream& ) const;

    /// Print the value of the associated variable
    virtual std::string getValueString() const;

    /// Print the value type of the associated variable
    virtual std::string getValueTypeString() const;

    /// @}

    /// @name Serialization Helper API
    /// @{

    static bool ParseString(const std::string& text, std::string* path, std::string* data = nullptr, Base* start = nullptr);

    bool parseString(const std::string& text, std::string* path, std::string* data = nullptr) const
    {
        return ParseString(text, path, data, this->getOwnerBase());
    }

    static std::string CreateString(const std::string& path, const std::string& data="");
    static std::string CreateStringPath(Base* object, Base* from);
    static std::string CreateStringData(BaseData* data);
    static std::string CreateString(Base* object, Base* from);
    static std::string CreateString(BaseData* data, Base* from);
    static std::string CreateString(Base* object, BaseData* data, Base* from);

    /// @}
    ///
    SOFA_END_DEPRECATION_AS_ERROR
protected:
    unsigned int m_flags;
    std::string m_name;
    std::string m_help;
    /// Number of changes since creation
    int m_counter;
    void updateCounter()
    {
        ++m_counter;
    }
};

} // namespace objectmodel

} // namespace core

// the BaseLink class is used everywhere
using core::objectmodel::BaseLink;

} // namespace sofa

#endif
