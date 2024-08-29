/*
 * Copyright 2019, Proyectos y Sistemas de Mantenimiento SL (eProsima).
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef EPROSIMA_XTYPES_MUTABLE_COLLECTION_TYPE_HPP_
#define EPROSIMA_XTYPES_MUTABLE_COLLECTION_TYPE_HPP_

#include <xtypes/CollectionType.hpp>

namespace eprosima {
namespace xtypes {

/// \brief DynamicType representing mutable collection of elements.
/// This is the base abstract class for all mutable collections.
/// This class specifier the CollectionType class for collection that
/// can change os size during their lives.
/// A MutableCollectionType represents a TypeKind::COLLECTION_TYPE.
class MutableCollectionType : public CollectionType
{
public:
    /// \brief Get the bounds of the collection.
    /// \returns The bounds of the collection. 0 value means that no bounds were specified.
    uint32_t bounds() const { return bounds_; }

protected:
    MutableCollectionType(
            TypeKind kind,
            const std::string& name,
            DynamicType::Ptr&& content,
            uint32_t bounds)
        : CollectionType(kind, name, std::move(content))
        , bounds_(bounds)
    {}

private:
    uint32_t bounds_;
};

} //namespace xtypes
} //namespace eprosima

#endif //EPROSIMA_XTYPES_MUTABLE_COLLECTION_TYPE_HPP_
