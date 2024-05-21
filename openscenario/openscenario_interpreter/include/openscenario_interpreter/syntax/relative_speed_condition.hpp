// Copyright 2015 TIER IV, Inc. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__RELATIVE_SPEED_CONDITION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__RELATIVE_SPEED_CONDITION_HPP_

#include <geometry/vector3/norm.hpp>
#include <openscenario_interpreter/scope.hpp>
#include <openscenario_interpreter/simulator_core.hpp>
#include <openscenario_interpreter/syntax/directional_dimension.hpp>
#include <openscenario_interpreter/utility/print.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/*
   RelativeSpeedCondition 1.3

   The current relative speed of a triggering entity/entities to a reference
   entity is compared to a given value. The logical operator used for the
   evaluation is defined by the rule attribute. If direction is used, only the
   projection to that direction is used in the comparison, with the triggering
   entity/entities as the reference.

   <xsd:complexType name="RelativeSpeedCondition">
     <xsd:attribute name="entityRef" type="String" use="required"/>
     <xsd:attribute name="rule" type="Rule" use="required"/>
     <xsd:attribute name="value" type="Double" use="required"/>
     <xsd:attribute name="direction" type="DirectionalDimension"/>
   </xsd:complexType>
*/
struct RelativeSpeedCondition : private SimulatorCore::ConditionEvaluation
{
  /*
     Reference entity.
  */
  const EntityRef entity_ref;

  /*
     The operator (less, greater, equal).
  */
  const Rule rule;

  /*
     Relative speed value. Unit: [m/s]. Range: ]-inf..inf[. Relative speed is
     defined as speed_rel = speed(triggering entity) - speed(reference entity)
  */
  const Double value;

  /*
     Direction of the speed (if not given, the total speed is considered).
  */
  const std::optional<DirectionalDimension> direction;

  const TriggeringEntities triggering_entities;

  std::vector<Double> evaluations;

  explicit RelativeSpeedCondition(
    const pugi::xml_node & node, Scope & scope, const TriggeringEntities & triggering_entities)
  : entity_ref(readAttribute<EntityRef>("entityRef", node, scope)),
    rule(readAttribute<Rule>("rule", node, scope)),
    value(readAttribute<Double>("value", node, scope)),
    direction(readAttribute<DirectionalDimension>("direction", node, scope, std::nullopt)),
    triggering_entities(triggering_entities),
    evaluations(triggering_entities.entity_refs.size(), Double::nan())
  {
  }

  auto description() const
  {
    auto description = std::stringstream();

    description << triggering_entities.description() << "'s relative ";

    if (direction) {
      description << *direction << " ";
    }

    description << "speed to given entity " << entity_ref << " = ";

    print_to(description, evaluations);

    description << " " << rule << " " << value << "?";

    return description.str();
  }

  auto evaluate()
  {
    evaluations.clear();

    return asBoolean(triggering_entities.apply([this](auto && triggering_entity) {
      evaluations.push_back([this](auto && v) {
        if (direction) {
          switch (*direction) {
            case DirectionalDimension::longitudinal:
              return v.x;
            case DirectionalDimension::lateral:
              return v.y;
            case DirectionalDimension::vertical:
              return v.z;
            default:
              return math::geometry::norm(v);
          }
        } else {
          return math::geometry::norm(v);
        }
      }(evaluateRelativeSpeed(triggering_entity, entity_ref)));
      return std::invoke(rule, evaluations.back(), value);
    }));
  }
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__RELATIVE_SPEED_CONDITION_HPP_