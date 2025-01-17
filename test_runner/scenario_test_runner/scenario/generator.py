from jinja2 import Environment, BaseLoader
import yaml
import datetime

template = """
OpenSCENARIO:
  FileHeader:
    author: 'Generated'
    date: '{{ current_date }}'
    description: 'Sample scenario (with Autoware)'
    revMajor: 1
    revMinor: 0
  ParameterDeclarations:
    ParameterDeclaration: []
  CatalogLocations:
    VehicleCatalog:
      Directory:
        path: $(ros2 pkg prefix --share openscenario_experimental_catalog)/vehicle
  RoadNetwork:
    LogicFile:
      filepath: $(ros2 pkg prefix --share simple_map)/map
  Entities:
    ScenarioObject:
      - name: npc0
        CatalogReference: &sample_vehicle
          catalogName: sample_vehicle
          entryName: sample_vehicle
      {% for i in range(num_npcs) %}
      - name: npc{{ i + 1 }}
        CatalogReference: *sample_vehicle
      {% endfor %}
  Storyboard:
    Init:
      Actions:
        Private:
          - entityRef: npc0
            PrivateAction:
              - TeleportAction:
                  Position:
                    LanePosition:
                      roadId: ''
                      laneId: 1
                      s: 1
                      offset: 0
                      Orientation: &DEFAULT_ORIENTATION
                        type: relative
                        h: 0
                        p: 0
                        r: 0
          {% for i in range(num_npcs) %}
          - entityRef: npc{{ i + 1 }}
            PrivateAction:
              - TeleportAction:
                  Position:
                    LanePosition:
                      roadId: ''
                      laneId: {{ i + 2 }}
                      s: 1
                      offset: 0
                      Orientation: *DEFAULT_ORIENTATION
          {% endfor %}
    Story:
      - name: ''
        Act:
          - name: _EndCondition
            ManeuverGroup:
              - maximumExecutionCount: 1
                name: ''
                Actors:
                  selectTriggeringEntities: false
                  EntityRef:
                    - entityRef: npc0
                Maneuver:
                  - name: ''
                    Event:
                      - name: ''
                        priority: parallel
                        StartTrigger:
                          ConditionGroup:
                            - Condition:
                                - name: ''
                                  delay: 0
                                  conditionEdge: none
                                  ByEntityCondition:
                                    TriggeringEntities:
                                      triggeringEntitiesRule: any
                                      EntityRef:
                                        - entityRef: npc0
                                    EntityCondition:
                                      ReachPositionCondition:
                                        Position:
                                          LanePosition:
                                            roadId: ''
                                            laneId: 1
                                            s: 200
                                            offset: 0
                                            Orientation: *DEFAULT_ORIENTATION
                                        tolerance: 0.5
                        Action:
                          - name: ''
                            UserDefinedAction:
                              CustomCommandAction:
                                type: exitSuccess
                      - name: ''
                        priority: parallel
                        StartTrigger:
                          ConditionGroup:
                            - Condition:
                                - name: ''
                                  delay: 0
                                  conditionEdge: none
                                  ByValueCondition:
                                    SimulationTimeCondition:
                                      value: 180
                                      rule: greaterThan
                        Action:
                          - name: ''
                            UserDefinedAction:
                              CustomCommandAction:
                                type: exitFailure
            StartTrigger:
              ConditionGroup:
                - Condition:
                    - name: ''
                      delay: 0
                      conditionEdge: none
                      ByValueCondition:
                        SimulationTimeCondition:
                          value: 0
                          rule: greaterThan
    StopTrigger:
      ConditionGroup: []
"""

def generate_scenario(num_npcs):
    env = Environment(loader=BaseLoader())
    template_obj = env.from_string(template)

    current_date = datetime.datetime.now().isoformat()
    scenario = template_obj.render(num_npcs=num_npcs, current_date=current_date)

    # Parse and format as YAML
    yaml_obj = yaml.safe_load(scenario)
    return yaml.dump(yaml_obj, allow_unicode=True, sort_keys=False)

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description='scenario generator')
    parser.add_argument('num_npcs', type=int, help='Number of NPC vehicles to generate')
    parser.add_argument('--output', '-o', default=None, help='Output filename (default: scenario_<num_npcs>npcs.yaml)')

    args = parser.parse_args()

    scenario = generate_scenario(args.num_npcs)

    output_file = args.output if args.output else f'scenario_{args.num_npcs}npcs.yaml'

    with open(output_file, 'w') as f:
        f.write(scenario)

    print(f"Generated scenario with {args.num_npcs} NPCs in {output_file}")
