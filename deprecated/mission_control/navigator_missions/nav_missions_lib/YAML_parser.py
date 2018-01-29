from txros import util
from mil_misc_tools.text_effects import fprint
from twisted.internet import defer
from mission import Mission


@util.cancellableInlineCallbacks
def yaml_parse(yaml_text, navigator, total_time):
    my_missions = {}
    mission_to_mission_dep = {}
    count = 0
    base_mission = None
    missions_left = []
    tree = []

    for name in yaml_text.keys():
        mission = yaml_text[name]
        marker_dep = mission["marker_dep"]
        mis_dep = mission["mission_dep"]
        is_base = mission["is_base"]
        min_time = mission["min_time"]
        points = mission["points"]
        looking_for = mission["looking_for"]
        weight = mission["weight"]
        if marker_dep != "None":
            marker = yield navigator.database_query(marker_dep, raise_exception=False)
            if not marker.found:
                fprint("NOT COMPLETING {}, NO MARKER FOUND".format(name), msg_color="green")
                continue
            marker = marker.objects[0]
        else:
            marker = None

        num_mission_deps = len(mis_dep)
        if mis_dep == "None":
            num_mission_deps = 0
        if type(mis_dep) is not list:
            num_mission_deps = 1

        if "mission_script" in mission.keys():
            mission_script = mission["mission_script"]
            m = Mission(name, marker, min_time, weight, points, looking_for, num_mission_deps, mission_script=mission_script)
        else:
            m = Mission(name, marker, min_time, weight, points, looking_for, num_mission_deps)
        my_missions[name] = m

        if is_base:
            base_mission = m
        elif mis_dep == "None":
            count += 1
            missions_left.append(m)
            tree.append(m)
        else:
            count += 1
            missions_left.append(m)
            mission_to_mission_dep[name] = mis_dep

    # Go through the missions and give them all children dependencies
    for mission_name in mission_to_mission_dep.keys():
        parent_name = mission_to_mission_dep[mission_name]
        if type(parent_name) is not list:
            parent_names = []
            parent_names.append(parent_name)
        else:
            parent_names = parent_name
        for p in parent_names:
            child_name = mission_name
            parent_mission = my_missions[p]
            child_mission = my_missions[child_name]
            parent_mission.add_child(child_mission)

    total_time -= count * 60
    defer.returnValue((missions_left, base_mission, tree, total_time))
