import json
import time

def remove_dups(list, comp):
    new_list = []
    for item in list:
        if comp(item) not in [comp(x) for x in new_list]:
            new_list.append(item)
    return new_list


def findMax(list):
    return max(list, key=len)

# STATE = {
#     "hazmat_running": False,
#     "run_hazmat": False,
#     "quit": False,
#     "clear_all_found": False,
#     "hazmat_delta": 1 / 10,
#     "frame_count": 0,
# }

def write_state(state, path):
    with open(path, "w+") as f:
        json.dump(state, f, indent=4)

def read_state(path):
    # try:
    with open(path, "r") as f:
        j = json.load(f)

        j["hazmat_running"] = bool(j["hazmat_running"])
        j["run_hazmat"] = bool(j["run_hazmat"])
        j["quit"] = bool(j["quit"])
        j["clear_all_found"] = bool(j["clear_all_found"])
        j["hazmat_delta"] = float(j["hazmat_delta"])
        j["frame_count"] = int(j["frame_count"])
        
        return j
    # except:
    #     time.sleep(0.02)
    #     return read_state(path)