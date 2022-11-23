import yaml
import sys
import ezdxf
import numpy as np
import csv
import math

"""
.dxf 파일의 POINT를 읽어 graph_map_server가 받는 yaml파일로 변경

To-Do
dxf 파일명 커멘드로 읽기
yaml 파일명 커멘드로 읽기
"""


def dxf_to_csv():
    doc = ezdxf.readfile("/home/jm/map/wonik_4th_floor_re.dxf")
    msp = doc.modelspace()

    nodes = []
    for e in msp.query("POINT"):
        x = e.dxf.location[0]
        y = e.dxf.location[1]
        nodes.append((x, y))

    x_p, y_p = zip(*nodes)

    node_trans = []

    plan_width = 4000
    plan_height = 4000
    sx = (-plan_width) / 2
    sy = (-plan_height) / 2

    H = [[1, 0, 0, sx], [0, 1, 0, sy], [0, 0, 1, 0], [0, 0, 0, 1]]

    H = np.array(H, dtype=np.float64)

    scale = 0.05
    node_trans.append(x_p)
    node_trans.append(y_p)
    node_trans = np.array(node_trans)

    node_trans = np.vstack([node_trans, np.zeros(node_trans.shape[1])])
    node_trans = np.vstack([node_trans, np.ones(node_trans.shape[1])])

    node_list = scale * H @ node_trans
    node_list = np.delete(node_list, -1, 0)
    node_list = np.delete(node_list, -1, 0)
    node_list = np.stack((node_list[0], node_list[1]), axis=1)
    np.savetxt("nodes.csv", node_list, delimiter=",")


def create_position(x, y, z):

    return {
        "x": float(x),
        "y": float(y),
        "z": float(z),
    }


def create_orientation(x, y, z, w):
    return {
        "x": float(x),
        "y": float(y),
        "z": float(z),
        "w": float(w),
    }


def create_node(id, position, orientation, station_name=None):
    return {
        "id": id,
        "station_name": station_name,
        "pose": {"position": position, "orientation": orientation},
    }


def create_edge(start_node_id, end_node_id, speed):
    return {
        "id_s": start_node_id,
        "id_e": end_node_id,
        "speed": speed,
    }


def example():
    f = open("nodes.csv", "r")
    rdr = csv.reader(f)

    nodes = []
    index = 0
    for i in rdr:
        float_x = float(i[0])
        float_y = float(i[1])
        nodes.append(
            create_node(
                index,
                create_position(round(float_x, 4), round(float_y, 4), 0.0),
                create_orientation(0.0, 0.0, 0.0, 1.0),
            )
        )
        index += 1

    edges = []
    for i in range(0, 137):
        edges.append(create_edge(i, i + 1, 0.5))
        edges.append(create_edge(i + 1, i, 0.5))

    map = {
        "nodes": nodes,
        "edges": edges
    }
    # print(map)

    with open('gmap.yaml','w') as f:
        yaml.dump(map,f,sort_keys=False)
    # print(yaml.dump(map,sort_keys=False))


if __name__ == "__main__":
    dxf_to_csv()
    example()