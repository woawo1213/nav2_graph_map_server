import yaml


def create_position(x, y, z):
    return {
        'x' : float(x), 
        'y' : float(y), 
        'z' : float(z),
    }

def create_orientation(x, y, z, w):
    return {
        'x' : float(x), 
        'y' : float(y), 
        'z' : float(z),
        'w' : float(w), 
    }

def create_node(id, position, orientation, station_name=None):
    return {
        'id' : id, 
        'station_name' : station_name,
        'pose' : {
            'position' : position, 
            'orientation': orientation
        },
    }

def create_edge(start_node_id, end_node_id, speed):
    return {
        'id_0' : start_node_id,
        'id_1' : end_node_id,
        'speed' : speed, 
    }


def example():
    
    nodes = [ 
        create_node(0, create_position(0.0, 0.0, 0.0), create_orientation(0.0, 0.0, 0.0, 1.0), 'ST0'),
        create_node(1, create_position(0.1, 0.0, 0.0), create_orientation(0.0, 0.0, 0.0, 1.0)),
        create_node(2, create_position(0.2, 0.0, 0.0), create_orientation(0.0, 0.0, 0.0, 1.0)),
        create_node(3, create_position(0.3, 0.0, 0.0), create_orientation(0.0, 0.0, 0.0, 1.0)),
        create_node(4, create_position(0.0, 0.0, 0.0), create_orientation(0.0, 0.0, 0.0, 1.0), 'ST1'),
    ]
    
    edges = [
        create_edge(0, 1, 0.2),
        create_edge(1, 2, 0.2),
        create_edge(2, 3, 0.2),
        create_edge(3, 4, 0.2),
        create_edge(4, 3, 0.3),
    ]
    
    map = {
        'nodes' : nodes, 
        'edges' : edges,
    }

    print(yaml.dump(map))


if __name__ == '__main__':
    example()

