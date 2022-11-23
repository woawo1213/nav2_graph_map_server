# nav2_graph_map_server

### Map 파일 형식 

- YAML format example 

```
nodes:
- id: 0
  pose:
    orientation:
      w: 1.0
      x: 0.0
      y: 0.0
      z: 0.0
    position:
      x: 0.0
      y: 0.0
      z: 0.0
  station_name: ST0  # <- 이름이 있으면 Station Node다 
- id: 1
  pose: 
    position:
      x: 0.1
      y: 0.0
      z: 0.0
  station_name: null # <- 이름이 없으면 일반 Node 
- id: 2
  pose: 
    position:
      x: 0.2
      y: 0.0
      z: 0.0
  station_name: null
- id: 3
  pose: 
    position:
      x: 0.3
      y: 0.0
      z: 0.0
  station_name: null
- id: 4
  pose:
    orientation:
      w: 1.0
      x: 0.0
      y: 0.0
      z: 0.0
    position:
      x: 0.0
      y: 0.0
      z: 0.0
  station_name: ST1
edges:
- id_0: 0
  id_1: 1
  speed: 0.2
- id_0: 1
  id_1: 2
  speed: 0.2
- id_0: 2
  id_1: 3
  speed: 0.2
- id_0: 3
  id_1: 4
  speed: 0.2
- id_0: 4
  id_1: 3
  speed: 0.3
```


### Parameters 

- yaml_filename 
    - map 파일 경로 



### Published Topics 

- graph_map(nav2_graph_map_msgs/GraphMap)
