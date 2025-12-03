Create a file `<MESH_IDENTITY>.labels` in this directory and assing matching labels. These labels will be read from the file by the leader (`docker_leader` service) when a node joins a swarm and is subsequently assigned to the node using `docker node update --label-add <label>`. Labels can also be added or removed manually. This list contains the labels we use for our essential services and testing, but new labels can be introduced at will.

These labels will be used to put constraints on docker swarm services to limit which nodes can start them. Note that constraints either (a) check if a label exists, (b) check if the label's value equals another value (`==`), or (c) check if the label's value does not equal another value (`!=`). Multiple constraints are connected with a logical AND, meaning all have to be true for a service to be started on a node.

### Data Sink (only use for one host)
- has_data_sink
- has_web
- has_2d_map_data
- has_3d_map_data
- has_db
- has_backup_db

### Managers
- can_become_manager

### Leader
- supposed_leader
- is_leader - currently not used as it is not reassigned if the leader changes

### Hardware
- architecture=x86_64
- cpus=8 - example for 8 CPUs
- ram_ge4 - "RAM is greater equal 4 GB"
- ram_ge8 - "RAM is greater equal 8 GB"
- ram_ge16 - "RAM is greater equal 16 GB"
- ram_lt4 - "RAM is less than 4 GB"
- ram_lt8 - "RAM is less than 8 GB"
- ram_lt16 - "RAM is less than 16 GB"
- has_gpu

> Note: It is suggested that if a node receives for instance ram_ge16, it is also given ram_ge8 and ram_ge4 as a service might only require ram_ge8.

### Specific Selection
- group_a - used to select a node or group of nodes for specific tasks or tests
- group_b
- group_c
- group_d
- group_e
- group_f
- group_g
- group_h

### User-Defined
Add custom labels here