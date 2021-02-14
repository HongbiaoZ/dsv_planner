# graph_planner

README is a work in progress

Graeme's todo list:

* world abstraction needs to correctly update "unvisited openings" when a waypoint or nearby waypoints is revisited. this needs to happen in all modes, even when paused.
* world abstraction needs to correctly label "unvisited openings" when polygons change
* dfs_behavior_planner needs to correctly bias towards the polygons (use misc_utils::DistancePoint2DToPolygon), not a single bias point
* dfs_behavior_planner needs to differentiate between "backtracking due to dead end" and "backtracking due to timer/operator etc.", since we only want graph_planner to override the first case
* testing single robot case
* testing with polygons
* testing multi-robot case with polygons
* test hysteresis for control handover between dfs and graph_planner... is there corner cases that require more thought?



longer term

* the current method of handing over control back and forth with service calls kinda sucks... 
* world_abstraction that is truly modular and not tightly coupled with dfs_behavior_planner
* share graphs between multiple robots, so that they go straight to their polygon (e.g. ugv sees something that operator wants to sent uav to visit).

## Dependencies

## Usage

Follow all instructions in the dfs_behavior_package

Then additionally run the following:
```bash
roslaunch graph_planner graph_planner.launch
```

## Who do I talk to?
Graeme Best (bestg@oregonstate.edu)
