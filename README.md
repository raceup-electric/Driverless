# How to test graph slam node

First thing you need to have four separate terminal windows, it is suggested the use of "Terminator". In your simulation folder you need to have the graph_based_slam folder, it is assumed that all the packages compile properly.


1. Select the simulation folder for all terminals: ```cd path_to_your_sim_folder``` 
2. For all the terminals: ```source install/setup.bash```
3. In the first terminal run: ```ros2 run graph_based_slam graph_visualizer ```
4. In the second run: ```ros2 run graph_based_slam incremental_graph_SLAM```
5. In the third run: ```rviz2```. This will start rviz application in its window you need to click the button "add" on the bottom-left. This will open a menu where on top you need to select "by topic" window. Here you need to click on the "graph_viz->Marker array" option. Then click OK.
6. Run the bags with ```ros2 bag play path_to_your_bag/your_bag.db3```
7. At this point you should see appearing in the rviz simulation green balls representing the car estimated pose and red balls representing cones positions.

