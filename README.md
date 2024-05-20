- Install Webots and clone the Repo
- Open the mapf.wbt world from the Worlds directory
- Launch the 4 controllers as extern controllers by adding the following aliases to bashrc :
export WEBOTS_HOME=/usr/local/webots
alias admin='$WEBOTS_HOME/webots-controller --robot-name=Admin /home/navin/catkin_ws/src/Multi-Agent-Path-Finding-Motion-Planning/Webots_Scripts/admin.py'
alias agent1='$WEBOTS_HOME/webots-controller --robot-name=Agent1 /home/navin/catkin_ws/src/Multi-Agent-Path-Finding-Motion-Planning/Webots_Scripts/agent1.py'
alias agent2='$WEBOTS_HOME/webots-controller --robot-name=Agent2 /home/navin/catkin_ws/src/Multi-Agent-Path-Finding-Motion-Planning/Webots_Scripts/agent2.py'
alias agent3='$WEBOTS_HOME/webots-controller --robot-name=Agent3 /home/navin/catkin_ws/src/Multi-Agent-Path-Finding-Motion-Planning/Webots_Scripts/agent3.py'
- change the controller path according to the file you want to use, agent<agentnum> for vanilla rrt* and path tracking, agent<agentname>_rvo for windowless RVO,
  agent<agent_num>_local for windowed RVO
- execute the 4 aliases in 4 instances of the terminal window, voila!
