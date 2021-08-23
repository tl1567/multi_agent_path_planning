"""

Extension of SIPP to multi-robot scenarios for the Multi-Robot Warehouse Environment (RWARE) 

Author: Tim Tsz-Kit Lau (@timlautk)

Based on the implementation of Ashwin Bose (@atb033) for CBS, and Filippos Christianos and Lukas Schafer for RWARE


See the article: 10.1109/ICRA.2011.5980306

"""

import argparse
import yaml
from math import fabs
from graph_generation import SippGraph, State
from sipp_rware import SippPlanner

import numpy as np
import robotic_warehouse
from robotic_warehouse.warehouse import Warehouse, RewardType
import gym


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("param", help="input file containing map and dynamic obstacles")
    parser.add_argument("output", help="output file with the schedule")
    parser.add_argument("env_name", help="environment name from RWARE")
    
    args = parser.parse_args()
    
    ## Read Map
    # with open(args.param, 'r') as param_file:
    #     try:
    #         param = yaml.load(map_file, Loader=yaml.FullLoader)
    #     except yaml.YAMLError as exc:
    #         print(exc)

    # warehouse = Warehouse(3, 3, 2, 10, 3, 3, 10, None, None, RewardType.GLOBAL)
    warehouse = gym.make(args.env_name)
    warehouse.reset()
    ## a list
    # dimension = param["map"]["dimensions"]
    dimension = list(warehouse.grid_size)
    # print(dimension)

    ## a list of coordinates in tuples (x, y)
    # obstacles = param["map"]["obstacles"]
    # obstacles = [(0, 0), (5, 5), (8, 9)]
    obstacles = []
    # print(obstacles)

    dynamic_obstacles = {}

    ## a list of dictionaries
    ## {'start': coordinates (list), 'goal': coordinates (list), 'name': 'agent{id}'}
    # agents = param['agents']    
    agents_loc = [[agent.y.item(), agent.x.item()] for agent in warehouse.agents]
    goals = [[shelf.y.item(), shelf.x.item()] for shelf in warehouse.request_queue]
    # agents_loc = [[agent.x.item(), agent.y.item()] for agent in warehouse.agents]
    # goals = [[shelf.x.item(), shelf.y.item()] for shelf in warehouse.request_queue]


    ## Shelf requesting the closest agent to pick it up
    def compute_dist_agents_goals(agents_loc, goals):
        dist = [[abs(agents_loc[i][0] - goals[j][0]) + abs(agents_loc[i][1] - goals[j][1]) for j in range(len(goals))] for i in range(len(agents_loc))]
        return dist

    dist = compute_dist_agents_goals(agents_loc, goals)

    def compute_dist_argmins(dist):
        dist_argmins = [np.argmin(dist[j]) for j in range(len(goals))]
        return dist_argmins

    dist_argmins = compute_dist_argmins(dist)

    ## solve recursively the goal of each agent (the agent with the minimum distance to a goal will be assigned with that goal;
    ## both the agent and the goal will be removed from the queues, and this is done recursively)
    def assign_goal_to_agent(agents_loc, goals):
        dist = compute_dist_agents_goals(agents_loc, goals)
        dist = np.array(dist)
        ind = np.unravel_index(np.argmin(dist, axis=None), dist.shape)
        agents.append({'start': agents_loc[ind[0]], 'goal': goals[ind[1]], 'name': f'agent{len(agents)}'})
        del agents_loc[ind[0]]
        del goals[ind[1]]
        return agents_loc, goals

    
    if len(set(np.argmin(dist, axis=1))) == len(np.argmin(dist, axis=1)):
        goals = [goals[i] for i in dist_argmins]
        names = [f'agent{i}' for i in range(warehouse.n_agents)]
        agents = [{'start': agents_loc[i], 'goal': goals[i], 'name': names[i]} for i in range(len(agents_loc))]
    else: 
        agents = []
        while len(agents_loc):
            agents_loc, goals = assign_goal_to_agent(agents_loc, goals)
    


    ## Write to input file 
    param = {}
    param["map"] = {"dimensions": dimension, "obstacles": obstacles}
    param["agents"] = agents
    param["dynamic_obstacles"] = dynamic_obstacles

    with open(args.param, 'w') as param_file:
        yaml.dump(param, param_file)

    ## Output file
    # with open(args.output, 'r') as output_yaml:
    #     try:
    #         output = yaml.load(output_yaml, Loader=yaml.FullLoader)
    #     except yaml.YAMLError as exc:
    #         print(exc)

    output = {"schedule": {f'agent{i}': [] for i in range(warehouse.n_agents)}}

    for i in range(len(param["agents"])):
        sipp_planner = SippPlanner(param, i)
    
        if sipp_planner.compute_plan():
            plan = sipp_planner.get_plan()
            output["schedule"].update(plan)
            param["dynamic_obstacles"].update(plan)

            with open(args.output, 'w') as output_yaml:
                yaml.safe_dump(output, output_yaml)  
        else: 
            print("Plan not found")


if __name__ == "__main__":
    main()
