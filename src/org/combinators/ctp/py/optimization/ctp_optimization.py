#!/usr/bin/python
import asyncio
import concurrent
import sys
from functools import partial, partialmethod
from hypermapper import optimizer

sys.path.append('/home/tristan/projects/ctp_py/src')

from org.combinators.ctp.py.optimization import mqttClient

import uuid
import paho.mqtt.client as mqtt

use_simplification = True

schedule_file = f"/home/tristan/projects/ctp_py/src/org/combinators/ctp/py/optimization/schedule.txt"
number_of_instances = 10
async_execution = "true"
write_path_files = "true"

bm_conf_string = f"""{{"iterations" : {number_of_instances}, "async" : {async_execution}, "maxTime" : 200, "maxMem" : 
1024, "writePathFiles" : {write_path_files} }} """

planner_string_list = ["sbmp_planner_PRM", "sbmp_planner_PRMStar", "sbmp_planner_LazyPRM",
                       "sbmp_planner_LazyPRMStar", "sbmp_planner_SST", "sbmp_planner_RRT", "sbmp_planner_RRTStar",
                       "sbmp_planner_LBTRRT", "sbmp_planner_TRRT", "sbmp_planner_LazyRRT",
                       "sbmp_planner_RRTConnect", "sbmp_planner_EST", "sbmp_planner_SBL", "sbmp_planner_LBKPIECE1",
                       "sbmp_planner_KPIECE1", "sbmp_planner_BKPIECE1",
                       "sbmp_planner_STRIDE", "sbmp_planner_PDST", "sbmp_planner_FMT", "sbmp_planner_BFMT",
                       "sbmp_planner_RRTsharp", "sbmp_planner_RRTXstatic", "sbmp_planner_InformedRRTstar",
                       "sbmp_planner_BITstar"]



sampler_string_list = ["sbmp_uniform_valid_state_sampler", "sbmp_obstacle_valid_state_sampler",
                       "sbmp_gaussian_valid_state_sampler", "sbmp_max_clearance_valid_state_sampler",
                       "sbmp_uniform_space_sampler", "sbmp_gaussian_space_sampler"]
state_validator_string_list = ["sbmp_fcl_validator", "sbmp_fcl_wafr_validator"]
motion_validator_string_list = ["sbmp_discrete_motion_validator", "sbmp_fcl_motion_validator"]
optimization_objective_list = ["sbmp_opt_path_length", "sbmp_opt_integral"]
cost_list = ["sbmp_default_cost_state", "sbmp_cost_state_change_weighted"]
simplification_list = ["sbmp_use_simplification", "sbmp_no_simplification"]
dimensionality_list = ["dimensionality_two_d_t", "dimensionality_three_d_t", "dimensionality_n_d_t"]


def get_planner_string(i):
    return planner_string_list[i]


def get_sampler_string(i):
    return sampler_string_list[i]


def get_state_validator_string(i):
    return state_validator_string_list[i]


def get_motion_validator_string(i):
    return motion_validator_string_list[i]


def get_optimization_objective_string(i):
    return optimization_objective_list[i]


def get_cost_string(i):
    return cost_list[i]


def get_cost_string(i):
    return simplification_list[i]


def get_dimensionality_string(i):
    return dimensionality_list[i]


def get_invalid_output_data():
    output = {'pathlength': 5000000.0, 'computationtime': 5000000.0, 'failures': float(number_of_instances),
              'Valid': "false"}
    return output


def ctp_function(X, problem_instance, state_validator=state_validator_string_list[0]):
    client = mqtt.Client()
    client.on_connect = mqttClient.on_connect
    print("trying to connect")
    client.connect("koopa.cs.tu-dortmund.de", 1883)
    print("Done Connecting")

    planner = X['planner']
    sampler = X['sampler']
    motion_validator = X['motionValidator']
    # simplification_time = X['simplification_time']
    planning_time = X['planning_time']

    new_uuid = uuid.uuid4()
    json_string = f"""{{
"planner" : "{planner_string_list[planner]}",
"sampler" : "{get_sampler_string(sampler)}",
"stateValidator" : "{state_validator}",
"motionValidator" : "{get_motion_validator_string(motion_validator)}",
"costs" : "not_specified",
"optObjective" : "not_specified",
"simplification" : "sbmp_use_simplification",
"sceneInput" : "scene_input_data_file",
"dimensionality" : "dimensionality_three_d_t",
"id" : "{new_uuid}",
"configurableAlg" : true,
"withStates" : false
}}
"""

    mqttClient.send_message(client, json_string, "bmInitRequest")
    loop = asyncio.get_event_loop()
    f = loop.create_future()
    # loop.create_task(mqttRequest(client, json_string, fooo[0]))
    print("Setting new on_message")
    client.on_message = partial(mqttClient.on_init_message, future=f, asyncloop=loop)
    print(f"Subscribing to topic bmInitResponse.{new_uuid}")
    client.subscribe(f"bmInitResponse.{new_uuid}", 2)
    client.loop_start()
    print("Waiting for init request to complete.")
    loop.run_until_complete(f)
    print("Future completed. Stopping mqtt loop")
    client.loop_stop()

    if f.result() == 1:  # Success
        bm_start_response_topic = f"bmStartResponse.{new_uuid}"
        bm_generic_input_response_topic = f"bmGenericInputResponse.{new_uuid}"
        bm_result_topic = f"bmResult.{new_uuid}"

        print(f"Subscribing to {bm_result_topic}")
        print(f"Subscribing to {bm_start_response_topic}")
        print(f"Subscribing to {bm_generic_input_response_topic}")
        client.subscribe(bm_result_topic, 2)
        client.subscribe(bm_start_response_topic, 2)
        client.subscribe(bm_generic_input_response_topic, 2)

        loop4 = asyncio.get_event_loop()
        f_generic_input_response = loop4.create_future()
        client.on_message = partial(mqttClient.on_input_message, future=f_generic_input_response, asyncloop=loop4)
        client.loop_start()
        mqttClient.send_message(client,
                                f"""["{problem_instance}.cfg", "--computationtime={planning_time} --simplificationtime=2.0"]""",
                                f"bmGenericInput.{new_uuid}")
        try:
            print(f"Waiting for bmGenericInputResponse {new_uuid} ...")
            loop4.run_until_complete(asyncio.wait_for(f_generic_input_response, 10.0))
        except Exception as e:
            client.loop_stop()
            output = get_invalid_output_data()
            print(f"TimeoutError for bmGenericInputResponse.{new_uuid}, returning output {output}")
            client.disconnect()
            return output

        loop3 = asyncio.get_event_loop()
        f_start_response = loop3.create_future()
        client.on_message = partial(mqttClient.on_start_message, future=f_start_response, asyncloop=loop3)
        print(f"""Sending conf string: {bm_conf_string}""")
        print(f"Sending conf string: {bm_conf_string}")
        mqttClient.send_message(client,
                                f"""{bm_conf_string}""",
                                f"bmStartRequest.{new_uuid}")

        try:
            print(f"Waiting for bmStartRequest.{new_uuid} ...")
            loop3.run_until_complete(asyncio.wait_for(f_start_response, 10.0))
        except Exception as e:
            client.loop_stop()
            output = get_invalid_output_data()
            print(f"TimeoutError for bmStartRequest.{new_uuid}, returning output {output}")
            client.disconnect()
            return output

        print("Messages sent. Waiting for result.")
        loop2 = asyncio.get_event_loop()
        f2 = loop2.create_future()
        client.on_message = partial(mqttClient.on_result_message, future=f2, asyncloop=loop2)
        loop2.run_until_complete(f2)
        client.loop_stop()

        print("Received result.")
        output = f2.result()
    else:
        print("No inhabitant found.")
        output = get_invalid_output_data()

    client.disconnect()
    print(f"End of iteration. Output: {output}")
    return output


def printIndices():
    print(f"{[i for i, x in enumerate(sampler_string_list)]}")
    print(f"{[i for i, x in enumerate(motion_validator_string_list)]}")
    print(f"{[i for i, x in enumerate(optimization_objective_list)]}")
    print(f"{[i for i, x in enumerate(cost_list)]}")
    print(f"{[i for i, x in enumerate(simplification_list)]}")
    print(f"{[i for i, x in enumerate(dimensionality_list)]}")


if __name__ == "__main__":
    print("Starting Main")
    import argparse

    parser = argparse.ArgumentParser(description='ctp optimization')
    parser.add_argument('--problem', default="Abstract", help='name of the problem instance', type=str)

    args = parser.parse_args()
    with open(schedule_file, 'r') as fin:
        data = fin.read().splitlines(True)
    with open(schedule_file, 'w') as fout:
        fout.writelines(data[1:])

    main_problem_instance = data[0] if data else args.problem
    print(f"Problem instance: {main_problem_instance}")
    ctp_fun = partial(ctp_function, problem_instance=main_problem_instance,
                      state_validator=state_validator_string_list[1]) if 'wafr' in main_problem_instance else partial(
        ctp_function, problem_instance=main_problem_instance)
    parameters_file = f"/home/tristan/projects/ctp_py/src/org/combinators/ctp/py/optimization/ctp_scenario_{main_problem_instance}.json"
    optimizer.optimize(parameters_file, ctp_fun)

    print("End of ctp optimization.")
