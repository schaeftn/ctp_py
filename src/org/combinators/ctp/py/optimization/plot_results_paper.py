import pandas as pd

from os.path import isfile, join
from os import walk
import os.path
import sys
import plotly.express as px
import plotly.graph_objects as go
import numpy as np

from org.combinators.ctp.py.optimization.plot_utils import get_new_pareto_plot_file_name

local_machine_path = 'C:\\workspace_py\\ctp_py\\src'
koopa_path = "/home/tristan/projects/ctp_py/src"

sys.path.append(local_machine_path)

from org.combinators.ctp.py.optimization.ctp_optimization import get_planner_string, get_sampler_string, \
    get_motion_validator_string

overwrite_out_files = False

local_machine_candidates_path = "C:\\workspace_py\\ctp_py\\resources\\candidates"
koopa_candidates_path = "/home/tristan/projects/hypermapper/outputs/candidates"

planner_readable_strings = ["PRM", "PRM*", "Lazy PRM",
                       "Lazy PRM Star", "SST", "RRT", "RRT*",
                       "LBTRRT", "TRRT", "Lazy RRT",
                       "RRT Connect", "EST", "SBL", "LBKPIECE1",
                       "KPIECE1", "BKPIECE1",
                       "STRIDE", "PDST", "FMT", "BFMT",
                       "RRT#", "RRTXstatic", "Informed RRT Star",
                       "BIT Star"]

sampler_readable_string_list = ["Uniform Valid State", "OB Valid State",
                       "Gaussian_Valid_State", "Max Clearance Valid State",
                       "Uniform Space Sample", "Gaussian Space Sample"]


plot_symbols = ["x", "circle", "star-diamond", "cross", "hash"]

# Number of different planners that are plotted and named in the legend. Sorted by occurence in output file.
num_planners = 4

# flag for plotting planners that are not among the top {num_planners} planners
show_other_planners = False


def get_single_plot_color(sampler):
    print(f"Sampler String: {sampler}")
    if sampler == 'sbmp_uniform_valid_state_sampler':
        return "darkred"
    if sampler == 'sbmp_obstacle_valid_state_sampler':
        return "green"
    if sampler == 'sbmp_gaussian_valid_state_sampler':
        return "blue"
    if sampler == 'sbmp_max_clearance_valid_state_sampler':
        return "goldenrod"
    if sampler == 'sbmp_uniform_space_sampler':
        return "magenta"
    if sampler == 'sbmp_gaussian_space_sampler':
        return "black"


def get_point_colors(samplers):
    return [get_single_plot_color(get_sampler_string(sampler)) for sampler in samplers]

def get_planner_trace(data_frame, planner, id):
    data_frame = data_frame.assign(hovertext=[f"Iteration: {i}<br>Planner: {planner_str}<br>Sampler: {s}<br>Motion validator: {mv}" for
                                  (s, i, mv, planner_str) in zip(
            [get_sampler_string(t_sampler) for t_sampler in
             data_frame.sampler], range(len(data_frame)),
            [get_motion_validator_string(t_mv) for t_mv in
             data_frame.motionValidator],
            [get_planner_string(t_p) for t_p in data_frame.planner])])

    #print(data_frame.pathlength)
    print(f"Planners: \r\n{data_frame.planner}")
    print(f"Planner: {planner}")
    filtered_df = data_frame[data_frame.planner == planner]
    print(filtered_df)

    return go.Scatter(x=filtered_df.pathlength, y=[t / 1000.0 for t in filtered_df.computationtime],
                      text=filtered_df.hovertext,
                      hovertemplate='<b>%{text}</b>',
                      mode='markers',
                      showlegend=True,
                      marker=dict(
                          symbol=plot_symbols[id],
                          size=16,
                          color=filtered_df.colors,
                          colorscale='Viridis',  # one of plotly colorscales
                          # showscale=True
                      ),
                      name=planner_readable_strings[planner])


def get_sampler_trace(data_frame, sampler_id):
    data_frame = data_frame.assign(hovertext=[f"Iteration: {i}<br>Planner: {planner_str}<br>Sampler: {s}<br>Motion validator: {mv}" for
                                  (s, i, mv, planner_str) in zip(
            [get_sampler_string(t_sampler) for t_sampler in
             data_frame.sampler], range(len(data_frame)),
            [get_motion_validator_string(t_mv) for t_mv in
             data_frame.motionValidator],
            [get_planner_string(t_p) for t_p in data_frame.planner])])

    #print(data_frame.pathlength)
    filtered_df = data_frame[data_frame.sampler == sampler_id]
    print(filtered_df)

    c = get_single_plot_color(get_sampler_string(sampler_id))

    print(f"Sampler: {get_sampler_string(sampler_id)}, Color: {c} ")
    return go.Scatter(x=filtered_df.pathlength, y=[t / 1000.0 for t in filtered_df.computationtime],
                      text=filtered_df.hovertext,
                      hovertemplate='<b>%{text}</b>',
                      mode='markers',
                      showlegend=True,
                      marker=dict(
                          symbol="square",
                          size=16,
                          color=c,
                          colorscale='Viridis',  # one of plotly colorscales
                          # showscale=True
                      ),
                      name=sampler_readable_string_list[sampler_id])


for root, dirs, files in walk(local_machine_candidates_path):
    onlyfiles = [root + "/" + f for f in files if isfile(join(root, f)) and f.endswith(".csv")]

    print(f"{onlyfiles}")
    df_list = [pd.read_csv(fi) for fi in onlyfiles]

    for (data_frame, file_name) in zip(df_list, onlyfiles):
        new_file_name = get_new_pareto_plot_file_name(file_name)

        if os.path.isfile(new_file_name + ".html") and not overwrite_out_files:
            print(f"Skipping file, {new_file_name}.html already exists.")
            continue

        new_data_frame = data_frame[~(data_frame.pathlength >= 50000.0) | ~(data_frame.computationtime >= 50000.0)]
        new_data_frame = new_data_frame.assign(colors=get_point_colors(new_data_frame.sampler))

        print(f"new df: {new_data_frame.colors}")

        fig = go.Figure()

        planner_histogram = {}
        for p in list(set(new_data_frame.planner)):
            planner_histogram[p] = list(new_data_frame.planner).count(p)

        sorted_planner_histogram = sorted(planner_histogram.items(), key=lambda kv: (kv[1], kv[0]), reverse=True)

        planner_list = [sorted_planner_histogram[i][0] for i in range(num_planners)]
        print(f"sorted_planner_histogram: {sorted_planner_histogram}")
        for i in range(num_planners):
            fig.add_trace(get_planner_trace(new_data_frame, sorted_planner_histogram[i][0], i))

        all_points_df = new_data_frame[new_data_frame.planner.isin(planner_list)]

        for s in range(6):
            fig.add_trace(get_sampler_trace(all_points_df, s))


        #if show_other_planners:
        #    fig.add_trace()

        #fig.add_trace(get_planner_trace(new_data_frame, 3, 0))

        fig.update_layout(
            yaxis_title="Computation Time [s]",
            xaxis_title="Solution Path Length [arbitrary unit]",
            font=dict(
                family="Arial",
                size=24,
                color="Black"
            )
        )
        fig.show()

        print(f"writing: {new_file_name}")
        out_dir = os.path.dirname(new_file_name)
        if not os.path.exists(out_dir):
            os.makedirs(out_dir)
            print(f"Creating dir: {out_dir}")

        fig.write_html(new_file_name + ".html")

print("done")
