import pandas as pd

from os.path import isfile, join
from os import walk
import os.path
import sys
import plotly.express as px
import plotly.graph_objects as go
import numpy as np

from org.combinators.ctp.py.optimization.plot_utils import get_new_pareto_plot_file_name

sys.path.append("/home/tristan/projects/ctp_py/src")

from org.combinators.ctp.py.optimization.ctp_optimization import get_planner_string, get_sampler_string, \
    get_motion_validator_string

overwrite_out_files = False

mypath = "/home/tristan/projects/hypermapper/outputs/candidates"


def get_single_plot_symbol(p):
    if p == "sbmp_planner_EST" or p == "sbmp_planner_PRM":
        return 'square'
    if p == "sbmp_planner_SBL" or p == "sbmp_planner_PRMStar":
        return 'circle'
    if p == "sbmp_planner_STRIDE" or p == "sbmp_planner_LazyPRMStar":
        return 'star-diamond'
    if p == "sbmp_planner_LBTRRT":
        return 'y-up'
    if p == "sbmp_planner_LazyRRT":
        return 'hash'
    print(f"Unknown symbol {p}")
    return 'star-diamond'


def get_plot_symbols(planner):
    print(f"Planner String: {planner}")

    return [get_single_plot_symbol(get_planner_string(p)) for p in planner]


def get_single_plot_color(sampler):
    print(f"Planner String: {sampler}")
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


for root, dirs, files in walk(mypath):
    onlyfiles = [root + "/" + f for f in files if isfile(join(root, f)) and f.endswith(".csv")]

    print(f"{onlyfiles}")
    df_list = [pd.read_csv(fi) for fi in onlyfiles]

    for (data_frame, file_name) in zip(df_list, onlyfiles):
        new_file_name = get_new_pareto_plot_file_name(file_name)

        if os.path.isfile(new_file_name + ".html") and not overwrite_out_files:
            print(f"Skipping file, {new_file_name}.html already exists.")
            continue

        new_data_frame = data_frame[~(data_frame.pathlength > 50000.0) | ~(data_frame.computationtime > 50000.0)]

        new_data_frame.symbols = get_plot_symbols(new_data_frame.planner)
        new_data_frame.colors = get_point_colors(new_data_frame.sampler)

        fig = go.Figure(
            data=go.Scatter(x=new_data_frame.pathlength, y=[t/1000.0 for t in new_data_frame.computationtime],
                            text=[f"Iteration: {i}<br>Planner: {planner}<br>Sampler: {s}<br>Motion validator: {mv}" for
                                  (s, i, mv, planner) in zip(
                                    [get_sampler_string(t_sampler) for t_sampler in
                                     new_data_frame.sampler], range(len(new_data_frame)),
                                    [get_motion_validator_string(t_mv) for t_mv in
                                     new_data_frame.motionValidator],
                                    [get_planner_string(t_p) for t_p in new_data_frame.planner])],
                            hovertemplate=
                            '<b>%{text}</b>',
                            mode='markers',
                            marker=dict(
                                symbol=new_data_frame.symbols,
                                size=16,
                                color=new_data_frame.colors,
                                # set color equal to a variable
                                colorscale='Viridis',  # one of plotly colorscales
                                showscale=True
                            )))
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
