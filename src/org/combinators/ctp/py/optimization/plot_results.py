import pandas as pd

from os.path import isfile, join
from os import walk
import os.path
import sys

from org.combinators.ctp.py.optimization.plot_utils import get_new_pareto_plot_file_name

sys.path.append("/home/tristan/projects/ctp_py/src")

from org.combinators.ctp.py.optimization.ctp_optimization import get_planner_string, get_sampler_string, \
    get_motion_validator_string

overwrite_out_files = False

mypath = "/home/tristan/projects/hypermapper/outputs"
for root, dirs, files in walk(mypath):
    onlyfiles = [root + "/" + f for f in files if isfile(join(root, f)) and f.endswith(".csv")]

    print(f"{onlyfiles}")
    df_list = [pd.read_csv(fi) for fi in onlyfiles]

    import plotly.express as px
    import plotly.graph_objects as go
    import numpy as np

    for (data_frame, file_name) in zip(df_list, onlyfiles):
        new_file_name = get_new_pareto_plot_file_name(file_name)

        if os.path.isfile(new_file_name + ".html") and not overwrite_out_files:
            print(f"Skipping file, {new_file_name}.html already exists.")
            continue

        new_data_frame = data_frame[~(data_frame.pathlength > 50000.0) | ~(data_frame.computationtime > 50000.0)]

        fig = go.Figure(data=go.Scatter(x=new_data_frame.pathlength, y=new_data_frame.computationtime, text=[
            f"Iteration: {i}<br>Planner: {planner}<br>Sampler: {s}<br>Motion validator: {mv}" for
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
                                            symbol='square',
                                            size=16,
                                            color=new_data_frame.index,  # set color equal to a variable
                                            colorscale='Viridis',  # one of plotly colorscales
                                            showscale=True
                                        )))
        fig.show()

        print(f"writing: {new_file_name}")
        out_dir = os.path.dirname(new_file_name)
        if not os.path.exists(out_dir):
            os.makedirs(out_dir)
            print(f"Creating dir: {out_dir}")

        fig.write_html(new_file_name + ".html")

print("done")