import pandas as pd
from os import listdir
from os.path import isfile, join
from os import walk
import numpy as np
import sys
import os.path
import plotly.graph_objects as go

from org.combinators.ctp.py.optimization.plot_utils import get_data_files, config_files, p_out_folder, \
    get_new_mavg_file_name, get_data_files_transfer_plots

sys.path.append("/home/tristan/projects/ctp_py/src")

overwrite_out_files = False


def simple_moving_avg(x, n):
    """moving average for dafaframe

    reads failures from df and computes moving average. x: data frame. n: Int.
    """
    skipped = x.to_numpy()[:150]
    cumsum = np.cumsum(np.insert(skipped, 0, 0))
    foo2 = (cumsum[n:] - cumsum[:-n]) / float(n)
    print(f"len(foo2): {len(foo2)}")
    return foo2


def fix_failure(f):
    if f == 5000000:
        return 10.0
    else:
        return f


problem_strings = ["Abstract", "Apartment", "Home", "cubicles", "pipedream"]
colors = {
    "Abstract": "darkred",
    "Apartment": "green",
    "Home": "blue",
    "cubicles": "goldenrod",
    "pipedream": "magenta"
}


n = 10


def plot_m_avg(p_fails, line_c, problem_name):
    fails2 = simple_moving_avg(p_fails, n)
    success_rate = [(10.0 - np.mean(it)) / 10.0 * 100 for it in fails2]
    x = list(range(n, len(np.array(fails2)) + n))
    print(f"x: {x}")
    print(f"line_c: {line_c}")
    print(f"problem_name: {problem_name}")

    if problem_name == "Abstract":
        del x[91:]
        del success_rate[91:]
    else:
        del x[0:90]
        del success_rate[0:90]

    tr = go.Scatter(x=x, y=success_rate, line_color=line_c, name=problem_name,line=dict(color=line_c, width=2))
    return tr


print("done")

if __name__ == "__main__":
    print("Starting Main")
    import argparse

    c_files = enumerate(config_files)
    [print(f"{i}, {c}") for i, c in c_files]

    ll = list(get_data_files_transfer_plots())

    avg_df = {}
    traces = {}
    for problem in problem_strings:
        current_files = list(filter(lambda x: problem in x[1], ll))
        if current_files:
            list_df = list(map(lambda x: x[0].failures, current_files))
            new_df = pd.concat(list_df, axis=1)
            avg_df[problem] = new_df.mean(axis=1)
            traces["Abstract"] = plot_m_avg(avg_df[problem], "darkred", "Abstract")
            traces[problem] = plot_m_avg(avg_df[problem], colors[problem], problem)

    fig = go.Figure()

    [fig.add_trace(t) for t in traces.values()]
    fig.update_layout(
        yaxis_title="Success rate (%)",
        xaxis_title="Iteration",
        font=dict(
            family="Arial",
            size=24,
            color="Black"
        )
    )
    fig.show()
    local_path_win = ""

    fig.write_html("C:\\workspace_py\\ctp_py\\resources\\out_files\\paper_mvavg-add-v2.html")

    print("Main done")
