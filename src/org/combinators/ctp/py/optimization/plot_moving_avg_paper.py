import pandas as pd
from os import listdir
from os.path import isfile, join
from os import walk
import numpy as np
import sys
import os.path
import plotly.graph_objects as go

from org.combinators.ctp.py.optimization.plot_utils import get_data_files, config_files, p_out_folder, \
    get_new_mavg_file_name

sys.path.append("/home/tristan/projects/ctp_py/src")

overwrite_out_files = False


def simple_moving_avg(x, n):
    skipped = x.failures.to_numpy()[:100]
    cumsum = np.cumsum(np.insert(skipped, 0, 0))
    foo2 = (cumsum[n:] - cumsum[:-n]) / float(n)
    print(f"len(foo2): {len(foo2)}")
    return foo2

def fix_failure(f):
    if f == 5000000:
        return 10.0
    else:
        return f

lll = ["Abstract", "Apartment", "Home", "cubicles", "pipedream"]
colors = ["darkred", "green", "blue", "goldenrod", "magenta", "black"]
n = 10

def plot_m_avg(p_fails, index):
    print(f"fails length: {len(p_fails[0])}")
    print(f"fails: {p_fails[0]}")
    print(f"fails file: {p_fails[1]}")
    # fails2 = [f if (f != 5000000.0) else 10.0 for f in p_fails]
    # fails2 = [[f for f in f_list[0].failures] for f_list in p_fails]
    fails2 = [simple_moving_avg(f_list[0], n) for f_list in p_fails]
    test = np.dstack(fails2)
    print(f"Number of files {len(fails2)}, Problem: {lll[index]}")
    test2 = test[0]
    fails = [(10.0 - np.mean(it)) / 10.0 * 100 for it in test2]
    x = list(range(n, len(np.array(fails)) + n))
    tr = go.Scatter(x=x, y=fails, line_color=colors[index], name=lll[index])
    return tr

print("done")


if __name__ == "__main__":
    print("Starting Main")
    import argparse
    c_files = enumerate(config_files)
    [print(f"{i}, {c}") for i, c in c_files]


    ll = [[(data_frame, file_name) for (data_frame, file_name) in get_data_files() if
           file_name.__contains__(selected_c_file + ".csv")] for selected_c_file in lll]

    print(f"len ll: {len(ll)}")
    traces = [plot_m_avg(df, i) for df, i in zip(ll, range(len(ll)))]
    fig = go.Figure()
    [fig.add_trace(t) for t in traces]
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
    fig.write_html("/home/tristan/projects/hypermapper/problems_output/paper_mvavg.html")

    print("Main done")
