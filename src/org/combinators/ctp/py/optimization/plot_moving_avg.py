import pandas as pd
from os import listdir
from os.path import isfile, join
from os import walk
import numpy as np
import sys
import os.path
import plotly.graph_objects as go

from org.combinators.ctp.py.optimization.plot_utils import get_data_files, config_files, p_out_folder, \
    get_new_mavg_file_name, pp_out_folder, get_new_multi_mavg_file_name

sys.path.append("/home/tristan/projects/ctp_py/src")

overwrite_out_files = False

plot_modes = ["failures", "pathlength", "computationtime"]
plot_mode = 1
min_num = 25
mavg_window = 1
averaged_multiple_runs = False

def simple_moving_avg(x, n):
    cumsum = np.cumsum(np.insert(x, 0, 0))
    return (cumsum[n:] - cumsum[:-n]) / float(n)

def fix_failure(f):
    if f == 5000000:
        return 10.0
    else:
        return f


def plot_multi_instances(df_fn_list, selected_c_file, plot_mode):
    print("""asdasd""")

    foo_list = [
        [simple_moving_avg(np.array([f if (f != 5000000.0) else 10.0 for f in data_frame.failures]), mavg_window),
         file_name]
        for
        data_frame, file_name in df_fn_list]

    if plot_mode == 1:
        foo_list = [
            [simple_moving_avg(np.array([f for f in data_frame.pathlength if f != 5000000.0]), mavg_window), file_name]
            for data_frame, file_name in df_fn_list]
    if plot_mode == 2:
        foo_list = [
            [simple_moving_avg(np.array([f for f in data_frame.computationtime if f != 5000000.0]), mavg_window),
             file_name]
            for data_frame, file_name in df_fn_list]

    if plot_mode == 1 or plot_mode == 2:
        foo_list = [[data_fix(data), get_new_mavg_file_name(fn, plot_mode)] for data, fn in foo_list]
        foo_list = [[d, f] for d, f in foo_list if len(d) == min_num]

    # foo_list(array(array))
    print(f"foo_list: {foo_list}")

    if averaged_multiple_runs:
        new_list = np.dstack(np.array(foo_list, dtype=object))
        avg_list = [np.average(mavals) for mavals, file_name in new_list]
        file_out = get_new_multi_mavg_file_name(selected_c_file, plot_mode)
        foo_list = [[avg_list[0], file_out]]
        # array m 1 element

    [plot_m_avg(data, new_file_name) for data, new_file_name in foo_list if len(data) > 0]


def data_fix(p_fails):
    print(f"plotavg: {plot_mode}, filtering")

    fails2 = [f if (f != 5000000.0) else 10.0 for f in p_fails]
    foo_list = [10.0 - f for f in fails2]

    if plot_mode == 1 or plot_mode == 2:
        print(f"plotMode: {plot_mode}, filtering")
        before = len(foo_list)
        foo_list = np.array([f for f in p_fails if (f != 5000000.0)])
        foo_list = foo_list[:min_num]
        print(f"filtered {before - len(foo_list)} elements. before: {before}, after: {len(foo_list)}")
        if (len(foo_list) < min_num):
            print("list does not contain enough elements, disregarding")
            return []
    return foo_list


def plot_m_avg(foo_list, new_file_name):
    if not (new_file_name.__contains__(p_out_folder) or new_file_name.__contains__(pp_out_folder)):
        print(f"Invalid output folder for file {new_file_name}")
        return

    x = list(range(len(np.array(foo_list))))

    fig = go.Figure()
    fig.add_trace(go.Scatter(
        x=x, y=foo_list,
        line_color='rgb(0,100,80)',
        name=plot_modes[plot_mode],
    ))

    fig.show()
    out_dir = os.path.dirname(new_file_name)
    if not os.path.exists(out_dir):
        os.makedirs(out_dir)
        print(f"Creating dir: {out_dir}")

    fig.write_html(new_file_name)


print("done")


if __name__ == "__main__":
    print("Starting Main")
    import argparse
    c_files = enumerate(config_files)
    [print(f"{i}, {c}") for i, c in c_files]

    u_input = input()
    if u_input == "all":
        [plot_m_avg(data_frame.computationtime, get_new_mavg_file_name(file_name)) for (data_frame, file_name) in get_data_files() if not (os.path.isfile(
            get_new_mavg_file_name(file_name)))]
    else:
        user_input = int(u_input)
        selected_c_file = config_files[user_input]

        print(f"selected config file: {selected_c_file}")
        ll = [(data_frame, file_name) for (data_frame, file_name) in get_data_files() if
              file_name.__contains__(selected_c_file + ".csv") and (not os.path.isfile(
                  get_new_mavg_file_name(file_name, plot_mode)) or averaged_multiple_runs)]
        plot_multi_instances(ll, selected_c_file, plot_mode)
    print("Main done")
