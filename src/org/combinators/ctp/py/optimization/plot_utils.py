import pandas as pd
from os import listdir
from os.path import isfile, join
from os import walk
import sys
import os.path

from functools import reduce

sys.path.append("/home/tristan/projects/ctp_py/src")

overwrite_out_files = False

res_folder = "/home/tristan/projects/ctp_py/3D/"

config_files = ["Abstract", "Apartment", "Apartment_hard", "Easy", "Home", "Twistycool", "Twistycooler", "alpha-1.0",
                "alpha-1.1", "alpha-1.2", "alpha-1.5", "bugtrap", "cubicles", "escape", "pipedream", "pipedream_ring",
                "spirelli"]

mypath = "/home/tristan/projects/hypermapper/outputs"
p_out_folder = "/home/tristan/projects/hypermapper/problems_output"
#pp_out_folder = "/home/tristan/projects/hypermapper/problems_pareto_output"
pp_out_folder = "/home/tristan/projects/hypermapper/outputs/candidates"

plot_modes = ["failures", "pathlength", "computationtime"]
plot_folders = ["failures/", "pathlength/", "computationtime/"]


def get_new_mavg_file_name(fn, plot_mode=0):
    file_base_name = os.path.basename(fn)
    config_name = [name for name in config_files if name in file_base_name]
    if config_name:
        max_config_name = max(config_name)
        return os.path.join(p_out_folder, plot_folders[plot_mode], max_config_name, file_base_name + ".html")
    else:
        print(f"Problem finding new filename for fn: {fn}, config_name (len should be ex 1): {config_name}")
        return os.path.join(os.path.dirname(fn), "test1.html")


def get_new_multi_mavg_file_name(config, plot_mode=0):
        return os.path.join(p_out_folder, plot_folders[plot_mode], config, config + "accu.html")


def get_new_pareto_plot_file_name(fn):
    file_base_name = os.path.basename(fn)
    config_name = [name for name in config_files if name in file_base_name]
    if config_name:
        max_config_name = max(config_name)
        return os.path.join(pp_out_folder, max_config_name, file_base_name + ".html")
    else:
        print(f"Problem finding new filename for fn: {fn}, config_name (len should be ex 1): {config_name}")
        return os.path.join(os.path.dirname(fn), "test1.html")


def get_data_files():
    files_nested = [
        [os.path.join(root, c_file) for c_file in files if isfile(join(root, c_file)) and c_file.endswith(".csv")]
        for root, dirs, files in walk(mypath)]
    only_files = reduce(lambda x, y: x + y, files_nested)
    for fi in only_files:
        print(f"reading fi: {fi}")
        pd.read_csv(fi)
    data_frame_list = [pd.read_csv(fi) for fi in only_files]
    data_frames = zip(data_frame_list, only_files)
    return data_frames


print("done")
