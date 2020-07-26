#!/usr/bin/python


import os
import sys
import glob
import re

import argparse
from collections import namedtuple

import numpy as np
import pandas as pd

from matplotlib import pyplot as plt


Config = namedtuple('Config', ['input'], verbose=False)


def get_arguments():
    """ 
    Get command-line arguments

    """
    # init parser:
    parser = argparse.ArgumentParser("Visualize lambda change during LM optimization.")

    # add required and optional groups:
    required = parser.add_argument_group('Required')
    optional = parser.add_argument_group('Optional')

    # add required:
    required.add_argument(
        "-i", dest="input", help="Input directory of lambda change log",
        required=True, type=str
    )

    # parse arguments:
    return parser.parse_args()


def main(config):
    """
    Enhance the original map with semantic info
    
    """
    # load lambda change logs:
    df_lambda_change_logs = []
    for filename_lambda_change_log in glob.glob(
        os.path.join(config.input, "curve_fitting_LM_log__*.csv")
    ):
        # parse strategy name:
        matches = re.match(
            "curve_fitting_LM_log__(\w+).csv", 
            os.path.basename(filename_lambda_change_log)
        )
        strategy_name = matches.group(1)
        # load lambda change log:
        df_lambda_change_log = pd.read_csv(filename_lambda_change_log)
        df_lambda_change_log['strategy'] = strategy_name

        df_lambda_change_logs.append(df_lambda_change_log)

    df_lambda_change_logs = pd.concat(df_lambda_change_logs)

    # visualize:
    fig, (ax1, ax2) = plt.subplots(nrows = 1, ncols = 2)
    ax1.set_xlabel('Iter')
    ax1.set_title('Lambda')
    ax2.set_xlabel('Iter')
    ax2.set_title('Chi Squared')
    for strategy, data in df_lambda_change_logs.groupby(['strategy']): 
        ax1.plot(data['iter'], data['lambda'], label = strategy)
        ax2.plot(data['iter'], data['chi_squared'], label = strategy)
    plt.legend(loc='best')    
    plt.show()

    sys.exit(os.EX_OK) 

if __name__ == '__main__':
    # parse arguments:
    arguments = get_arguments()

    config = Config(
        input = arguments.input
    )

    main(config)
