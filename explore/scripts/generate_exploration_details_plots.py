#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

def main():
    df = pd.read_csv ("/home/react-ws-1/catkin_ws/src/m-explore/explore/data/data_wiserx/coverage_overlap_wiserx_exploration_test_1759448899.csv")
    # sns.lineplot(data=df, x="time_elapsed_sec", y="merged_coverage_percent")
    sns.lineplot(data=df, x="time_elapsed_sec", y="coverage_overlap_percent")
    plt.show()

if __name__ == "__main__":
    main()