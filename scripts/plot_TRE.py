import os
import glob
import numpy as np
import matplotlib.pyplot as plt

# Define labels of the plot
labels = ['plastic skull 1', 'plastic skull 2', 'cadaver1', 'cadaver2 CT-scan1', 'cadaver2 CT-scan2', 'cadaver2 scan1-scan2']

def loadTREs(csv_files):
    """ load csv file for every TRE and convert to numpy array
    """
    TREs = {}
    for csv_file in csv_files: 
        TRE = np.genfromtxt(csv_file, delimiter=',')
        TRE = TRE.reshape((-1,1))
        basename = os.path.basename(csv_file)
        TREs[basename] = TRE
    return TREs

def linkLabelToCSVFile():
    link = {
        'plastic skull 1' : 'plastic_skull_1.csv',
        'plastic skull 2' : 'plastic_skull_2.csv',
        'cadaver1' : '1939m_CT_scan.csv',
        'cadaver2 CT-scan1' : '1932f_CT_scan1.csv',
        'cadaver2 CT-scan2' : '1932f_CT_scan2.csv',
        'cadaver2 scan1-scan2' : '1932f_scan1_scan2.csv',
    }

    return link

def plot_TREs(TREs, dict_label_to_CSVFile):
    x_pos = np.arange(len(labels)) # x position of the labels
    mean_bar_length = 0.4

    # Build the plot
    fig, ax = plt.subplots()
    for i, label in enumerate(labels):
        errors = TREs[dict_label_to_CSVFile[label]]
        xs = [1*i] * len(errors) # x position of scatter points for current label, has to have the same number of TREs for current label 
        # plot scatter points for current label
        ax.scatter(xs, errors, color="0", alpha=.35)
        # plot mean bar for current label if more than one TRE 
        if len(errors) > 1:
            # input (x1, x2), (y1, y2) for plotting a line, where the coords of the two point are (x1, y1), (x2, y2)
            ax.plot([i - mean_bar_length/2, i + mean_bar_length/2], [np.mean(errors), np.mean(errors)])

    ax.set_ylabel('TRE (mm)')
    ax.set_xticks(x_pos)
    ax.set_xticklabels(labels)
    # ax.set_title('')
    ax.yaxis.grid(True)
    # ax.legend()
    ax.legend(loc='upper center')

    # Save the figure and show
    plt.tight_layout()
    # plt.savefig('test.png')
    plt.show()

def main():

    csv_folder = "tre_csv_files"
    csv_files = glob.glob(os.path.join(csv_folder, "*.csv"))
    csv_files.sort()
    TREs = loadTREs(csv_files)
    dict_label_to_CSVFile = linkLabelToCSVFile()
    plot_TREs(TREs, dict_label_to_CSVFile)

if __name__ == "__main__":
    main()