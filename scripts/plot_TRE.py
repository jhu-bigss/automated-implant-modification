import os
import glob
import numpy as np
import matplotlib.pyplot as plt

# Define labels of the plot
labels = ['1st & 2nd 3D scans\n (Plastic Skull 1)',
          '1st & 2nd 3D scans\n (Plastic Skull 2)',
          'CT & single 3D scan\n (Cadaver 1)',
          ' CT & 1st 3D scan\n (Cadaver 2)',
          '1st & 2nd 3D scans\n (Cadaver2)',
          'CT & 2nd 3D scan\n (Cadaver 2)'
         ]

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
        labels[0] : 'plastic_skull_1.csv',
        labels[1] : 'plastic_skull_2.csv',
        labels[2] : '1939m_CT_scan.csv',
        labels[3] : '1932f_CT_scan1.csv',
        labels[4] : '1932f_scan1_scan2.csv',
        labels[5] : '1932f_CT_scan2.csv',
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
        print("mean error:", np.mean(errors), " mm")

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