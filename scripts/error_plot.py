import numpy as np
import matplotlib.pyplot as plt

plt.rcParams.update({'font.size': 24})

# Define labels, positions, bar heights and error bar heights
exp_name = ['Conventional', '3D-Scanning']
labels = ['Specimen1', 'Specimen2']

def contour_based_errors_ICRA():
    chad_means = [1.18, 4.4, 3.45, 2.51, 3.1, 3.13]
    p4_means = [1.24, 1.91, 2.47, 1.29, 0.78, 1.34]
    pv_means = [0.84, 1.26, 1.2, 0.94, 1.01, 0.84]
    mean_list = [chad_means, p4_means, pv_means]

    chad_stds = [1.91, 4.65, 3.71, 2.51, 3.3, 3.33]
    p4_stds = [1.54, 2.26, 2.86, 1.61, 0.93, 1.69]
    pv_stds = [0.98, 1.34, 1.47, 1.08, 1.1, 0.96]
    std_list = [chad_stds, p4_stds, pv_stds]

    chad_max = [3.07, 8.27, 7.55, 5.89, 5.42, 5.47]
    p4_max = [3.68, 4.05, 5.46, 4.12, 2.18, 3.78]
    pv_max = [2.00, 2.17, 3.26, 2.94, 2.33, 1.94]

    exp_name = ['Manual', 'Optical Tracking', '3D Scanning']
    labels = ['1', '2', '3', '4', '5', '6']

    x_pos = np.arange(len(labels))
    CTEs = chad_means
    error = chad_stds

    # Build the plot
    fig, ax = plt.subplots()
    for i in range(3):
        rects = ax.bar(x_pos + 0.2*i, mean_list[i],
            # yerr=std_list[i],
            width=0.2,
            align='center',
            alpha=0.5,
            ecolor='black',
            capsize=10,
            label=exp_name[i])

        
        # ha = {'center': 'center', 'right': 'left', 'left': 'right'}
        # offset = {'center': 0, 'right': 1, 'left': -1}
        # xpos = 'center'
        # # if i == 0:
        # #     xpos = 'left'
        # # elif i == 2:
        # #     xpos = 'right'

        # for rect in rects:
        #     height = rect.get_height()
        #     ax.annotate('{}'.format(height),
        #                 xy=(rect.get_x() + rect.get_width() / 2, height),
        #                 xytext=(offset[xpos]*3, 3),  # use 3 points offset
        #                 textcoords="offset points",  # in both directions
        #                 ha=ha[xpos], va='bottom', size=6.5)

    ax.set_ylabel('Gap distance (mm)')
    ax.set_xticks(x_pos)
    ax.set_xticklabels(labels)
    # ax.set_title('Contour-based evaluation for final outcome')
    ax.yaxis.grid(True)
    # ax.legend(bbox_to_anchor=(0.5, 0.5), loc='upper right')
    # ax.legend(loc="upper center")
    # plt.legend(loc="upper center"")

    # Save the figure and show
    plt.tight_layout()
    # plt.savefig('bar_plot_with_error_bars.png')
    plt.show()

    # def autolabel(rects, xpos='center'):
    #     """
    #     Attach a text label above each bar .

    #     *xpos* indicates which side to place the text w.r.t. the center of
    #     the bar. It can be one of the following {'center', 'right', 'left'}.
    #     """

    #     ha = {'center': 'center', 'right': 'left', 'left': 'right'}
    #     offset = {'center': 0, 'right': 1, 'left': -1}

    #     for rect in rects:
    #         height = rect.get_height()
    #         ax.annotate('{}'.format(height),
    #                     xy=(rect.get_x() + rect.get_width() / 2, height),
    #                     xytext=(offset[xpos]*3, 3),  # use 3 points offset
    #                     textcoords="offset points",  # in both directions
    #                     ha=ha[xpos], va='bottom')
                        
def volume_based_errors_ICRA():
    chad_means = [0.16, 0.2, 0.62, 0.32, 0.19, 1.33]
    p4_means = [0.16, 0.16, 0.18, 0.16, 0.18, 0.18]
    pv_means = [0.14, 0.58, 0.12, 0.16, 0.16, 0.23]
    mean_list = [chad_means, p4_means, pv_means]

    chad_stds = [0.3, 0.38, 1.43, 0.71, 0.34, 2.76]
    p4_stds = [0.31, 0.33, 0.39, 0.36, 0.42, 0.4]
    pv_stds = [0.28, 1.28, 0.25, 0.32, 0.35, 0.47]
    std_list = [chad_stds, p4_stds, pv_stds]

    chad_max = [1.43, 1.47, 7.08, 3.4, 1.85, 10.06]
    p4_max = [1.46, 1.45, 2.53, 2.01, 2.81, 1.48]
    pv_max = [1.42, 5.24, 1.49, 1.53, 2.1, 2.04]

    exp_name = ['Manual', 'Optical Tracking', '3D Scanning']
    labels = ['1', '2', '3', '4', '5', '6']

    x_pos = np.arange(len(labels))
    CTEs = chad_means
    error = chad_stds

    # Build the plot
    fig, ax = plt.subplots()
    for i in range(3):
        rects = ax.bar(x_pos + 0.2*i, mean_list[i],
            # yerr=std_list[i],
            width=0.2,
            align='center',
            alpha=0.5,
            ecolor='black',
            capsize=10,
            label=exp_name[i])

        
        ha = {'center': 'center', 'right': 'left', 'left': 'right'}
        offset = {'center': 0, 'right': 1, 'left': -1}
        xpos = 'center'
        # if i == 0:
        #     xpos = 'left'
        # elif i == 2:
        #     xpos = 'right'

        for rect in rects:
            height = rect.get_height()
            ax.annotate('{}'.format(height),
                        xy=(rect.get_x() + rect.get_width() / 2, height),
                        xytext=(offset[xpos]*3, 3),  # use 3 points offset
                        textcoords="offset points",  # in both directions
                        ha=ha[xpos], va='bottom', size=6.5)

    ax.set_ylabel('Mean errors (mm)')
    ax.set_xticks(x_pos)
    ax.set_xticklabels(labels)
    ax.set_title('Volume-based evaluation for final outcome')
    ax.yaxis.grid(True)
    ax.legend()

    # Save the figure and show
    plt.tight_layout()
    # plt.savefig('bar_plot_with_error_bars.png')
    plt.show()


def defect_wall_errors_two_plastic_skulls_RAL():

    chad_max = [7.08, 7.97]
    pv_max = [3.33, 2.94]
    max_list = [chad_max, pv_max]

    chad_means = [3.30, 3.58]
    pv_means = [1.24, 1.19]
    mean_list = [chad_means, pv_means]

    chad_stds = [[3.30, 3.58], [3.57, 4.05]]
    pv_stds = [[1.24, 1.19], [1.36, 1.34]]
    std_list = [chad_stds, pv_stds]

    x_pos = np.arange(len(labels))

    # Build the plot
    fig, ax = plt.subplots()
    for i in range(2):
        rects = ax.bar(x_pos + 0.2*i, mean_list[i],
            yerr=std_list[i],
            width=0.2,
            align='center',
            alpha=0.5,
            ecolor='black',
            capsize=10,
            label=exp_name[i])

        ax.scatter(x_pos + 0.2*i, max_list[i], color="0", alpha=.35)

        
        # ha = {'center': 'center', 'right': 'left', 'left': 'right'}
        # offset = {'center': 0, 'right': 1, 'left': -1}
        # xpos = 'center'
        # # if i == 0:
        # #     xpos = 'left'
        # # elif i == 2:
        # #     xpos = 'right'

        # for rect in rects:
        #     height = rect.get_height()
        #     ax.annotate('{}'.format(height),
        #                 xy=(rect.get_x() + rect.get_width() / 2, height),
        #                 xytext=(offset[xpos]*3, 3),  # use 3 points offset
        #                 textcoords="offset points",  # in both directions
        #                 ha=ha[xpos], va='bottom', size=6.5)

    ax.set_ylabel('Gap distance (mm)')
    ax.set_xticks(x_pos)
    ax.set_xticklabels(labels)
    # ax.set_title('Contour-based evaluation for final outcome')
    ax.yaxis.grid(True)
    # ax.legend()
    ax.legend(loc='upper center')

    # Save the figure and show
    plt.tight_layout()
    # plt.savefig('bar_plot_with_error_bars.png')
    plt.show()

    # def autolabel(rects, xpos='center'):
    #     """
    #     Attach a text label above each bar .

    #     *xpos* indicates which side to place the text w.r.t. the center of
    #     the bar. It can be one of the following {'center', 'right', 'left'}.
    #     """

    #     ha = {'center': 'center', 'right': 'left', 'left': 'right'}
    #     offset = {'center': 0, 'right': 1, 'left': -1}

    #     for rect in rects:
    #         height = rect.get_height()
    #         ax.annotate('{}'.format(height),
    #                     xy=(rect.get_x() + rect.get_width() / 2, height),
    #                     xytext=(offset[xpos]*3, 3),  # use 3 points offset
    #                     textcoords="offset points",  # in both directions
    #                     ha=ha[xpos], va='bottom')


def defect_edge_errors_plastic_and_cadaver_skulls_RAL():

    manual_max = [7.156, 7.578, 7.625, 12.603]
    automated_max = [2.887, 2.979, 6.422, 5.091]
    max_list = [manual_max, automated_max]

    manual_means = [4.936, 3.409, 4.495, 4.350]
    automated_means = [1.455, 1.420, 2.938, 2.732]
    mean_list = [manual_means, automated_means]

    manual_rms = [5.036, 3.887, 4.652, 5.234]
    automated_rms = [1.570, 1.535, 3.139, 2.846]
    rms_list = [manual_rms, automated_rms]

    exp_name = ['Manual', 'Proposed']
    labels = ['Plastic\nSkull 1', 'Plastic\nSkull 2', 'Cadaver\n1', 'Cadaver\n2']

    x_pos = np.arange(len(labels))

    # Build the plot
    fig, ax = plt.subplots()
    for i in range(2):
        rects = ax.bar(x_pos + 0.3*i, mean_list[i],
            # yerr=rms_list[i],
            width=0.3,
            align='center',
            alpha=0.5,
            ecolor='black',
            capsize=10,
            label=exp_name[i])

        # # Plot RMS
        # ax.plot([x_pos + 0.3*i - 0.1/2, x_pos + 0.3*i + 0.1/2], [rms_list[i], rms_list[i]], color='gray')
        # ax.plot([x_pos + 0.3*i, x_pos + 0.3*i], [[0,0,0,0], rms_list[i]], color='gray')
        ax.scatter(x_pos + 0.3*i, max_list[i], color='b', alpha=.35)

    ax.set_ylabel('Gap Distance (mm)')
    ax.set_xticks(x_pos + 0.15)
    ax.set_xticklabels(labels)
    # ax.set_title('Contour-based evaluation for final outcome')
    ax.yaxis.grid(True)
    # ax.legend()
    ax.legend(loc='upper center')

    # Save the figure and show
    plt.tight_layout()
    # plt.savefig('bar_plot_with_error_bars.png')
    plt.show()

def top_surface_errors_plastic_and_cadaver_skulls_RAL():

    manual_max = [2.848, 1.495, 3.543, 4.257]
    automated_max = [0.826, 2.476, 2.444, 0.262]
    max_list = [manual_max, automated_max]

    manual_means = [1.840, -0.707, 0.283, -1.552]
    automated_means = [-0.617, 0.793, -2.802, -1.511]
    mean_list = [manual_means, automated_means]

    manual_rms = [1.905, 1.059, 0.947, 2.218]
    automated_rms = [0.893, 0.961, 2.953, 1.751]
    rms_list = [manual_rms, automated_rms]

    exp_name = ['Manual', 'Proposed']
    labels = ['Plastic\nSkull 1', 'Plastic\nSkull 2', 'Cadaver\n1', 'Cadaver\n2']

    x_pos = np.arange(len(labels))

    # Build the plot
    fig, ax = plt.subplots()
    for i in range(2):
        rects = ax.bar(x_pos + 0.3*i, mean_list[i],
            # yerr=rms_list[i],
            width=0.3,
            align='center',
            alpha=0.5,
            ecolor='black',
            capsize=10,
            label=exp_name[i])

        ax.plot([x_pos + 0.3*i - 0.1/2, x_pos + 0.3*i + 0.1/2], [rms_list[i], rms_list[i]], color='gray')
        ax.plot([x_pos + 0.3*i, x_pos + 0.3*i], [[0,0,0,0], rms_list[i]], color='gray')
        # ax.scatter(x_pos + 0.3*i, max_list[i], color='b', alpha=.35)

    ax.set_ylabel('Field Distance (mm)')
    ax.set_xticks(x_pos + 0.15)
    ax.set_xticklabels(labels)
    # ax.set_title('Contour-based evaluation for final outcome')
    ax.yaxis.grid(True)
    # ax.legend()
    ax.legend(loc='upper center')

    # Save the figure and show
    plt.tight_layout()
    # plt.savefig('bar_plot_with_error_bars.png')
    plt.show()

if __name__ == "__main__":
    # Call the plotting function
    defect_edge_errors_plastic_and_cadaver_skulls_RAL()

    # chad_means = [2.25, 4.02, 2.47, 1.92, 1.78, 2.42]
    # p4_means = [1.45, 1.59, 2.47, 1.78, 1.45, 1.71]
    # pv_means = [0.92, 0.71, 1.40, 1.14, 0.80, 1.13]

    # accum_chad = []
    # accum_p4 = []
    # for i in range(6):
    #     diff_chad = chad_means[i] - pv_means[i]
    #     diff_p4 = p4_means[i] - pv_means[i]
    #     accum_chad.append(diff_chad/chad_means[i])
    #     accum_p4.append(diff_p4/p4_means[i])


    # chad = sum(accum_chad)/len(accum_chad)
    # p4 = sum(accum_p4)/len(accum_p4)
    # print(chad)
    # print(p4)

    # chad = sum(chad_means)/len(chad_means)
    # p4 = sum(p4_means)/len(p4_means)
    # pv = sum(pv_means)/len(pv_means)
    # print((chad-pv)/chad)
    # print((p4-pv)/p4)
