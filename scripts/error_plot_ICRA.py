import numpy as np
import matplotlib
import matplotlib.pyplot as plt


font = {'family' : 'normal',
        # 'weight' : 'bold',
        'size'   : 22}

matplotlib.rc('font', **font)

exp_name = ['Manual', 'Optical Tracking', '3D Scanning']
labels = ['1', '2', '3', '4', '5', '6']
x_pos = np.arange(len(labels))

chad_means = [2.242134, 4.018399, 2.474487, 1.915592, 2.336843, 2.423811]
p4_means =   [1.415911, 1.585113, 2.472585, 1.777397, 1.449479, 1.709679]
pv_means =   [0.897038, 0.706432, 1.404943, 1.137069, 0.803154, 1.132152]
mean_list = [chad_means, p4_means, pv_means]

chad_stds = [1.061936, 1.358429, 1.511489, 1.176343, 1.375836, 1.344616]
p4_stds =   [0.990091, 0.877834, 1.608729, 0.886480, 0.879476, 1.102035]
pv_stds =   [0.552488, 0.406847, 0.634068, 0.672649, 0.552752, 0.802943]
std_list = [chad_stds, p4_stds, pv_stds]

# Build the plot
fig, ax = plt.subplots()
for i in range(3):
    rects = ax.bar(x_pos + 0.2*i, mean_list[i],
        yerr=std_list[i],
        width=0.2,
        align='center',
        alpha=0.5,
        ecolor='black',
        capsize=10,
        label=exp_name[i])
ax.set_ylabel('Gap distance (mm)')
ax.set_xticks(x_pos)
ax.set_xticklabels(labels)
# ax.set_title('Gap Distance (mm)')
ax.yaxis.grid(True)

# Save the figure and show
plt.tight_layout()
# plt.savefig('bar_plot_with_error_bars.png')
plt.show()