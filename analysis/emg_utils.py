import pandas as pd
import numpy as np
import matplotlib.pyplot as plt


def compute_RMS(data, window_length):
    try:
        data_array = data.to_numpy()
    except AttributeError:
        data_array = data
    rms_emg = np.zeros(data_array.size - window_length)
    # rms_emg = []
    for i in range(data_array.size - window_length):
        window = data_array[i: i + window_length]
        # rms_emg.append(np.sqrt(np.mean(np.square(window))))
        rms_emg[i] = np.sqrt(np.mean(np.square(window)))
    return rms_emg


def draw_background(axis, type_array, rms_array, min, max):
    time = type_array['Time'].to_numpy()
    trial_type = type_array['type'].to_numpy()

    trial_time = []

    rest_fill = None
    load_fill = None
    unload_fill = None
    for i in range(time.size):
        if i == time.size - 1:
            trial_time = [time[i] - rms_array['Time'].iloc[0], rms_array['Time'].iloc[-1] - rms_array['Time'].iloc[0]]
        else:
            trial_time = [time[i] - rms_array['Time'].iloc[0], time[i+1] - rms_array['Time'].iloc[0]]

        if trial_type[i] == 'rest':
            rest_fill = axis.fill_between(trial_time, min, max, alpha=0.2, color='blue', label='Rest')
        elif trial_type[i] == 'load':
            load_fill = axis.fill_between(trial_time, min, max, alpha=0.2, color='green', label='Load')
        elif trial_type[i] == 'unload':
            unload_fill = axis.fill_between(trial_time, min, max, alpha=0.2, color='yellow', label='Unload')

    return rest_fill, load_fill, unload_fill


def plot_overview(trial_type, emg, state, mass, window, mvc, limits):
    # plt.close('all')
    fig, ax = plt.subplots(3,1)

    zero = pd.Series([0])
    max_mass = mass['mass'].max()
    mass_limit = 0.95

    # EMG
    ax[0].plot(emg['Time'].iloc[:-window] - emg['Time'].iloc[0], compute_RMS(emg['data_0'], window)/mvc, linewidth=1)
    compensation, = ax[0].step(pd.concat([zero, mass['Time'] - emg['Time'].iloc[0]]), pd.concat([zero, mass['mass']*mass_limit*limits[0][1]/max_mass]), where='post', linewidth=0.4, color='red', label='Compensation')
    rest, load, unload = draw_background(ax[0], trial_type, emg, limits[0][0], limits[0][1])

    ax[0].set_ylim(limits[0])
    ax[0].set_xlim([trial_type['Time'].iloc[0] - emg['Time'].iloc[0], emg['Time'].iloc[-window-1] - emg['Time'].iloc[0]])
    # ax[0].set_title('EMG Predictive Exo Support')
    ax[0].set_ylabel('Normalized EMG')
    # ax[0].set_yticks([0.1, 0.2, 0.3])
    # ax[0].set_xlabel('Time (s)')
    ax[0].grid(True)

    # q
    ax[1].plot(state['Time'] - state['Time'][0], state['q_state.q'], linewidth=1)
    ax[1].step(pd.concat([zero, mass['Time'] - emg['Time'].iloc[0]]), pd.concat([zero, mass['mass']*mass_limit*limits[1][1]/max_mass]), where='post', linewidth=0.4, color='red', label='Compensation')
    draw_background(ax[1], trial_type, state, limits[1][0], limits[1][1])
    ax[1].set_xlim([trial_type['Time'].iloc[0] - emg['Time'].iloc[0], emg['Time'].iloc[-window-1] - emg['Time'].iloc[0]])
    ax[1].set_ylim(limits[1])
    ax[1].set_ylabel('q (deg)')

    # tau
    ax[2].plot(state['Time'] - state['Time'][0], state['tau'], linewidth=1)
    # compensation, = ax[2].step(pd.concat([zero, mass['Time'] - emg['Time'].iloc[0]]), pd.concat([zero-limits[2][0], mass['mass']*mass_limit*(limits[2][1]-limits[2][0])/max_mass-limits[2][0]]), where='post', linewidth=0.4, color='red', label='Compensation')
    compensation, = ax[2].step(pd.concat([zero, mass['Time'] - emg['Time'].iloc[0]]), 
                               pd.concat([zero - abs(limits[2][0]), mass['mass']*mass_limit*(limits[2][1] - limits[2][0])/max_mass - abs(limits[2][0])]), 
                               where='post', linewidth=0.4, color='red', label='Compensation')

    rest, load, unload = draw_background(ax[2], trial_type, state, limits[2][0], limits[2][1])
    ax[2].set_xlim([trial_type['Time'].iloc[0] - emg['Time'].iloc[0], emg['Time'].iloc[-window-1] - emg['Time'].iloc[0]])
    ax[2].set_ylim(limits[2])
    ax[2].set_ylabel('Torque (Nm)')
    ax[2].set_xlabel('Time (s)')
    ax[2].legend([rest, load, unload, compensation], ['Rest', 'Load', 'Unload', 'Compensation'], frameon=True, ncol=4, loc="lower right")

    return fig, ax


def single_EMG(ax, trial_type, emg, window, plot_prediction, mass, min, max, mvc):
    # plt.close('all')
    # fig, ax = plt.subplots()
    ax.plot(emg['Time'].iloc[:-window] - emg['Time'].iloc[0], compute_RMS(emg['data_0'], window)/mvc, linewidth=1)
    rest, load, unload = draw_background(ax, trial_type, emg, min, max)


    ax.set_ylim([min, max])
    ax.set_xlim([trial_type['Time'].iloc[0] - emg['Time'].iloc[0], emg['Time'].iloc[-1] - emg['Time'].iloc[0]])
    ax.set_ylabel('Normalized EMG RMS')
    ax.set_xlabel('Time (s)')
    # ax.set_yticks([0, 0.05, 0.1, 0.15, 0.20, 0.25, 0.30])
    ax.grid(True, axis='y')
    if plot_prediction:
        max_mass = mass['mass'].max()
        mass_limit = 0.9
        zero = pd.Series([0])
        compensation, = ax.step(pd.concat([zero, mass['Time'] - emg['Time'].iloc[0]]), pd.concat([zero, mass['mass']*mass_limit*max/max_mass]), where='post', linewidth=0.4, color='red', label='Compensation')
        ax.legend([rest, load, unload, compensation], ['Rest', 'Load', 'Unload', 'Compensation'], frameon=True, ncol=4)
    else:
        ax.legend([rest, load, unload], ['Rest', 'Load', 'Unload'], frameon=True, ncol=3)

    # return fig, ax


def EMG_comp(trial_type, trial_emg, baseline_type, baseline_emg, window, compensation, mass, direction, min, max, mvc):
    figure, axis = None, None
    if direction == 'horizontal':
        figure, axis = plt.subplots(1, 2)
    elif direction == 'vertical':
        figure, axis = plt.subplots(2, 1)

    single_EMG(axis[0], trial_type, trial_emg, window, compensation, mass, min, max, mvc)
    axis[0].set_title('EMG Exoskeleton Support')
    single_EMG(axis[1], baseline_type, baseline_emg, window, False, mass, min, max, mvc)
    axis[1].set_title('EMG Baseline')
    return figure, axis


def plot_average(axis, emg_list, type_list, window, min, max, mvc):
    shortest = emg_list[0]['Time'][emg_list[0]['Time'] >= type_list[0]['Time'][0]]
    shortest_loads = type_list[0]
    for i in range(len(emg_list)):
        filtered_time = emg_list[i]['Time'][emg_list[i]['Time'] >= type_list[i]['Time'][0]]
        if filtered_time.size < shortest.size:
            shortest = filtered_time
            shortest_loads = type_list[i]

    filtered_emg_data = []
    for i in range(len(emg_list)):
        filtered_time = emg_list[i]['Time'][emg_list[i]['Time'] >= type_list[i]['Time'][0]]
        filtered_data = emg_list[i]['data_0'][filtered_time.index]
        filtered_emg_data.append(filtered_data.iloc[:shortest.size].to_numpy())

    avg_emg = (filtered_emg_data[0] + filtered_emg_data[1] + filtered_emg_data[2])/3
    # window = 25
    axis.plot(shortest.iloc[:-window] - shortest_loads['Time'].iloc[0], compute_RMS(avg_emg, window)/mvc, linewidth=0.75)

    axis.set_xlim([0, shortest.iloc[-window] - shortest_loads['Time'].iloc[0]])
    axis.set_ylim([min, max])


    time = shortest_loads['Time'].to_numpy()
    trial_type = shortest_loads['type'].to_numpy()

    trial_time = []

    rest_fill = None
    load_fill = None
    unload_fill = None
    for i in range(time.size):
        if i == time.size - 1:
            trial_time = [time[i] - shortest_loads['Time'].iloc[0], shortest.iloc[-window] - shortest_loads['Time'].iloc[0]]
        else:
            trial_time = [time[i] - shortest_loads['Time'].iloc[0], time[i+1] - shortest_loads['Time'].iloc[0]]

        if trial_type[i] == 'rest':
            rest_fill = axis.fill_between(trial_time, min, max, alpha=0.2, color='blue', label='Rest')
        elif trial_type[i] == 'load':
            load_fill = axis.fill_between(trial_time, min, max, alpha=0.2, color='green', label='Load')
        elif trial_type[i] == 'unload':
            unload_fill = axis.fill_between(trial_time, min, max, alpha=0.2, color='yellow', label='Unload')

    axis.legend([rest_fill, load_fill, unload_fill], ['Rest', 'Load', 'Unload'], frameon=True)


def average_EMG_comp(trial_type, trial_emg, baseline_type, baseline_emg, window, direction, min, max, mvc):
    figure, axis = None, None
    if direction == 'horizontal':
        figure, axis = plt.subplots(1, 2)
    elif direction == 'vertical':
        figure, axis = plt.subplots(2, 1)

    plot_average(axis[0], trial_emg, trial_type, window, min, max, mvc)
    axis[0].set_title('Average EMG Exoskeleton Support')
    plot_average(axis[1], baseline_emg, baseline_type, window, min, max, mvc)
    axis[1].set_title('Average EMG Baseline')
    return figure, axis


def calculate_average(type_data, emg_data, mvc):

    rest_emg = np.array([])
    load_emg = np.array([])
    unload_emg = np.array([])

    for i in range(type_data['Time'].size):
        filter_time = None
        filter_data = None
        if i == type_data['Time'].size - 1:
            filter_time = emg_data['Time'][emg_data['Time'] >= type_data['Time'].iloc[i]]
            filter_data = emg_data['data_0'][filter_time.index].to_numpy()/mvc
        else:
            filter_time = emg_data['Time'][emg_data['Time'] >= type_data['Time'].iloc[i]]
            # filter_time = emg_data['Time'][emg_data['Time'] < type_data['Time'].iloc[i+1]]
            filter_time = filter_time[emg_data['Time'] < type_data['Time'].iloc[i+1]]
            filter_data = emg_data['data_0'][filter_time.index].to_numpy()/mvc

        if type_data['type'].iloc[i] == 'rest':
            rest_emg = np.concatenate((rest_emg, filter_data))
        elif type_data['type'].iloc[i] == 'load':
            load_emg = np.concatenate((load_emg, filter_data))
        elif type_data['type'].iloc[i] == 'unload':
            unload_emg = np.concatenate((unload_emg, filter_data))

    rest_avg = np.mean(rest_emg)
    load_avg = np.mean(load_emg)
    unload_avg = np.mean(unload_emg)

    rest_var = np.var(rest_emg)
    load_var = np.var(load_emg)
    unload_var = np.var(unload_emg)

    return rest_avg, load_avg, unload_avg, rest_var, load_var, unload_var

def calculate_average_rms(type_data, emg_data, mvc, window_length):
    rest_emg = np.array([])
    load_emg = np.array([])
    unload_emg = np.array([])

    for i in range(type_data['Time'].size):
        filter_time = None
        filter_data = None
        if i == type_data['Time'].size - 1:
            filter_time = emg_data['Time'][emg_data['Time'] >= type_data['Time'].iloc[i]]
            # print(filter_time)
            filter_data = compute_RMS(emg_data['data_0'][filter_time.index], window_length)/mvc
        else:
            filter_time = emg_data['Time'][emg_data['Time'] >= type_data['Time'].iloc[i]]
            # print(filter_time)
            # filter_time = emg_data['Time'][emg_data['Time'] < type_data['Time'].iloc[i+1]]
            filter_time = filter_time[emg_data['Time'] < type_data['Time'].iloc[i+1]]
            filter_data = compute_RMS(emg_data['data_0'][filter_time.index], window_length)/mvc

        # print(filter_time.index)

        if type_data['type'].iloc[i] == 'rest':
            rest_emg = np.concatenate((rest_emg, filter_data))
        elif type_data['type'].iloc[i] == 'load':
            load_emg = np.concatenate((load_emg, filter_data))
        elif type_data['type'].iloc[i] == 'unload':
            unload_emg = np.concatenate((unload_emg, filter_data))

    rest_avg = np.mean(rest_emg)
    load_avg = np.mean(load_emg)
    unload_avg = np.mean(unload_emg)

    rest_var = np.var(rest_emg)
    load_var = np.var(load_emg)
    unload_var = np.var(unload_emg)

    return rest_avg, load_avg, unload_avg, rest_var, load_var, unload_var


def calculate_average_all(type_list, emg_list, mvc):

    rest_avg_list = []
    load_avg_list = []
    unload_avg_list = []
    rest_var_list = []
    load_var_list = []
    unload_var_list = []
    for i in range(len(type_list)):
        rest_avg, load_avg, unload_avg, rest_var, load_var, unload_var = calculate_average(type_list[i], emg_list[i], mvc)
        rest_avg_list.append(rest_avg)
        load_avg_list.append(load_avg)
        unload_avg_list.append(unload_avg)
        rest_var_list.append(rest_var)
        load_var_list.append(load_var)
        unload_var_list.append(unload_var)

    return rest_avg_list, load_avg_list, unload_avg_list, rest_var_list, load_var_list, unload_var_list

def calculate_average_rms_all(type_list, emg_list, mvc, window_length):
    rest_avg_list = []
    load_avg_list = []
    unload_avg_list = []
    rest_var_list = []
    load_var_list = []
    unload_var_list = []
    for i in range(len(type_list)):
        rest_avg, load_avg, unload_avg, rest_var, load_var, unload_var = calculate_average_rms(type_list[i], emg_list[i], mvc, window_length)
        rest_avg_list.append(rest_avg)
        load_avg_list.append(load_avg)
        unload_avg_list.append(unload_avg)
        rest_var_list.append(rest_var)
        load_var_list.append(load_var)
        unload_var_list.append(unload_var)

    return rest_avg_list, load_avg_list, unload_avg_list, rest_var_list, load_var_list, unload_var_list


def load_rest_ratio(load_list, rest_list):
    ratio_list = []
    for i in range(len(load_list)):
        ratio_list.append(load_list[i]/rest_list[i])

    return ratio_list

def load_rest_diff(load_list, rest_list):
    diff_list = []
    for i in range(len(load_list)):
        diff_list.append(load_list[i] - rest_list[i])

    return diff_list

def calculate_average_trial(type_data, emg_data, mvc):

    rest_emg = []
    load_emg = []
    unload_emg = []

    for i in range(type_data['Time'].size):
        filter_time = None
        filter_data = None
        if i == type_data['Time'].size - 1:
            filter_time = emg_data['Time'][emg_data['Time'] >= type_data['Time'].iloc[i]]
            filter_data = emg_data['data_0'][filter_time.index].to_numpy()/mvc
        else:
            filter_time = emg_data['Time'][emg_data['Time'] >= type_data['Time'].iloc[i]]
            # filter_time = emg_data['Time'][emg_data['Time'] < type_data['Time'].iloc[i+1]]
            filter_time = filter_time[emg_data['Time'] < type_data['Time'].iloc[i+1]]
            filter_data = emg_data['data_0'][filter_time.index].to_numpy()/mvc

        if type_data['type'].iloc[i] == 'rest':
            rest_emg.append(filter_data)
        elif type_data['type'].iloc[i] == 'load':
            load_emg.append(filter_data)
        elif type_data['type'].iloc[i] == 'unload':
            unload_emg.append(filter_data)

    shortest_load = load_emg[0].size
    for i in range(1, len(load_emg)):
        current_load = load_emg[i].size
        if current_load < shortest_load:
            shortest_load = current_load

    shortest_rest = rest_emg[0].size
    for i in range(1, len(rest_emg)):
        current_rest = rest_emg[i].size
        if current_rest < shortest_rest:
            shortest_rest = current_rest
    
    shortest_unload = unload_emg[0].size
    for i in range(1, len(unload_emg)):
        current_unload = unload_emg[i].size
        if current_unload < shortest_unload:
            shortest_unload = current_unload

    for i in range(len(load_emg)):
        load_emg[i] = load_emg[i][:shortest_load]

    for i in range(len(unload_emg)):
        unload_emg[i] = unload_emg[i][:shortest_unload]

    for i in range(len(rest_emg)):
        rest_emg[i] = rest_emg[i][:shortest_rest]

    rest_stack = np.stack(rest_emg)
    load_stack = np.stack(load_emg)
    unload_stack = np.stack(unload_emg)

    return rest_stack, load_stack, unload_stack

def calculate_rms_average_trial(type_data, emg_data, mvc, window_length):

    rest_emg = []
    load_emg = []
    unload_emg = []

    for i in range(type_data['Time'].size):
        filter_time = None
        filter_data = None
        if i == type_data['Time'].size - 1:
            filter_time = emg_data['Time'][emg_data['Time'] >= type_data['Time'].iloc[i]]
            filter_data = emg_data['data_0'][filter_time.index].to_numpy()/mvc
        else:
            filter_time = emg_data['Time'][emg_data['Time'] >= type_data['Time'].iloc[i]]
            filter_time = filter_time[emg_data['Time'] < type_data['Time'].iloc[i+1]]
            filter_data = emg_data['data_0'][filter_time.index].to_numpy()/mvc

        if type_data['type'].iloc[i] == 'rest':
            rest_emg.append(filter_data)
        elif type_data['type'].iloc[i] == 'load':
            load_emg.append(filter_data)
        elif type_data['type'].iloc[i] == 'unload':
            unload_emg.append(filter_data)

    shortest_load = load_emg[0].size
    for i in range(1, len(load_emg)):
        current_load = load_emg[i].size
        if current_load < shortest_load:
            shortest_load = current_load

    shortest_rest = rest_emg[0].size
    for i in range(1, len(rest_emg)):
        current_rest = rest_emg[i].size
        if current_rest < shortest_rest:
            shortest_rest = current_rest
    
    shortest_unload = unload_emg[0].size
    for i in range(1, len(unload_emg)):
        current_unload = unload_emg[i].size
        if current_unload < shortest_unload:
            shortest_unload = current_unload

    for i in range(len(load_emg)):
        load_emg[i] = load_emg[i][:shortest_load]

    for i in range(len(unload_emg)):
        unload_emg[i] = unload_emg[i][:shortest_unload]

    for i in range(len(rest_emg)):
        rest_emg[i] = rest_emg[i][:shortest_rest]

    rest_stack = np.stack(rest_emg)
    load_stack = np.stack(load_emg)
    unload_stack = np.stack(unload_emg)

    emg_stack = np.hstack((rest_stack, load_stack, unload_stack))

    emg_list = []

    for i in range(emg_stack.shape[0]):
        emg_list.append(compute_RMS(emg_stack[i], window_length))

    rms_stack = np.stack(emg_list)


    # return rest_stack, load_stack, unload_stack
    return rms_stack


def plot_average_trial(axis, trial_type, emg, mvc, window_length, ylimits):
    rest_stack, load_stack, unload_stack = calculate_average_trial(trial_type, emg, mvc)

    rest_avg = np.mean(rest_stack, axis=0)
    rest_var = np.var(rest_stack, axis=0)

    load_avg = np.mean(load_stack, axis=0)
    load_var = np.var(load_stack, axis=0)

    unload_avg = np.mean(unload_stack, axis=0)
    unload_var = np.var(unload_stack, axis=0)

    loading_time = 9 # 6 
    unloading_time = 3
    resting_time = 3 # 5

    emg_sequence = compute_RMS(np.concatenate((rest_avg, load_avg, unload_avg)), window_length)
    time = np.linspace(0, loading_time + unloading_time + resting_time, emg_sequence.size)
    axis.plot(time, emg_sequence, linewidth=0.75)

    axis.fill_between([0, resting_time], 0, 1, alpha=0.2, color='blue', label='Rest')
    axis.fill_between([resting_time, resting_time+loading_time], 0, 1, alpha=0.2, color='green', label='Load')
    axis.fill_between([resting_time+loading_time, resting_time+loading_time+unloading_time], 0, 1, alpha=0.2, color='yellow', label='Unload')



    axis.fill_between(np.linspace(0, resting_time, num=rest_avg.size), rest_avg-rest_var, rest_avg+rest_var,alpha=0.5)
    axis.fill_between(np.linspace(resting_time, resting_time+loading_time, num=load_avg.size), load_avg-load_var, load_avg+load_var,alpha=0.5)
    axis.fill_between(np.linspace(resting_time+loading_time, resting_time+loading_time+unloading_time, num=unload_avg.size), unload_avg-unload_var, unload_avg+unload_var,alpha=0.5)

    axis.set_xlim([0, loading_time + unloading_time + resting_time])
    axis.set_ylim(ylimits)

def plot_rms_average_trial(axis, trial_type, emg, mvc, window_length, ylimits):
    loading_time = 9 # 6 
    unloading_time = 3
    resting_time = 3 # 5

    # rest_stack, load_stack, unload_stack = calculate_rms_average_trial(trial_type, emg, mvc, window_length)

    rms_stack = calculate_rms_average_trial(trial_type, emg, mvc, window_length)

    rms_avg = np.mean(rms_stack, axis=0)
    rms_var = np.var(rms_stack, axis=0)

    time = np.linspace(0, loading_time + unloading_time + resting_time, rms_avg.size)
    axis.plot(time, rms_avg, linewidth=0.75)
    axis.fill_between(time, rms_avg-rms_var, rms_avg+rms_var, alpha=0.5)

    # rest_avg = np.mean(rest_stack, axis=0)
    # rest_var = np.var(rest_stack, axis=0)

    # load_avg = np.mean(load_stack, axis=0)
    # load_var = np.var(load_stack, axis=0)

    # unload_avg = np.mean(unload_stack, axis=0)
    # unload_var = np.var(unload_stack, axis=0)

    # emg_sequence = np.concatenate((rest_avg, load_avg, unload_avg))
    # time = np.linspace(0, loading_time + unloading_time + resting_time, emg_sequence.size)
    # axis.plot(time, emg_sequence, linewidth=0.75)

    axis.fill_between([0, resting_time], 0, 1, alpha=0.2, color='blue', label='Rest')
    axis.fill_between([resting_time, resting_time+loading_time], 0, 1, alpha=0.2, color='green', label='Load')
    axis.fill_between([resting_time+loading_time, resting_time+loading_time+unloading_time], 0, 1, alpha=0.2, color='yellow', label='Unload')



    # axis.fill_between(np.linspace(0, resting_time, num=rest_avg.size), rest_avg-rest_var, rest_avg+rest_var,alpha=0.5)
    # axis.fill_between(np.linspace(resting_time, resting_time+loading_time, num=load_avg.size), load_avg-load_var, load_avg+load_var,alpha=0.5)
    # axis.fill_between(np.linspace(resting_time+loading_time, resting_time+loading_time+unloading_time, num=unload_avg.size), unload_avg-unload_var, unload_avg+unload_var,alpha=0.5)

    # axis.set_xlim([0, loading_time + unloading_time + resting_time])
    axis.set_ylim(ylimits)