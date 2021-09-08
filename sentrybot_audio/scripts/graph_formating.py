import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns


def plot_line_graph(graph, data):
    graph.axhline(y=0.5, color='r', linestyle='--')

    ang_data = data['ang']
    exc_data = data['exc']
    neu_data = data['neu']
    sad_data = data['sad']
    hap_data = data['sad']
    fea_data = data['sad']
    sur_data = data['sad']

    graph.plot(ang_data, label="Anger", color='r')
    graph.plot(exc_data, label="Excited", color='y')
    graph.plot(neu_data, label="Neutral", color='g')
    graph.plot(sad_data, label="Sad", color='b')
    graph.plot(hap_data, label="Happy", color='c')
    graph.plot(fea_data, label="Fear", color='k')
    graph.plot(sur_data, label="Surprised", color='m')
    graph.legend(loc="upper left")

    return graph


def plot_bar_graph(graph, data):
    graph.axhline(y=0.5, color='r', linestyle='--')
    graph.bar(x=0, height=data[0], label="Anger", color='r')
    graph.bar(x=1, height=data[1], label="Excited", color='y')
    graph.bar(x=2, height=data[2], label="Neutral", color='g')
    graph.bar(x=3, height=data[3], label="Sad", color='b')
    graph.bar(x=3, height=data[4], label="Happy", color='c')
    graph.bar(x=3, height=data[5], label="Fear", color='k')
    graph.bar(x=3, height=data[6], label="Surprised", color='m')
    graph.legend(loc="upper left")

    return graph


def plot_text(text_array, polarity):
    char_size = 12
    empty_string_size = char_size
    total_input_string_size = 0

    word_index = 0.012
    text_total= ""

    plt.cla()

    plt.rcParams.update({'font.size': char_size})

    if polarity > 0:
        colour = "green"
    elif polarity < 0:
        colour = "red"
    else:
        colour = "gray"

    for text in text_array:
        text = text + " "
        text_size = len(text) * 0.012
        total_input_string_size += text_size

        plt.text(word_index, 0.5, text, bbox=dict(facecolor=colour,
                                                  alpha=0.5))
        word_index += text_size

    plt.text(word_index + 1, 0.5, str(polarity), bbox=dict(facecolor=colour, alpha=0.5))

    plt.xlim([0, word_index])

    return plt


def noise_filter(prev_val: pd.DataFrame, cur_val: pd.DataFrame, a = 0.8):

    filtered_value = pd.DataFrame()
    for col in prev_val.columns.values:
        filtered_value[col] = [a * float(prev_val[col]) +
                               (1-a) * float(cur_val[col])]

    return filtered_value

