""" Provides some utilities to deal with jupyter notebook specific issues """

from inspect import getsource
from IPython.display import HTML, display,IFrame
from pygments.formatters import HtmlFormatter
from pygments.lexers import PythonLexer
from pygments import highlight
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.collections import LineCollection
import numpy as np
from operator import attrgetter

def source(*functions):
    """ source a script as if we have written the script in jupyter notebook and executed it """
    source_code = '\n\n'.join(getsource(fn) for fn in functions)        
    display(HTML(highlight(source_code, PythonLexer(), HtmlFormatter(full=True))))

""" animation function used in GIS routing notebook """
def update_plot(i, data, scat):
    scat.set_array(data[i])
    return scat,

def animate_simple(G, colors, speed = 100):
    # to handle big animations like bi directional search
    matplotlib.rcParams['animation.embed_limit'] = 2**128

    colors = np.array(colors)
    numframes = len(colors)
    node_Xs = [float(x) for _, x in G.nodes(data='x')]
    node_Ys = [float(y) for _, y in G.nodes(data='y')]
    fig, ax =  plt.subplots(figsize=(15, 11))
    ax.set_facecolor('w')
    lines = []
    for u, v, data in G.edges(keys=False, data=True):
            if 'geometry' in data:
                xs, ys = data['geometry'].xy
                lines.append(list(zip(xs, ys)))
            else:
                x1 = G.nodes[u]['x']
                y1 = G.nodes[u]['y']
                x2 = G.nodes[v]['x']
                y2 = G.nodes[v]['y']
                line = [(x1, y1), (x2, y2)]
                lines.append(line)

    lc = LineCollection(lines, colors='#999999', linewidths=0.9, alpha=0.3)

    ax.add_collection(lc)
    scat = ax.scatter(node_Xs, node_Ys,c=colors[0], s=30)
    ax.get_xaxis().set_visible(False)
    ax.get_yaxis().set_visible(False)
    ani = animation.FuncAnimation(fig, update_plot, frames=list(range(numframes)),interval=speed
                                  ,fargs = (colors, scat))
    return ani