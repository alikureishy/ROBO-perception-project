'''
Created on Dec 22, 2016

@author: safdar
'''
import numpy as np
import matplotlib.pyplot as plt

#
# Plottable base class
#
class Plottable(object):
    def __init__(self, title):
        self.__title__ = title
    def title(self):
        return self.__title__

#
# Image object to be plotted
#
class Image(Plottable):
    def __init__(self, title, image, cmap):
        Plottable.__init__(self, title)
        self.__image__ = np.copy(image)
        self.__cmap__ = cmap
    def cmap(self):
        return self.__cmap__
    def image(self):
        return self.__image__
    
#
# Graph object to be plotted
#
class Graph(Plottable):
    def __init__(self, title, xs, ys, xlabel, ylabel):
        Plottable.__init__(self, title)
        self.__xs__ = xs
        self.__ys__ = ys
        self.__xlabel__ = xlabel
        self.__ylabel__ = ylabel

class Canvas(object):
    def __init__(self, axes):
        self.__axes__ = axes
    def plot(self):
        raise "Not implemented"
    def replot(self, axes):
        raise "Not implemented"
    
class GraphSlot(Canvas):
    def __init__(self, axes):
        Canvas.__init__(self, axes)
    def plot(self, graphdata):
        if graphdata.__xs__ is not None:
            self.__axes__.plot(graphdata.__xs__, graphdata.__ys__)
        else:
            self.__axes__.plot(graphdata.__ys__)
        if graphdata.__xlabel__ is not None:
            self.__axes__.set_xlabel(graphdata.__xlabel__)
        if graphdata.__ylabel__ is not None:
            self.__axes__.set_ylabel(graphdata.__ylabel__)
    def replot(self, graphdata):
        self.__axes__.clear()
        self.plot(graphdata)

class ImageSlot(Canvas):
    def __init__(self, axes):
        Canvas.__init__(self, axes)
        self.__axesimage__ = None
    def plot(self, imagedata):
        self.__axes__.set_xticks([])
        self.__axes__.set_yticks([])
        self.__axesimage__ = self.__axes__.imshow(imagedata.image(), cmap=imagedata.cmap())
    def replot(self, imagedata):
        self.__axesimage__.set_data(imagedata.image())

#
# Plotting class
#
class Illustrator(object):
    def __init__(self, plot):
        if plot:
            self.__canvas__ = PyplotCanvas()
        else:
            self.__canvas__ = ImageCanvas()
        
    def nextframe(self, framenum):
        return Frame(self.__canvas__, framenum)
    
class ImageCanvas(object):
    def __init__(self):
        pass
    def redraw(self, sections, framenum):
        if framenum % 100 == 0:
            print ("{}".format(framenum))

class PyplotCanvas(object):
    def __init__(self):
        self.__figure__ = None
        self.__axes__ = None
        self.__figure_text__ = None
        self.__canvases__ = None
        
    def redraw(self, sections, framenum):
        # If there's only one section, split it into rows/cols:
        h,v = None, None
        if len(sections)==1:
            N = len(sections[0])
            if (N > 0):
                h=int(round(np.sqrt(N)))
                v=int(np.ceil(N/h))
                diff = (h*v) - N
                while diff < 0:
                    v += 1
                    diff = (h*v) - N
                if diff > 0:
                    sample = np.zeros((20, 20, 3), dtype=np.uint8)
                    for _ in range(diff):
                        sections[0].append(Image("--Blank--", sample, None))
                sections = np.reshape(np.array(sections), (v,h))
        else:
            _, maxsection = max(enumerate(sections), key = lambda tup: len(tup[1]))
            h = len(maxsection)
            v = len(sections)

        # Plot:
        if not v is None and not h is None:
            if self.__figure__ == None or len(self.__figure__.get_axes()) is not (v*h):
                # If we didn't draw the figure before, or if the grid needs to change:
                needs_refresh = False
                if self.__figure__ is not None:
                    plt.close(self.__figure__)
                    self.__figure__ = None
                    needs_refresh = True
                self.__figure__, _  = plt.subplots (v, h)
                self.__slots__ = []
                if self.__figure_text__ is None:
                    self.__figure_text__ = self.__figure__.suptitle("", fontsize=14, fontweight='bold')

#                 self.__axes_images__ = []
                for i in range(v):
                    self.__slots__.append([])
                    for j in range(h):
                        if j >= len(sections[i]):
                            break # We've finished one section (horizontal plots). Goto next i.
                        idx = (i*h) + j
    
                        toplot = sections[i][j]
                        axes = self.__figure__.get_axes()[idx]
                        font = min (max (int( 100 / (np.sqrt(len(toplot.title())) * v * h)), 7), 15)
                        axes.set_title(toplot.title(), fontsize=font)
                        if type(toplot)==Image:
                            slot = ImageSlot(axes)
                        elif type(toplot)==Graph:
                            slot = GraphSlot(axes)
                        else:
                            raise "Type {} unsupported for any slot opereations".format(type(toplot))
                        self.__slots__[i].append(slot)                  
                        slot.plot(toplot)
                plt.ion()
                plt.show()
            else:
                for i in range(v):
                    for j in range(h):
                        if j >= len(sections[i]):
                            break # We've finished one section (horizontal plots). Goto next i.
                        idx = (i*h) + j
                        axes = self.__figure__.get_axes()[idx]
                        
                        toplot = sections[i][j]
                        font = min (max (int( 100 / (np.sqrt(len(toplot.title())) * v * h)), 7), 15)
                        axes.set_title(toplot.title(), fontsize=font)
                        slot = self.__slots__[i][j]
                        slot.replot(toplot)
                plt.ion()
                self.__figure__.canvas.draw()
                plt.pause(0.00001)
            self.__figure_text__.set_text("Frame: {}".format(framenum))
        if framenum % 100 == 0:
            print ("{}".format(framenum))
    
# A Frame represents state that is to be reflected in the current
# pyplot frame. The actual plotting is performed by the Illustrator
class Frame(object):
    def __init__(self, canvas, framenum):
        self.__plotter__ = canvas
        self.__sections__ = []
        self.__framenum__ = framenum
    
    def add(self, plottable, index=-1):
        assert len(self.__sections__)>0, "Must invoke newsection() first before calling add()"
        self.__sections__[index].append(plottable)

    def framenumber(self):
        return self.__framenum__
    
    def newsection(self, name):
        self.__sections__.append([])
    
    def render(self):
        self.__plotter__.redraw(self.__sections__, self.__framenum__)
