#!/usr/bin/env python

from graph_helper import *

from Tkinter import *
from tkFileDialog import askopenfilename
import tkMessageBox
import rospkg
import numpy as np
import ConfigParser as CP
import math

dirname = rospkg.RosPack().get_path('mrpp_ideal')
dic = {0: 'Undirected', 1: 'Directed'}
cpg = Tk()

class Create_Planar_Graph:

	def __init__(self, frame):
		self.frame = frame
		self.config = CP.ConfigParser()
		self.config.read(dirname + '/config_file.txt')
		self.f = None

		self.x_pos = []
		self.y_pos = []

		self.edges = []

		self.name = ''
		self.node_num = 0
		self.edge_num = 0
		self.node_count = -1
		self.edge_count = -1
		self.graph_type = 'Undirected'

		self.gff = Button(self.frame, text = 'Create Graph From File', command = self.gff_callback)
		self.gff.pack()

		self.l0 = Label(self.frame, text = "**********")
		self.l0.pack()

		self.l1 = Label(self.frame, text = 'New Graph Name')
		self.l1.pack()

		self.e1 = Entry(self.frame, bd = 5, textvariable = StringVar())
		self.e1.pack()

		self.l2 = Label(self.frame, text = 'Number of nodes')
		self.l2.pack()
		
		self.e2 = Entry(self.frame, bd = 5, textvariable = IntVar())
		self.e2.pack()

		self.l3 = Label(self.frame, text = 'Number of edges')
		self.l3.pack()

		self.e3 = Entry(self.frame, bd = 5, textvariable = IntVar())
		self.e3.pack()

		self.l4 = Label(self.frame, text = 'Graph Type')
		self.l4.pack()

		self.lb = Listbox(self.frame, height = 2, selectmode = SINGLE)
		self.lb.insert(0, 'Undirected')
		self.lb.insert(1, 'Directed')
		self.lb.pack()

		self.gen = Button(self.frame, text = 'Start Graph', command = self.gen_callback)
		self.gen.pack()

		self.x_low = np.inf
		self.x_high = 0
		self.y_low = np.inf
		self.y_high = 0

		self.l5 = Label(self.frame, text = 'Node Co-ordinates (x, y)')
		self.l5.pack()

		self.e4 = Entry(self.frame, bd = 5, textvariable = DoubleVar())
		self.e4.pack()

		self.e5 = Entry(self.frame, bd = 5, textvariable = DoubleVar())
		self.e5.pack()

		self.adn = Button(self.frame, text = 'Add Node', state = DISABLED, command = self.node_callback)
		self.adn.pack()

		self.l6 = Label(self.frame, text = 'Edge between - (Node 1, Node2)')
		self.l6.pack()

		self.e6 = Entry(self.frame, bd = 5, textvariable = IntVar())
		self.e6.pack()

		self.e7 = Entry(self.frame, bd = 5, textvariable = IntVar())
		self.e7.pack()

		self.ade = Button(self.frame, text = 'Add Edge', state = DISABLED, command = self.edge_callback)
		self.ade.pack()

		self.fin = Button(self.frame, text = 'Generate Graph', state = DISABLED, command = self.fin_callback)
		self.fin.pack()

		self.frame.mainloop()

	def gff_callback(self):
		self.file_name = askopenfilename(defaultextension = '.in', initialdir = dirname + '/graph_file')
		
		temp = self.file_name.split('.')
		temp = temp[0].split('/')

		self.name = temp[-1]
		duplicate = self.config.has_section(self.name)
		if duplicate:
			self.name = ''
			self.node_num = 0
			self.edge_num = 0
			tkMessageBox.showinfo('Name Error', 'Graph already exists, please try a different one!')
		
		else:
			img_name = dirname + '/graph_png/' + temp[-1] + '.png'

			graph = extract_graph(self.file_name, False)
			bl = boundary_limits(graph)
			
			self.x_low = bl[0]
			self.y_low = bl[1]
			self.x_high = bl[2]
			self.y_high = bl[3]

			graph_type = self.lb.curselection()
			self.graph_type = dic[graph_type[0]]

			self.config.add_section(temp[-1])
			create_image(graph, img_name, self.x_low, self.y_low, self.x_high, self.y_high, 800, 1000, 100)

			self.config.set(temp[-1], 'display_params', str([self.x_low, self.y_low, self.x_high, self.y_high, 800, 1000, 100]))
			self.config.set(self.name, 'graph_type', self.graph_type)
			with open(dirname+'/config_file.txt', 'w') as config_file:
				self.config.write(config_file)

			self.frame.destroy()


	def fin_callback(self):
		img_name = dirname + '/graph_png/' + self.name + '.png'
		graph = extract_graph(self.file_name, self.graph_type == 'Undirected')

		create_image(graph, img_name, self.x_low, self.y_low, self.x_high, self.y_high, 800, 1000, 100)

		self.config.set(self.name, 'display_params', str([self.x_low, self.y_low, self.x_high, self.y_high, 800, 1000, 100]))
		self.config.set(self.name, 'graph_type', self.graph_type)
		with open(dirname+'/config_file.txt', 'w') as config_file:
			self.config.write(config_file)

		self.frame.destroy()

	def edge_callback(self):
		node1 = int(self.e6.get())
		node2 = int(self.e7.get())
		edge = [node1, node2]

		if node1 < 0 or node1 >= self.node_count or node2 < 0 or node2 >= self.node_count:
			tkMessageBox.showinfo('Value Error', 'Please insert a valid node ID!')
		
		elif edge in self.edges:
			tkMessageBox.showinfo('Value Error', 'This edge already exists!')
		
		elif self.graph_type == 'Undirected' and edge.reverse() in self.edges:
			tkMessageBox.showinfo('Value Error', 'This edge already exists!')

		else:
			self.edge_count += 1
			self.edges.append(edge)
			self.f.write('{} {}\n'.format(node1, node2))

			if self.edge_count == self.edge_num:
				self.f.close()
				self.ade.configure(state = DISABLED)
				self.fin.configure(state = NORMAL)

	def node_callback(self):
		x_pos = float(self.e4.get())
		y_pos = float(self.e5.get())

		too_close = False
		
		for i in range(len(self.x_pos)):
			dist = math.sqrt((x_pos - self.x_pos[i]) ** 2 + (y_pos - self.y_pos[i]) ** 2)
			if dist < 10.0:
				too_close = True
				break

		if too_close:
			tkMessageBox.showinfo('Value Error', 'Please enter a node away from the existing ones!')

		elif x_pos < 0 or y_pos < 0:
			tkMessageBox.showinfo('Value Error', 'Please insert a positive value')
		
		else:
			self.node_count += 1
			self.x_pos.append(x_pos)
			self.y_pos.append(y_pos)

			if x_pos < self.x_low:
				self.x_low = x_pos
			elif x_pos > self.x_high:
				self.x_high = x_pos

			if y_pos < self.y_low:
				self.y_low = y_pos
			elif y_pos > self.y_high:
				self.y_high = y_pos

			self.f.write('{} {}\n'.format(x_pos, y_pos))

			if self.node_count == self.node_num:
				self.adn.configure(state = DISABLED)
				self.ade.configure(state = NORMAL)


	def gen_callback(self):
		self.name = self.e1.get()
		self.node_num = int(self.e2.get())
		self.edge_num = int(self.e3.get())

		graph_type = self.lb.curselection()
		duplicate = self.config.has_section(self.name)
		
		if duplicate:
			self.name = ''
			self.node_num = 0
			self.edge_num = 0
			tkMessageBox.showinfo('Name Error', 'Name already exists, please try a different one!')
		
		elif self.name == '':
			self.name = ''
			self.node_num = 0
			self.edge_num = 0
			tkMessageBox.showinfo('Name Error', 'Please enter a name!')

		elif len(graph_type) == 0:
			self.name = ''
			self.node_num = 0
			self.edge_num = 0
			tkMessageBox.showinfo('Type Error', 'Please select Undirected/Directed!')

		elif self.node_num == 0 or self.edge_num == 0 or self.edge_num < self.node_num - 1:
			self.name = ''
			self.node_num = 0
			self.edge_num = 0
			tkMessageBox.showinfo('Count Error', 'Please check the number of nodes/edges!')

		elif graph_type == 0 and self.edge_num > (self.node_num * (self.node_num - 1) / 2):
			self.name = ''
			self.node_num = 0
			self.edge_num = 0
			tkMessageBox.showinfo('Count Error', 'Please check the number of nodes/edges!')

		elif graph_type == 1 and self.edge_num > (self.node_num * (self.node_num - 1)):
			self.name = ''
			self.node_num = 0
			self.edge_num = 0
			tkMessageBox.showinfo('Count Error', 'Please check the number of nodes/edges!')

		else:
			self.graph_type = dic[graph_type[0]]
			self.node_count = 0
			self.edge_count = 0
			self.gen.configure(state = DISABLED)
			self.adn.configure(state = NORMAL)
			
			self.config.add_section(self.name)
			self.file_name = dirname + '/graph_file/' + self.name + '.in'
			
			self.f = open(self.file_name, 'w')
			self.f.write('{} {}\n'.format(self.node_num, self.edge_num))

graph = Create_Planar_Graph(cpg)