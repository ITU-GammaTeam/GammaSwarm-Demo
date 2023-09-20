import tkinter as tk
from tkinter import ttk
import os
from classes import *
from itertools import zip_longest
import datetime
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg,NavigationToolbar2Tk
from matplotlib.backend_bases import key_press_handler
from PIL import ImageTk, Image  
import mplcursors


class GammaAnalyser(tk.Tk):
    def __init__(self, master=None, log_path=None):
        super().__init__(master)
        self.log_path = log_path
        self.sequances = []
        self.prepare_data()
        self.set_duration()
        self.current_seq_idx = 0
        self.selected_uav = "uav_1"
        self.inserted_uavs = set([])
        self.render_window()


    def render_window(self):
        self.title("Gamma Analyser")
        #maximized
        w, h = self.winfo_screenwidth(), self.winfo_screenheight()
        self.geometry("%dx%d+0+0" % (w, h))
        seq = self.sequances[self.current_seq_idx]

        # def hover(event):
        #     vis = annot.get_visible()
        #     if event.inaxes == self.ax:
        #         cont, ind = sc.contains(event)
        #         if cont:
        #             update_annot(ind)
        #             annot.set_visible(True)
        #             fig.canvas.draw_idle()
        #         else:
        #             if vis:
        #                 annot.set_visible(False)
        #                 fig.canvas.draw_idle()

        # def update_annot(ind):
        #     pos = sc.get_offsets()[ind["ind"][0]]
        #     annot.xy = pos
        #     text = "{}, {}".format(" ".join(list(map(str,ind["ind"]))), 
        #                         " ".join([names[n] for n in ind["ind"]]))
        #     annot.set_text(text)
        #     annot.get_bbox_patch().set_facecolor(cmap(norm(c[ind["ind"][0]])))
        #     annot.get_bbox_patch().set_alpha(0.4)

        self.fig = Figure(figsize=(6, 5), dpi=100)
        # self.fig.canvas.mpl_connect("motion_notify_event", hover)
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlim3d(-2.5, 2.5)
        self.ax.set_ylim3d(-2.5, 2.5)
        self.ax.set_zlim3d(0, 5)
        self.fill_plot()
        # annot = self.ax.annotate("", xy=(0,0), xytext=(20,20),textcoords="offset points",
        #                     bbox=dict(boxstyle="round", fc="w"),
        #                     arrowprops=dict(arrowstyle="->"))
        # annot.set_visible(False)
        self.canvas = FigureCanvasTkAgg(self.fig, master=self)
        self.canvas.draw()
        self.canvas.get_tk_widget().place(x=1240, y=0)
        self.tree_frame = tk.Frame(self, width=900, height=500)
        columns = ("ID", "params" ,"State", "Command", "Trajectory Command")
        tv = ttk.Treeview(self.tree_frame, columns=columns, show='headings', height=8)
        tv.place(x=0, y=0)
        tv.heading("ID", text="ID")
        tv.column("ID", width=60)
        tv.heading("params", text="params")
        tv.column("params", width=110)
        tv.heading("State", text="State")
        tv.column("State", width=240)
        tv.heading("Command", text="Command")
        tv.column("Command", width=240)
        tv.heading("Trajectory Command", text="Trajectory Command")
        tv.column("Trajectory Command", width=240)
        sb = tk.Scrollbar(self.tree_frame, orient=tk.VERTICAL)
        sb.place(x=880, y=0, height=500)
        #connect scroll bar to tree view
        tv.configure(yscrollcommand=sb.set)
        sb.configure(command=tv.yview)

        def on_tree_select(event):
            selected = tv.focus()
            temp = tv.item(selected, 'values')
            self.selected_uav = temp[0]
            seq_type = self.sequances[self.current_seq_idx].all_logs[0]["type"]
            print(seq_type)
            if seq_type == "state":
                self.ax.clear()
                self.fill_plot()
                self.canvas.draw()
            update_bottom_graph()
            self.selected.set(self.selected_uav)

        for uav in seq.uav_names:
            values = self.set_uav_values(uav, seq)
            tv.insert("", "end", values=values,iid=uav)
            #empty insert
            self.inserted_uavs.add(uav)
        
        #insert some empty rows to tv
        for i in range(5):
            tv.insert("", "end", values=("", "", "", "", ""))

        def tree_updater():
            #update all values
            child_id = self.selected_uav
            tv.focus_set()
            tv.focus(child_id)
            tv.selection_set(child_id)
            for uav in self.inserted_uavs:
                values = self.set_uav_values(uav, self.sequances[self.current_seq_idx])
                tv.item(uav, values=values)
            self.selected.set(self.selected_uav)

            


        tv.bind('<<TreeviewSelect>>', on_tree_select)


        style = ttk.Style()
        style.configure("Treeview",rowheight=130)
        style.theme_use("default")
        style.map("Treeview")
        self.tree_frame.place(x=340, y=0)

        def update(val):
            self.selected_uav = self.selected.get()
            self.current_seq_idx = int(self.time_picker.get())
            #filter seq types
            seq_type = self.sequances[self.current_seq_idx].all_logs[0]["type"]
            print(seq_type)
            if seq_type == "state":
                self.ax.clear()
                self.fill_plot()
                self.canvas.draw()
            tree_updater()
            update_bottom_graph()
            self.selected.set(self.selected_uav)
            # self.set_current_duration()
            self.timevar.set(self.sequances[self.current_seq_idx].seq_time)


        self.time_picker_frame = tk.Frame(self,background="red",width=1800)
        #create zero timestamp for time picker
        start = tk.Label(self.time_picker_frame, text=str(self.flight_duration))
        # start.pack(side=tk.LEFT)
        self.time_picker = tk.Scale(self.time_picker_frame, from_=0, to=len(self.sequances), orient=tk.HORIZONTAL,command=update, length=1800)
        self.time_picker.pack()
        end = tk.Label(self.time_picker_frame, text=str(self.flight_duration))
        # end.pack(side=tk.LEFT)
        self.time_picker_frame.place(x=50, y=500)

        def update_bottom_graph(val=None):
            self.bot_ax.clear()
            self.fill_bot_plot()
            self.bot_canvas.draw()

        self.select_data_frame = tk.Frame(self,width=340,height=500,border=3,relief="groove")
        options = list(self.inserted_uavs)
        self.selected = tk.StringVar()
        self.selected.set(options[0])
        velcityortime = ["time", "velocity"]
        xyz = ["x", "y", "z"]
        self.xyzval = tk.StringVar()
        self.velpcityortimeval = tk.StringVar()
        self.velpcityortimeval.set(velcityortime[0])
        self.xyzval.set(xyz[0])
        self.l1 = tk.Label(self.select_data_frame, text="Select UAV")
        self.uav_selector = tk.OptionMenu(self.select_data_frame, self.selected, *options,command=update)
        self.l2 = tk.Label(self.select_data_frame, text="X axis type")
        self.l3 = tk.Label(self.select_data_frame, text="select axis")
        self.velocityortimeselect = tk.OptionMenu(self.select_data_frame, self.velpcityortimeval, *velcityortime,command=update_bottom_graph)
        self.xyzselect = tk.OptionMenu(self.select_data_frame, self.xyzval, *xyz, command=update_bottom_graph)
        #create check boxes for data
        self.position_check_var = tk.IntVar()
        self.velocity_check_var = tk.IntVar()
        self.command_check_var = tk.IntVar()
        self.trajectory_vel_check_var = tk.IntVar()
        self.trajectory_pos_check_var = tk.IntVar()
        self.trajectory_yaw_check_var = tk.IntVar()
        self.trajectory_omega_check_var = tk.IntVar()
        self.trajectory_accel_check_var = tk.IntVar()
        self.position_check = tk.Checkbutton(self.select_data_frame, text="Position",command=update_bottom_graph,variable=self.position_check_var)
        self.velocity_check = tk.Checkbutton(self.select_data_frame, text="Velocity",command=update_bottom_graph,variable=self.velocity_check_var)
        self.command_check = tk.Checkbutton(self.select_data_frame, text="Command",command=update_bottom_graph,variable=self.command_check_var)
        self.trajectory_vel_check = tk.Checkbutton(self.select_data_frame, text="Trajectory Command",command=update_bottom_graph,variable=self.trajectory_vel_check_var)
        self.trajectory_pos_check = tk.Checkbutton(self.select_data_frame, text="Trajectory Position",command=update_bottom_graph,variable=self.trajectory_pos_check_var)
        self.trajectory_yaw_check = tk.Checkbutton(self.select_data_frame, text="Trajectory Yaw",command=update_bottom_graph,variable=self.trajectory_yaw_check_var)
        self.trajectory_omega_check = tk.Checkbutton(self.select_data_frame, text="Trajectory Omega",command=update_bottom_graph,variable=self.trajectory_omega_check_var)
        self.trajectory_accel_check = tk.Checkbutton(self.select_data_frame, text="Trajectory Acceleration",command=update_bottom_graph,variable=self.trajectory_accel_check_var)


        self.l1.grid(row=0, column=0)
        self.uav_selector.grid(row=0, column=1)
        self.l2.grid(row=1, column=0)
        self.velocityortimeselect.grid(row=1, column=1)
        self.l3.grid(row=2, column=0)
        self.xyzselect.grid(row=2, column=1)
        self.position_check.grid(row=3, column=0)
        self.velocity_check.grid(row=3, column=1)
        self.command_check.grid(row=4, column=0,columnspan=2)
        self.trajectory_vel_check.grid(row=5, column=0)
        self.trajectory_pos_check.grid(row=5, column=1)
        self.trajectory_yaw_check.grid(row=6, column=0)
        self.trajectory_omega_check.grid(row=6, column=1)

        self.select_data_frame.place(x=10, y=100)
        #place logo
        img = Image.open("logo.png")
        scaling_factor = 0.1
        img = img.resize((int(img.size[0] * scaling_factor), int(img.size[1] * scaling_factor)), Image.ANTIALIAS)
        self.logo = ImageTk.PhotoImage(img)
        self.logo_label = tk.Label(self, image=self.logo)
        self.logo_label.place(x=57, y=318)

        #write "gamma analyser"
        self.gamma_analyser_label = tk.Label(self, text="Gamma Analyser", font=("Helvetica", 23, "bold"))
        self.gamma_analyser_label.place(x=51, y=38)

        self.bot_fig = Figure(figsize=(20, 4), dpi=100)
        self.bot_ax = self.bot_fig.add_subplot(111)
        self.fill_bot_plot()
        self.bot_canvas = FigureCanvasTkAgg(self.bot_fig, master=self)
        self.bot_canvas.draw()
        self.bot_canvas.get_tk_widget().place(x=-40, y=550)
        self.bot_toolbar = NavigationToolbar2Tk(self.bot_canvas, self)
        self.bot_toolbar.update()
        self.bot_canvas._tkcanvas.place(x=-40, y=550)

        self.timevar = tk.StringVar()
        self.timevar.set(seq.seq_time)
        self.time_label = tk.Entry(self, textvariable=self.timevar, width=25)
        self.current_time_label = tk.Label(self, text="Current Time:")
        self.current_time_label.place(x=15, y=460)
        self.time_label.place(x=110, y=460)

        



    def fill_plot(self):
        self.ax.set_xlim3d(-2.5, 2.5)
        self.ax.set_ylim3d(-2.5, 2.5)
        self.ax.set_zlim3d(0, 5)
        seq = self.sequances[self.current_seq_idx]
        for uav in seq.uav_names:
            uav__state_logs = filter(lambda log: log["id"] == uav and log["type"]=="state",seq.all_logs)
            for log in uav__state_logs:
                state = log["data"]
                if state.id == self.selected_uav:
                    a = self.ax.scatter3D(state.pose.position.x, state.pose.position.y, state.pose.position.z, c='r', marker='o',label=state.id)

                else:
                    b = self.ax.scatter3D(state.pose.position.x, state.pose.position.y, state.pose.position.z, c='b', marker='o',label=state.id)

        self.ax.legend()

    def fill_bot_plot(self):
        pos_check = bool(self.position_check_var.get())
        vel_check = bool(self.velocity_check_var.get())
        com_check = bool(self.command_check_var.get())
        t_vel_check = bool(self.trajectory_vel_check_var.get())
        t_pos_check = bool(self.trajectory_pos_check_var.get())
        t_yaw_check = bool(self.trajectory_yaw_check_var.get())
        t_omega_check = bool(self.trajectory_omega_check_var.get())
        t_accel_check = bool(self.trajectory_accel_check_var.get())

        time = np.linspace(0, int(str(self.flight_duration)), len(self.sequances))
        uav = self.selected_uav
        #get all position data for selected uav
        position_x = np.zeros(len(self.sequances))
        position_y = np.zeros(len(self.sequances))
        position_z = np.zeros(len(self.sequances))
        velocity_x = np.zeros(len(self.sequances))
        velocity_y = np.zeros(len(self.sequances))
        velocity_z = np.zeros(len(self.sequances))
        command_x = np.zeros(len(self.sequances))
        command_y = np.zeros(len(self.sequances))
        command_z = np.zeros(len(self.sequances))
        t_vel_x = np.zeros(len(self.sequances))
        t_vel_y = np.zeros(len(self.sequances))
        t_vel_z = np.zeros(len(self.sequances))
        t_pos_x = np.zeros(len(self.sequances))
        t_pos_y = np.zeros(len(self.sequances))
        t_pos_z = np.zeros(len(self.sequances))
        t_yaw = np.zeros(len(self.sequances))
        t_omega_x = np.zeros(len(self.sequances))
        t_omega_y = np.zeros(len(self.sequances))
        t_omega_z = np.zeros(len(self.sequances))
        t_accel_x = np.zeros(len(self.sequances))
        t_accel_y = np.zeros(len(self.sequances))
        t_accel_z = np.zeros(len(self.sequances))



        for idx,seq in enumerate(self.sequances):
            uav_state_logs = filter(lambda log: log["id"] == uav and log["type"]=="state",seq.all_logs)
            uav_command_logs = filter(lambda log: log["id"] == uav and log["type"]=="command",seq.all_logs)
            uav_trajectory_command_logs = filter(lambda log: log["id"] == uav and log["type"]=="t_command",seq.all_logs)
            

            for log in uav_state_logs:
                state = log["data"]
                position_x[idx] = state.pose.position.x
                position_y[idx] = state.pose.position.y
                position_z[idx] = state.pose.position.z
                velocity_x[idx] = state.twist.linear.x
                velocity_y[idx] = state.twist.linear.y
                velocity_z[idx] = state.twist.linear.z

            for log in uav_command_logs:
                command = log["data"]
                command_x[idx] = command.twist.linear.x
                command_y[idx] = command.twist.linear.y
                command_z[idx] = command.twist.linear.z

            for log in uav_trajectory_command_logs:
                command = log["data"]
                t_vel_x[idx] = command.twist.linear.x
                t_vel_y[idx] = command.twist.linear.y
                t_vel_z[idx] = command.twist.linear.z
                t_pos_x[idx] = command.pose.position.x
                t_pos_y[idx] = command.pose.position.y
                t_pos_z[idx] = command.pose.position.z
                t_yaw[idx] = command.yaw
                t_omega_x[idx] = command.twist.angular.x
                t_omega_y[idx] = command.twist.angular.y
                t_omega_z[idx] = command.twist.angular.z
                t_accel_x[idx] = command.accelaeration.x
                t_accel_y[idx] = command.accelaeration.y
                t_accel_z[idx] = command.accelaeration.z


        position_x[position_x == 0] = np.nan
        position_y[position_y == 0] = np.nan
        position_z[position_z == 0] = np.nan
        velocity_x[velocity_x == 0] = np.nan
        velocity_y[velocity_y == 0] = np.nan
        velocity_z[velocity_z == 0] = np.nan
        command_x[command_x == 0] = np.nan
        command_y[command_y == 0] = np.nan
        command_z[command_z == 0] = np.nan
        t_vel_x[t_vel_x == 0] = np.nan
        t_vel_y[t_vel_y == 0] = np.nan
        t_vel_z[t_vel_z == 0] = np.nan
        t_pos_x[t_pos_x == 0] = np.nan
        t_pos_y[t_pos_y == 0] = np.nan
        t_pos_z[t_pos_z == 0] = np.nan
        t_yaw[t_yaw == 0] = np.nan
        t_omega_x[t_omega_x == 0] = np.nan
        t_omega_y[t_omega_y == 0] = np.nan
        t_omega_z[t_omega_z == 0] = np.nan
        t_accel_x[t_accel_x == 0] = np.nan
        t_accel_y[t_accel_y == 0] = np.nan
        t_accel_z[t_accel_z == 0] = np.nan



        if self.velpcityortimeval.get() == "time":
            x_axis = time
            self.bot_ax.set_xlabel('time')
            x_text = "time"
            vertical_x = x_axis[self.current_seq_idx]

        elif self.velpcityortimeval.get() == "velocity":
            x_axis = velocity_x
            self.bot_ax.set_xlabel('velocity')
            x_text = "velocity"
            vertical_x = x_axis[self.current_seq_idx]


            
        if pos_check and self.xyzval.get() == "x":
            pos_x = self.bot_ax.plot(x_axis, position_x ,label="position")
            mplcursors.cursor(pos_x)
        if pos_check and self.xyzval.get() == "y":
            pos_y = self.bot_ax.plot(x_axis, position_y ,label="position")
            mplcursors.cursor(pos_y)
        if pos_check and self.xyzval.get() == "z":
            pos_z = self.bot_ax.plot(x_axis, position_z ,label="position")
            mplcursors.cursor(pos_z)
        if vel_check and self.xyzval.get() == "x":
            vel_x = self.bot_ax.plot(x_axis, velocity_x ,label="velocity")
            mplcursors.cursor(vel_x)
        if vel_check and self.xyzval.get() == "y":
            vel_y = self.bot_ax.plot(x_axis, velocity_y ,label="velocity")
            mplcursors.cursor(vel_y)
        if vel_check and self.xyzval.get() == "z":
            vel_z = self.bot_ax.plot(x_axis, velocity_z ,label="velocity")
            mplcursors.cursor(vel_z)
        if com_check and self.xyzval.get() == "x":
            com_x = self.bot_ax.plot(x_axis, command_x ,label="command")
            mplcursors.cursor(com_x)
        if com_check and self.xyzval.get() == "y":
            com_y = self.bot_ax.plot(x_axis, command_y ,label="command")
            mplcursors.cursor(com_y)
        if com_check and self.xyzval.get() == "z":
            com_z = self.bot_ax.plot(x_axis, command_z ,label="command")
            mplcursors.cursor(com_z)
        if t_vel_check and self.xyzval.get() == "x":
            t_vel_x = self.bot_ax.plot(x_axis, t_vel_x ,label="t velocity")
            mplcursors.cursor(t_vel_x)
        if t_vel_check and self.xyzval.get() == "y":
            t_vel_y = self.bot_ax.plot(x_axis, t_vel_y ,label="t velocity")
            mplcursors.cursor(t_vel_y)
        if t_vel_check and self.xyzval.get() == "z":
            t_vel_z = self.bot_ax.plot(x_axis, t_vel_z ,label="t velocity")
            mplcursors.cursor(t_vel_z)
        if t_pos_check and self.xyzval.get() == "x":
            t_pos_x = self.bot_ax.plot(x_axis, t_pos_x ,label="t position")
            mplcursors.cursor(t_pos_x)
        if t_pos_check and self.xyzval.get() == "y":
            t_pos_y = self.bot_ax.plot(x_axis, t_pos_y ,label="t position")
            mplcursors.cursor(t_pos_y)
        if t_pos_check and self.xyzval.get() == "z":
            t_pos_z = self.bot_ax.plot(x_axis, t_pos_z ,label="t position")
            mplcursors.cursor(t_pos_z)
        if t_yaw_check:
            t_yaw = self.bot_ax.plot(x_axis, t_yaw ,label="t yaw")
            mplcursors.cursor(t_yaw)
        if t_omega_check and self.xyzval.get() == "x":
            t_omega_x = self.bot_ax.plot(x_axis, t_omega_x ,label="t omega")
            mplcursors.cursor(t_omega_x)
        if t_omega_check and self.xyzval.get() == "y":
            t_omega_y = self.bot_ax.plot(x_axis, t_omega_y ,label="t omega")
            mplcursors.cursor(t_omega_y)
        if t_omega_check and self.xyzval.get() == "z":
            t_omega_z = self.bot_ax.plot(x_axis, t_omega_z ,label="t omega")
            mplcursors.cursor(t_omega_z)
        if t_accel_check and self.xyzval.get() == "x":
            t_accel_x = self.bot_ax.plot(x_axis, t_accel_x ,label="t acceleration")
            mplcursors.cursor(t_accel_x)
        if t_accel_check and self.xyzval.get() == "y":
            t_accel_y = self.bot_ax.plot(x_axis, t_accel_y ,label="t acceleration")
            mplcursors.cursor(t_accel_y)
        if t_accel_check and self.xyzval.get() == "z":
            t_accel_z = self.bot_ax.plot(x_axis, t_accel_z ,label="t acceleration")
            mplcursors.cursor(t_accel_z)

        def format_coord(x, y):
            # output numbers (not in scientific notation) with one decimal place
            return f"{x_text}={x:.5f}, value={y:.5f}"
        
        self.bot_ax.format_coord = format_coord
        
        self.bot_ax.axvline(x = vertical_x, color = 'grey', label = 'current time')
        self.bot_ax.legend()


    
                
    def set_uav_values(self, uav, seq):
        params = "active\nposition\norientation\nlinear\nangular\nacceleration\nyaw"
        state_column = ""
        command_column = ""
        t_command_column = ""
        uav_logs = filter(lambda log: log["id"] == uav,seq.all_logs)
        for log in uav_logs:
            if log["type"] == "state":
                state = log["data"]
                state_column = "{}\n{}\n{}\n{}\n{}\n{}\n{}".format(state.active, 
                                                                state.pose.position.toStr(), 
                                                                state.pose.orientation.toStr(), 
                                                                state.twist.linear.toStr(), 
                                                                state.twist.angular.toStr(), 
                                                                state.accelaeration.toStr(),
                                                                "-")

            if log["type"] == "command":
                command = log["data"]
                command_column = "{}\n{}\n{}\n{}\n{}\n{}\n{}".format(command.active, 
                                                                "-", 
                                                                "-", 
                                                                command.twist.linear.toStr(), 
                                                                command.twist.angular.toStr(), 
                                                                "-",
                                                                "-")
            if log["type"] == "t_command":
                t_command = log["data"]
                t_command_column = "{}\n{}\n{}\n{}\n{}\n{}\n{}".format(t_command.active, 
                                                                t_command.pose.position.toStr(), 
                                                                t_command.pose.orientation.toStr(), 
                                                                t_command.twist.linear.toStr(), 
                                                                t_command.twist.angular.toStr(), 
                                                                t_command.accelaeration.toStr(),
                                                                t_command.yaw)
        value = (uav, params , state_column, command_column, t_command_column)
        return value

    def set_duration(self):
        last_duration = self.sequances[-1].seq_time
        first_duration = self.sequances[0].seq_time
        self.flight_duration = last_duration - first_duration
        self.flight_duration = self.flight_duration.seconds 
        self.current_duration = 0
    
    def set_current_duration(self):
        current = self.sequances[self.current_seq_idx].seq_time.microsecond - self.flight_duration
        self.current_duration = current


    def prepare_data(self):
        uav_names = set([])
        uav_log_files = [os.path.join(os.path.join(self.log_path, "uav"), file) for file in os.listdir(os.path.join(self.log_path, "uav"))]
        ugv_log_files = [os.path.join(self.log_path, file) for file in os.listdir(os.path.join(self.log_path, "ugv"))]
        uav_datas = {}
        reference_data_key = "uav_1"
        estimated_seq_num = 0
        for file in uav_log_files:
            uav_names.add(file.split("/")[-1].split(".")[0])
            with open(file, "r") as f:
                if len(f.readlines()) > estimated_seq_num:
                    estimated_seq_num = len(f.readlines())
                    reference_data_key = file.split("/")[-1].split(".")[0]

        for uav_log_file in uav_log_files:
            with open(uav_log_file, "r") as f:
                lines = f.readlines()
                uav_data = self.parse_data(lines)
                uav_datas[uav_log_file.split("/")[-1].split(".")[0]] = uav_data
                if len(lines)>estimated_seq_num:
                    estimated_seq_num = len(lines)


        all_data = zip_longest(*uav_datas.values(), fillvalue=None)
        all_data = list(all_data)

        for idx,ref_data in enumerate(uav_datas[reference_data_key]):
            log = self.str2log(ref_data)
            if len(log)>0:
                logs_of_seq = []
                seq_time = log["time"]
                for uav_data in all_data[idx]:
                    if uav_data is not None:
                        log = self.str2log(uav_data)
                        if log["time"] == seq_time:
                            logs_of_seq.append(log)
                uavs_of_logs_of_seq = set([log["id"] for log in logs_of_seq])
                if len(uavs_of_logs_of_seq) == len(uav_names): 
                    self.sequances.append(Sequance(idx, logs_of_seq,uav_names))


    def parse_data(self, lines):
        data = []
        for line in lines:
            line = line.replace("\n", "")
            data.append(line)
        return data
    
    def str2log(self, log_str):
        log = {}

        log_str = log_str.replace("'", "").replace("[", "").replace("]", "").replace(",", "")
        pure_data = log_str.split(" ")

        pure_data[6] = pure_data[6]=="True"
        pure_data[7:] = [float(x) for x in pure_data[7:]]

        if "SIM_STATE" in log_str:
            time_str = pure_data[0:2]
            time_str = time_str[0] + " " + time_str[1][0:8] + "," + time_str[1][8:]
            log["time"] = datetime.datetime.strptime("".join(time_str), "%Y-%m-%d %H:%M:%S,%f")
            log["type"] = "state"
            log["id"] = pure_data[5]
            state = State(
                id = pure_data[5],
                active = bool(pure_data[6]),
                pose = Pose(
                    position = Point(x = pure_data[7], y = pure_data[8], z = pure_data[9]),
                    orientation = Quaternion(x = pure_data[10], y = pure_data[11], z = pure_data[12], w = pure_data[13])),
                twist = Twist(
                    linear = Vector3(x = pure_data[14], y = pure_data[15], z = pure_data[16]),
                    angular = Vector3(x = pure_data[17], y = pure_data[18], z = pure_data[19])),
                accelaeration = Vector3(x = pure_data[20], y = pure_data[21], z = pure_data[22]),
            )
            log["data"] = state

        elif "T_COMMAND" in log_str:
            time_str = pure_data[0:2]
            time_str = time_str[0] + " " + time_str[1][0:8] + "," + time_str[1][8:]
            log["time"] = datetime.datetime.strptime("".join(time_str), "%Y-%m-%d %H:%M:%S,%f")
            log["type"] = "t_command"
            log["id"] = pure_data[5]
            log["data"] = TrajectoryCommand(
                id = pure_data[5],
                active = pure_data[6],
                pose = Pose(
                    position = Point(pure_data[7], pure_data[8], pure_data[9]),
                    orientation = Quaternion(pure_data[10], pure_data[11], pure_data[12], pure_data[13])),
                twist = Twist(
                    linear = Vector3(pure_data[14], pure_data[15], pure_data[16]),
                    angular = Vector3(pure_data[17], pure_data[18], pure_data[19])),
                accelaeration = Vector3(pure_data[20], pure_data[21], pure_data[22]),
                yaw = pure_data[23],
            )

        elif "COMMAND" in log_str:
            time_str = pure_data[0:2]
            time_str = time_str[0] + " " + time_str[1][0:8] + "," + time_str[1][8:]
            log["time"] = datetime.datetime.strptime("".join(time_str), "%Y-%m-%d %H:%M:%S,%f") 
            log["type"] = "command"
            log["id"] = pure_data[5]
            log["data"] = Command(
                id = pure_data[5],
                active = pure_data[6],
                twist = Twist(
                    linear = Vector3(pure_data[7], pure_data[8], pure_data[9]),
                    angular = Vector3(pure_data[10], pure_data[11], pure_data[12])),
            )
        return log
        


class SelectingWindow(tk.Frame):
    def __init__(self, master=None, log_path=None):
        super().__init__(master)
        self.log_path = log_path
        self.selected_log_file = None
        self.render_window()


    def render_window(self):
        def on_select(event):
            selected_indices = listbox.curselection()
            if len(selected_indices) > 0:
                self.selected_log_file = os.path.join(str(self.log_path),log_files[selected_indices[0]])
                self.select_button.config(state=tk.NORMAL)

        def on_close():
            self.master.destroy()

        self.master.title("Selecting Log File")
        log_files = os.listdir(self.log_path)
        log_files.sort()
        if len(log_files) == 0:
            list_variable = tk.StringVar(value=["No Log File"])
            return

        frame = tk.Frame(self.master, width=200, height=220)
        label = tk.Label(frame, text="Select Log File")
        label.place(x=0, y=0)
        list_variable = tk.StringVar(value=log_files)
        listbox = tk.Listbox(frame, listvariable=list_variable, height=10)
        listbox.place(x=0, y=0)
        scrollbar = tk.Scrollbar(frame)
        scrollbar.place(x=165, y=20, height=135,width=20)
        listbox.config(yscrollcommand=scrollbar.set)
        scrollbar.config(command=listbox.yview)
        listbox.bind("<<ListboxSelect>>", on_select)
        self.select_button = tk.Button(frame, text="Select", command=on_close, state=tk.DISABLED)
        self.select_button.place(x=50, y=185)
        frame.pack()
        self.pack()



if __name__ == "__main__":
    log_path_files_path = os.path.expanduser("~/.ros/gamma_logs")
    selecting_window = SelectingWindow(log_path=log_path_files_path)
    selecting_window.mainloop()
    if selecting_window.selected_log_file is not None:
        log_path = selecting_window.selected_log_file
        ga = GammaAnalyser(log_path=log_path)
        ga.mainloop()
