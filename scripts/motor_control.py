#!/usr/bin/env python2

import rospy

from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float32
import numpy as np
import matplotlib.pyplot as plt
import tf.transformations as tr
import time
import os

class motor_control:


    def __init__(self, save_dir):
        self.save_dir = save_dir
        rospy.init_node('motor_control', anonymous=True)
        tool_sub = rospy.Subscriber('NDI/TraxatalTool/measured_cp', TransformStamped, self.tool_sub_callback)
        tip_sub  = rospy.Subscriber('NDI/SnakeCalJig/measured_cp', TransformStamped, self.tip_sub_callback)
        bendleft_sub = rospy.Subscriber('motor_bendleft/measured_jp', Float32, self.bendleft_sub_callback)
        bendright_sub = rospy.Subscriber('motor_bendright/measured_jp', Float32, self.bendright_sub_callback)

        self.bendright_pub = rospy.Publisher('motor_bendright/move_jp', Float32)
        self.bendleft_pub = rospy.Publisher('motor_bendleft/move_jp', Float32)

        self.bendleft_position = 0.0
        self.bendright_position = 0.0
        
        self.bendleft_limits = [0.0, 2.0]
        self.bendright_limits = [0.0, 2.0]


        self.ms = 0.001
        self.sec = 1.0
        self.tool_transform = TransformStamped()
        self.tip_transform = TransformStamped()

        self.tip_zero_transform = np.eye(4)
        
        self.tool_transforms_measured = []
        self.tip_transforms_measured = []
        self.bendleft_position_measured = []
        self.bendright_position_measured = []

    def run(self):
       do_run = True
       while do_run:
            user_in = raw_input("Press Enter to collect datapoint, 'X' to exit, 'F' to fit for the left motor: , 'Z' to set zero transform, 'L' or 'R' to set motor move command: ")
            if user_in is "X": break

            if user_in is "F":
                self.fit_polynomials_to_TS(self.bendleft_position_measured, self.tip_transforms_measured, self.tip_zero_transform, 3)
                continue

            if user_in is "R":
                move_cmd = raw_input("Move right motor to x.x mm: ")
                self.move_abs_bendright(float(move_cmd))
                continue

            if user_in is "L":
                move_cmd = raw_input("Move left motor to x.x mm: ")
                self.move_abs_bendleft(float(move_cmd))
                continue

            avg_tool, avg_tip, avg_bendleft, avg_bendright = self.collect_and_average(1000*self.ms, 20*self.ms) 

            if user_in is 'Z':
                self.tip_zero_transform = avg_tip

            self.tool_transforms_measured.append(avg_tool)
            self.tip_transforms_measured.append(avg_tip)
            self.bendleft_position_measured.append(avg_bendleft)
            self.bendright_position_measured.append(avg_bendright)


    def collect_and_average(self, collect_time, period):
        rospy.sleep(0.5) #ensure new values
        start_time = rospy.Time.now()
        collect_duration =  rospy.Duration(collect_time)
        period_duration = rospy.Duration(period)
        tool_transforms_collected = []
        tip_transforms_collected = []
        bendleft_pos_collected = []
        bendright_pos_collected = []

        while (rospy.Time.now() - start_time < collect_duration):
            loop_start_time = rospy.Time.now() # TODO: implement using rosTimer
            while (rospy.Time.now() - loop_start_time < period_duration):
                tool_transforms_collected.append(self.tool_transform)
                tip_transforms_collected.append(self.tip_transform)
                bendleft_pos_collected.append(self.bendleft_position)
                bendright_pos_collected.append(self.bendright_position)

        averaged_tool = self.average_list_of_TransformStamped(tool_transforms_collected)
        averaged_tip = self.average_list_of_TransformStamped(tip_transforms_collected)
        averaged_bendleft = sum(bendleft_pos_collected)/len(bendleft_pos_collected)
        averaged_bendright = sum(bendright_pos_collected)/len(bendright_pos_collected)

        return (averaged_tool, averaged_tip, averaged_bendleft, averaged_bendright)


    def fit_polynomials_to_TS(self, lengths, TS, zero_TS, degree):
        x = np.zeros(len(lengths))
        y = np.zeros(len(lengths))
        z = np.zeros(len(lengths))
        thy = np.zeros(len(lengths))

        print("zero_TS: ", zero_TS)
        zero_T_inv = np.linalg.inv(zero_TS)
        for i in range(len(lengths)):
            Q = np.dot(zero_T_inv,  TS[i])
            x[i]=Q[0,3]
            y[i]=Q[1,3]
            z[i]=Q[2,3]
            print(Q[0:3,0:3])
            th,dir,_ = tr.rotation_from_matrix(Q)
            thy[i] = th * np.sign(dir[1])
            



        # TransformStamped.,transform.
        l = np.array(lengths)


        x_coeff = np.polyfit(l, x, degree)
        y_coeff = np.polyfit(l, y, degree)
        z_coeff = np.polyfit(l, z, degree)
        thy_coeff = np.polyfit(l, thy, degree)

        poly_x = np.poly1d(x_coeff)
        poly_y = np.poly1d(y_coeff)
        poly_z = np.poly1d(z_coeff)
        poly_thy = np.poly1d(thy_coeff)

        new_l = np.linspace(min(l), max(l))
        new_x = poly_x(new_l)
        new_y = poly_y(new_l)
        new_z = poly_z(new_l)
        new_thy = poly_thy(new_l)

        fig, (ax_x, ax_y, ax_z, ax_thy) = plt.subplots(4)
        ax_x.plot(l, x, "o", new_l, new_x)
        ax_x.set(xlabel="cable length (mm)", ylabel="Tip X value (m)")
        ax_y.plot(l, y, "o", new_l, new_y)
        ax_y.set(xlabel="cable length (mm)", ylabel="Tip Y value (m)")
        ax_z.plot(l, z, "o", new_l, new_z)
        ax_z.set(xlabel="cable length (mm)", ylabel="Tip Z value (m)")
        ax_thy.plot(l, thy, "o", new_l, new_thy)
        ax_thy.set(xlabel="cable length (mm)", ylabel="Tip Rotation Angle (th_y) (rad)")        

        timestamp = time.strftime("%Y%m%d%H%M%S_")
        save_filename = self.save_dir+timestamp+"xyzthy_snake_polyfit_coeffs.txt"
        np.savetxt(save_filename,np.array((x_coeff,y_coeff,z_coeff,thy_coeff)))
        print("Coefficients saved to " + save_filename)
        print("Close Graph to Continue:")
        plt.show()


    def average_list_of_TransformStamped(self,list_of_TS):
        N = len(list_of_TS)
        average = TransformStamped()
        average.transform.rotation.w = sum([T.transform.rotation.w for T in list_of_TS])/N
        average.transform.rotation.x = sum([T.transform.rotation.x for T in list_of_TS])/N
        average.transform.rotation.y = sum([T.transform.rotation.y for T in list_of_TS])/N
        average.transform.rotation.z = sum([T.transform.rotation.z for T in list_of_TS])/N
        average.transform.translation.x = sum([T.transform.translation.x for T in list_of_TS])/N
        average.transform.translation.y = sum([T.transform.translation.y for T in list_of_TS])/N
        average.transform.translation.z = sum([T.transform.translation.z for T in list_of_TS])/N

        R = average.transform.rotation
        p = average.transform.translation
        q = np.array([R.x,R.y,R.z,R.w])
        q = q / np.linalg.norm(q)
        g = tr.quaternion_matrix(q)
        g[0:3,-1] = np.array([p.x,p.y,p.z])

        return g

    def tool_sub_callback(self,transform):
        self.tool_transform = transform

    def tip_sub_callback(self,transform):
        self.tip_transform = transform

    def bendleft_sub_callback(self,pos):
        self.bendleft_position = pos.data

    def bendright_sub_callback(self,pos):
        self.bendright_position = pos.data

    def move_abs_bendright(self, move_cmd):
        if not (self.bendright_limits[0] <= move_cmd <= self.bendright_limits[1]):
            print("Command: " + move_cmd + " is out of limit " + self.bendright_limits)
            return
        msg = Float32()
        msg.data = move_cmd
        self.bendright_pub.publish(msg)

    def move_abs_bendleft(self, move_cmd):
        if not (self.bendleft_limits[0] <= move_cmd <= self.bendleft_limits[1]):
            print("Command: " + move_cmd + " is out of limit " + self.bendleft_limits)
            return
        msg = Float32()
        msg.data = move_cmd
        self.bendleft_pub.publish(msg)



if __name__ == '__main__':
    try:
        # TODO: check if output dir exists
        calibration_save_directory = os.path.dirname(os.path.abspath(__file__))+"/output/"
        cal_node = motor_control(calibration_save_directory)
        cal_node.run()

    except rospy.ROSInterruptException:
        pass

