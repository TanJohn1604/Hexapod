import numpy as np
import time

import math

class Hexapod():
	def __init__(self):
		super(Hexapod, self).__init__()
		# self.time_step = int(self.getBasicTimeStep())
		self.l1 = 0.04
		self.l2 = 0.08
		self.l3 = 0.12
		# self.motors = {}     
		# self.loop_counter = 0   
		# for leg in ['R0','R1','R2','L0','L1','L2']:
		# 	for seg in ['0','1', '2']:
		# 		self.motors[leg + '_' + seg] = self.getDevice(leg + '_' + seg)
		self.test_stance = []
	def init_pose(self, a=0, b=np.pi/4, c=np.pi/4):
		for leg in ['R0','R1','R2','L0','L1','L2']:
			self.motors[leg + '_0'].setPosition(a)
			self.motors[leg + '_1'].setPosition(b)
			self.motors[leg + '_2'].setPosition(c)
			
	def set_pose(self, pose:dict):
		'''poses is a dict of dicts: {"leg_name":{"leg_seg":angle}}
		'''
		for leg, v in pose.items():
			for seg, angle in v.items():
				self.motors[leg + '_' + seg].setPosition(angle)
				self.current_pose[leg][seg] = angle
	def walk(self):

		t0,t1,t2 = self.inverse_kinematic(0.1155,0.14,-0.06)


		while not self.my_step():
			print(f"self.time_step = {self.time_step} ----self.loop_counter = {self.loop_counter }")
			if self.loop_counter >0:			
				self.motors['R0' + '_0'].setPosition(t0)
				self.motors['R0' + '_1'].setPosition(t1)
				self.motors['R0' + '_2'].setPosition(t2)
			if self.loop_counter > 100:
				self.motors['R0' + '_0'].setPosition(0)
				self.motors['R0' + '_1'].setPosition(0)
				self.motors['R0' + '_2'].setPosition(0)	
			if self.loop_counter > 200:
				self.loop_counter = 0
	def my_step(self):
		if self.step(self.time_step) == -1:
			return 1
		else:
			self.loop_counter += 1
			
			return 0
	def inverse_kinematic(self,x3,y3,z3):
		theta = []
		r1 = self.l1
		r2 = self.l2
		r3 = self.l3
		
		theta0 = math.atan(x3/y3)
	
		
		theta1_a = math.acos(round((r3**2*r1 - r2**2*r1 + r2**2*y3 - r3**2*y3 - 3*r1*y3**2 + 3*r1**2*y3 - r1*z3**2 + y3*z3**2 - r1**3 + y3**3 + z3*(- r2**4 + 2*r2**2*r3**2 + 2*r2**2*r1**2 - 4*r2**2*r1*y3 + 2*r2**2*y3**2 + 2*r2**2*z3**2 - r3**4 + 2*r3**2*r1**2 - 4*r3**2*r1*y3 + 2*r3**2*y3**2 + 2*r3**2*z3**2 - r1**4 + 4*r1**3*y3 - 6*r1**2*y3**2 - 2*r1**2*z3**2 + 4*r1*y3**3 + 4*r1*y3*z3**2 - y3**4 - 2*y3**2*z3**2 - z3**4)**(1/2))/(2*(r2**2)**(1/2)*(r1**2 - 2*r1*y3 + y3**2 + z3**2)),6))
		theta1_b = - math.acos(round((r2**2*r1 - r3**2*r1 - r2**2*y3 + r3**2*y3 + 3*r1*y3**2 - 3*r1**2*y3 + r1*z3**2 - y3*z3**2 + r1**3 - y3**3 + z3*(- r2**4 + 2*r2**2*r3**2 + 2*r2**2*r1**2 - 4*r2**2*r1*y3 + 2*r2**2*y3**2 + 2*r2**2*z3**2 - r3**4 + 2*r3**2*r1**2 - 4*r3**2*r1*y3 + 2*r3**2*y3**2 + 2*r3**2*z3**2 - r1**4 + 4*r1**3*y3 - 6*r1**2*y3**2 - 2*r1**2*z3**2 + 4*r1*y3**3 + 4*r1*y3*z3**2 - y3**4 - 2*y3**2*z3**2 - z3**4)**(1/2))/(2*(r2**2)**(1/2)*(r1**2 - 2*r1*y3 + y3**2 + z3**2)),6)) + math.pi
		theta2_a = - math.acos(round((r2**2 + r3**2 - r1**2 + 2*r1*y3 - y3**2 - z3**2)/(2*(r2**2)**(1/2)*(r3**2)**(1/2)),6)) + math.pi
		theta2_b = - math.acos(round((r2**2 + r3**2 - r1**2 + 2*r1*y3 - y3**2 - z3**2)/(2*(r2**2)**(1/2)*(r3**2)**(1/2)),6)) + math.pi

		
		y2_a = (r3**2*r1 - r2**2*r1 + r2**2*y3 - r3**2*y3 - r1*y3**2 - r1**2*y3 + r1*z3**2 + y3*z3**2 + z3*((r2**2 + 2*r2*r3 + r3**2 - r1**2 + 2*r1*y3 - y3**2 - z3**2)*(- r2**2 + 2*r2*r3 - r3**2 + r1**2 - 2*r1*y3 + y3**2 + z3**2))**(1/2) + r1**3 + y3**3)/(2*(r1**2 - 2*r1*y3 + y3**2 + z3**2))
		y2_b = (r3**2*r1 - r2**2*r1 + r2**2*y3 - r3**2*y3 - r1*y3**2 - r1**2*y3 + r1*z3**2 + y3*z3**2 - z3*((r2**2 + 2*r2*r3 + r3**2 - r1**2 + 2*r1*y3 - y3**2 - z3**2)*(- r2**2 + 2*r2*r3 - r3**2 + r1**2 - 2*r1*y3 + y3**2 + z3**2))**(1/2) + r1**3 + y3**3)/(2*(r1**2 - 2*r1*y3 + y3**2 + z3**2))

		z2_a = (r2**2*z3 - r3**2*z3 + r1**2*z3 + y3**2*z3 + r1*((r2**2 + 2*r2*r3 + r3**2 - r1**2 + 2*r1*y3 - y3**2 - z3**2)*(- r2**2 + 2*r2*r3 - r3**2 + r1**2 - 2*r1*y3 + y3**2 + z3**2))**(1/2) - y3*((r2**2 + 2*r2*r3 + r3**2 - r1**2 + 2*r1*y3 - y3**2 - z3**2)*(- r2**2 + 2*r2*r3 - r3**2 + r1**2 - 2*r1*y3 + y3**2 + z3**2))**(1/2) + z3**3 - 2*r1*y3*z3)/(2*(r1**2 - 2*r1*y3 + y3**2 + z3**2))
		z2_b = (r2**2*z3 - r3**2*z3 + r1**2*z3 + y3**2*z3 - r1*((r2**2 + 2*r2*r3 + r3**2 - r1**2 + 2*r1*y3 - y3**2 - z3**2)*(- r2**2 + 2*r2*r3 - r3**2 + r1**2 - 2*r1*y3 + y3**2 + z3**2))**(1/2) + y3*((r2**2 + 2*r2*r3 + r3**2 - r1**2 + 2*r1*y3 - y3**2 - z3**2)*(- r2**2 + 2*r2*r3 - r3**2 + r1**2 - 2*r1*y3 + y3**2 + z3**2))**(1/2) + z3**3 - 2*r1*y3*z3)/(2*(r1**2 - 2*r1*y3 + y3**2 + z3**2))

		if z2_a < 0:
			theta1_a = - theta1_a
		if z2_b < 0:
			theta1_b = - theta1_b
		if self.ModAngle(y3,z3,r1,0,-theta1_a) < 0:
			theta2_a = - theta2_a
		if self.ModAngle(y3,z3,r1,0,-theta1_b) < 0:
			theta2_b = - theta2_b
		# điều chỉnh theo hệ tọa độ của motor trong webots
		theta1_a = - theta1_a
		theta1_b = - theta1_b
		theta2_a = - theta2_a
		theta2_b = - theta2_b
		if z2_a >= 0:
			theta = [theta0,theta1_a,theta2_a]
		if z2_b >= 0:
			theta = [theta0,theta1_b,theta2_b]
		return theta
	def ModAngle(self,x1,y1,x0,y0,theta):
		#  x2 = x0 + (x1 - x0) * math.cos(theta) - (y1 - y0) * math.sin(theta);
		y2 = y0 + (x1 - x0) * math.sin(theta) + (y1 - y0) * math.cos(theta)
		return y2
	def generate_motion_step(self, data):
		gait = data['Gait']
		length = data['Length']
		height = data['Height']
		step_num = data['StepNum']

		stance_pose = []
		swing_pose = []
		self.walking_sequence_ik = {}
		if gait == 'Tripod':
			
			swing_z_1 = np.linspace(0, height/2, int(step_num/2), endpoint=False)
			swing_z_2 = np.linspace(height/2, height, int(step_num/2), endpoint=True)
			swing_z = np.append(swing_z_1, swing_z_2, axis=None)

			print(len(swing_z))
			print(swing_z)
			swing_y = np.linspace(-length/2, length/2, int(step_num), endpoint=True)

			stance_x_forward = np.linspace(-length/2, length/2, int(step_num), endpoint=False)
			stance_x_backward = np.linspace(length/2, -length/2, int(step_num), endpoint=True)
			stance_x = np.append(stance_x_forward, stance_x_backward, axis=None)
			# print(len(stance_x))

			for sty in stance_x:
				self.test_stance.append(self.inverse_kinematic(sty,0.109,-0.08))
			# print(swing_pose)
			# for sty, swy, swz in zip(stance_y, swing_y, swing_z):
			# 	stance_pose.append(self.solve_body_ik([0,0,0],[0, -sty, 0]))
			# 	swing_pose.append(self.solve_body_ik([0,0,0],[0, -swy, -swz]))
			
			# for n in self.motors.keys():
			# 	leg = n.split('_')[0]
			# 	if leg in ('MR', 'FL', 'HL'):
			# 		self.walking_sequence_ik[leg] = {
			# 			'COXA':[p[leg]['COXA'] for p in stance_pose] + [p[leg]['COXA'] for p in swing_pose],
			# 			'FEMUR':[p[leg]['FEMUR'] for p in stance_pose] + [p[leg]['FEMUR'] for p in swing_pose],
			# 			'TIBIA':[p[leg]['TIBIA'] for p in stance_pose] + [p[leg]['TIBIA'] for p in swing_pose],
			# 		}
			# 	else:
			# 		self.walking_sequence_ik[leg] = {
			# 			'COXA':[p[leg]['COXA'] for p in swing_pose] + [p[leg]['COXA'] for p in stance_pose],
			# 			'FEMUR':[p[leg]['FEMUR'] for p in swing_pose] + [p[leg]['FEMUR'] for p in stance_pose],
			# 			'TIBIA':[p[leg]['TIBIA'] for p in swing_pose] + [p[leg]['TIBIA'] for p in stance_pose],
			# 		}


print("------------start-------------")
robot = Hexapod()

robot.generate_motion_step({
	'Gait':'Tripod',
	'Length': 0.1,
	'Height': 0.06,
	'StepNum':20
})

a=[1,2,3]
b=a[0:1]
print(b)