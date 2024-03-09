import numpy as np
import time
from controller import Robot
import math

class Hexapod(Robot):
	def __init__(self):
		super(Hexapod, self).__init__()
		self.time_step = int(self.getBasicTimeStep())
		self.l1 = 0.04
		self.l2 = 0.08
		self.l3 = 0.12
		self.motors = {}     
		self.loop_counter = 0   
		self.motor_name = ['R0','R1','R2','L0','L1','L2']

		for leg in ['R0','R1','R2','L0','L1','L2']:
			for seg in ['0','1', '2']:
				self.motors[leg + '_' + seg] = self.getDevice(leg + '_' + seg)
		
		self.motion_right = []
		self.motion_left = []


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

		self.walking_sequence_ik = {}
		if gait == 'Tripod':
			self.tripod_motion_1L2R = []
			self.tripod_motion_1R2L = []
			swing_z_1 = np.linspace(0, height/2, int(step_num/2), endpoint=False)
			swing_z_2 = np.linspace(height/2, height, int(step_num/2), endpoint=True)
			swing_z = np.append(swing_z_1, swing_z_2, axis=None)
			stance_x_forward = np.linspace(length/2, -length/2, int(step_num), endpoint=False)
			stance_x_backward = np.linspace(-length/2, length/2, int(step_num), endpoint=True)
			tempa =[]
			tempb =[]
			for sxb,sxf, swz in zip(stance_x_backward,stance_x_forward ,swing_z):
				tempa.append(self.inverse_kinematic(sxb,0.109,-0.08+swz))
				tempb.append(self.inverse_kinematic(sxf,0.109,-0.08))
			self.tripod_motion_1L2R.extend(tempa)
			self.tripod_motion_1L2R.extend(tempb)

			self.tripod_motion_1R2L.extend(tempb)
			self.tripod_motion_1R2L.extend(tempa)

	def generate_motion_circle(self, data):
		gait = data['Gait']
		length = data['Length']
		height = data['Height']
		step_num = data['StepNum']
		alpha = data['Alpha']

		stance_pose = []

		self.walking_sequence_ik = {}
		if gait == 'Tripod':
			self.tripod_motion_1L2R = []
			self.tripod_motion_1R2L = []

			x1 = 0.5*length*math.sin(alpha)
			y1 = 0.5*length*math.cos(alpha)

			x0 = -x1
			y0 = -y1

			y0 = y0 + 0.109
			y1 = y1 + 0.109

			x_forward = np.linspace(x0,x1,step_num,endpoint=True)
			x_backward = np.linspace(x1,x0,step_num,endpoint=True)

			y_forward = np.linspace(y0,y1,step_num,endpoint=True)
			y_backward = np.linspace(y1,y0,step_num,endpoint=True)

			z_forward_a = np.linspace(0, height/2, int(step_num/2), endpoint=False)
			z_forward_b = np.linspace(height/2, height, int(step_num/2), endpoint=True)
			z_forward = np.append(z_forward_a, z_forward_b, axis=None)

			stance_R1 = []
			swing_R1 = []
			stance_L2 = []
			swing_L2 = []
			for x, y in zip(x_backward,y_backward):
				stance_R1.append(self.inverse_kinematic(x,y,-0.08))
			for x, y, z in zip(x_forward,y_forward,z_forward):
				swing_R1.append(self.inverse_kinematic(x,y,-0.08+z))

			for x, y in zip(x_forward,y_forward):
				stance_L2.append(self.inverse_kinematic(x,y,-0.08))
			for x, y, z in zip(x_backward,y_backward,z_forward):
				swing_L2.append(self.inverse_kinematic(x,y,-0.08+z))
			pose_L2 = stance_L2 + swing_L2
			pose_R1 = stance_R1 + swing_R1
			pose_R1L2 = {
				'Left':pose_L2,
				'Right': pose_R1,
			}
			
			return pose_R1L2

	def walk(self):
		indx = 10
		while not self.my_step():
			if self.loop_counter >= 1:
				indx += 1
				self.loop_counter = 0
			idx = indx % len(self.tripod_motion_1L2R)
			if indx == len(self.tripod_motion_1L2R) - 1:
				indx = 0
			print(f"self.time_step = {self.time_step} ----idx = {idx}")

			for name in self.motor_name:
				if name in 	['L0','L2','R1']:
					if name == "R1":
						self.motors[name + '_0'].setPosition(self.tripod_motion_1R2L[idx][0])				
						self.motors[name + '_1'].setPosition(self.tripod_motion_1R2L[idx][1])			
						self.motors[name + '_2'].setPosition(self.tripod_motion_1R2L[idx][2])				
					else:
						self.motors[name + '_0'].setPosition(-self.tripod_motion_1R2L[idx][0])
						self.motors[name + '_1'].setPosition(self.tripod_motion_1R2L[idx][1])
						self.motors[name + '_2'].setPosition(self.tripod_motion_1R2L[idx][2])

				else:
					if name == "L1":
						self.motors[name + '_0'].setPosition(-self.tripod_motion_1L2R[idx][0])				
						self.motors[name + '_1'].setPosition(self.tripod_motion_1L2R[idx][1])			
						self.motors[name + '_2'].setPosition(self.tripod_motion_1L2R[idx][2])				
					else:
						self.motors[name + '_0'].setPosition(self.tripod_motion_1L2R[idx][0])
						self.motors[name + '_1'].setPosition(self.tripod_motion_1L2R[idx][1])
						self.motors[name + '_2'].setPosition(self.tripod_motion_1L2R[idx][2])
	def inv_pose(self,pose):
		pose_inv={}
		Left = pose['Left']
		Right = pose['Right']

		Left_a = Left[:int(len(Left)/2)]
		Left_b = Left[int(len(Left)/2):int(len(Left))]
		Left_inv = Left_b + Left_a

		Right_a = Right[:int(len(Right)/2)]
		Right_b = Right[int(len(Right)/2):int(len(Right))]
		Right_inv = Right_b + Right_a
		pose_inv = {
				'Left':Left_inv,
				'Right': Right_inv,
			}
		return pose_inv
	def walk2(self,pose):
		indx = 10
		pose_inv = self.inv_pose(pose)

		while not self.my_step():
			if self.loop_counter >= 1:
				indx += 1
				self.loop_counter = 0
			idx = indx % len(pose['Left'])
			if indx == len(pose['Left']) - 1:
				indx = 0
			print(f"self.time_step = {self.time_step} ----idx = {idx}")

			for name in self.motor_name:
				if name in 	['L0','L2','R1']:
					if name == "R1":
						self.motors[name + '_0'].setPosition(pose['Right'][idx][0])				
						self.motors[name + '_1'].setPosition(pose['Right'][idx][1])			
						self.motors[name + '_2'].setPosition(pose['Right'][idx][2])				
					else:
						self.motors[name + '_0'].setPosition(-pose['Left'][idx][0])
						self.motors[name + '_1'].setPosition(pose['Left'][idx][1])
						self.motors[name + '_2'].setPosition(pose['Left'][idx][2])

				else:
					if name == "L1":
						self.motors[name + '_0'].setPosition(-pose_inv['Left'][idx][0])				
						self.motors[name + '_1'].setPosition(pose_inv['Left'][idx][1])			
						self.motors[name + '_2'].setPosition(pose_inv['Left'][idx][2])				
					else:
						self.motors[name + '_0'].setPosition(pose_inv['Right'][idx][0])
						self.motors[name + '_1'].setPosition(pose_inv['Right'][idx][1])
						self.motors[name + '_2'].setPosition(pose_inv['Right'][idx][2])

print("------------start-------------")
robot = Hexapod()
robot.init_pose(0,-math.pi/4,2*math.pi/3)
robot.init_pose(0,math.pi/2,0)

robot.generate_motion_step({
	'Gait':'Tripod',
	'Length': 0.06,
	'Height': 0.04,
	'StepNum':20
})
robot.walk()
# pose_R1L2 = robot.generate_motion_circle({
# 	'Gait':'Tripod',
# 	'Length': 0.06,
# 	'Height': 0.04,
# 	'StepNum':20,
# 	'Alpha':math.pi/4
# })


# robot.walk2(pose_R1L2)




