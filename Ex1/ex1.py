import math


#Exercise 1.1 ----------------------------------------

class TMatrix:

	#sets matrix according to list of input values
	def __init__(self, *args):
		input = [0] * 16
		if len(args) == 0:
			self.values = [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]
		else:
			for i in range( len(args[0]) ):
				input[i] = args[0][i]
			self.values = [input[0:4], input[4:8], input[8:12], input[12:16]]

	def mult(self, other_matrix):
		if type(self) != type(other_matrix):
			print("other matrix is of wrong type")
			return

		rtn_mat = [[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]]

		for row in range (4):
			for col in range(4):
				sum = 0
				for i in range(4):
					sum += self.values[row][i] * other_matrix.values[i][col]
				rtn_mat[row][col] = sum

		return rtn_mat

	def mult_vec(self, vector):
		rtn_vec = Vector4(0,0,0,0)

		for col in range(4):
			sum = 0
			for i in range(4):
				sum += self.values[i][col] * vector[col]
			rtn_vec[col] = sum

		return rtn_vec

	def __eq__(self, other):
		for row in range (4):
			for col in range(4):
				if self.values[row][col] != other.values[row][col]:
					return False


# print("Mat 1")
# mat1 = TMatrix()

# print("Mat 2")
# mat2 = TMatrix([0,1,2,3,4,5,6,7,5])

#test type comparison
# TMatrix.mult(mat1,"egg")

A = TMatrix([1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16])
B = TMatrix([1,3,5,7,9,11,13,15,2,4,6,8,10,12,14,16])

print("A x B =")
result = TMatrix.mult(A, B)
print(result)


#Exercise 1.2 ----------------------------------------

def make_trans_mat(x, y, z):
	return TMatrix([1,0,0,0,0,1,0,0,0,0,1,0,x,y,z,1])

def make_rot_mat(degree,axis):
	d = math.radians(degree);
	if axis == 'x':
		return TMatrix([1,0,0,0,0, math.cos(d), -math.sin(d),0,0,math.sin(d), math.cos(d),0,0,0,0,1])
	elif axis == 'y':
		return TMatrix([math.cos(d),0,math.sin(d), 0,0,1,0,0,-math.sin(d),0,math.cos(d),0,0,0,0,1])
	elif axis == 'z':
		return TMatrix([math.cos(d),-math.sin(d),0,0,math.sin(d),math.cos(d),0,0,0,0,1,0,0,0,0,1])



def make_scale_mat(sx, sy, sz):
	return TMatrix([sx,0,0,0,0,sy,0,0,0,0,sz,0,0,0,0,1])


print(make_trans_mat(1,2,3).values)
print(make_rot_mat(45,'x').values)
print(make_rot_mat(90, 'y').values)
print(make_rot_mat(120, 'z').values)
print(make_scale_mat(1,2,3).values)



#Exercise 1.3----------------------------------------

class Vector4:
	def __init__(self, *args):
		for x in range(4):
			self.x = args[0]
			self.y = args[1]
			self.z = args[2]
			self.w = args[3]

	def __getitem__(self, index):
		if index == 0:
			return self.x
		elif index == 1:
			return self.y
		elif index == 2:
			return self.z
		elif index == 3:
			return self.w

	def __setitem__(self, key, value):
		if key == 0:
			self.x = value
		elif key == 1:
			self.y = value
		elif key == 2:
			self.z = value
		elif key == 3:
			self.w = value

	def __str__(self):
		return "(" + str(self.x) + "," + str(self.y) + "," +  str(self.z) + "," +  str(self.w) + ")"
		
def euclidean_distance(point1,point2):

	dist = math.pow((point1.x - point2.x), 2)
	dist += math.pow((point1.y - point2.y) ,2)
	dist += math.pow((point1.z - point2.z) ,2)
	dist = math.sqrt(dist)
	return dist

print(euclidean_distance(Vector4(2, 4, 6, 2), Vector4(0, 0, 0, 1)))


#Exercise 1.4 ----------------------------------------

print("A * v = ")
v = Vector4(1,2,3,1)
print(A.mult_vec(v))


#Exercise 1.5 ----------------------------------------

#alpha and beta can both be 0

# for i in range(360):
# 	for j in range(1,360):
# 		alpha = i
# 		beta = j

# 		ls = make_rot_mat(90,'x').mult(make_rot_mat(alpha, 'z'))
# 		rs = make_rot_mat(beta,'y').mult(make_rot_mat(90, 'x'))

# 		if ls == rs:
# 			print("Found woo: " + str(alpha) + ", " + str(beta))
# 			print("ls: " + str(ls))
# 			print("rs: " + str(rs))



alpha = 90
beta = 180
ls = make_rot_mat(90,'x').mult(make_rot_mat(alpha, 'z'))
rs = make_rot_mat(beta,'y').mult(make_rot_mat(90, 'x'))

print("ls: " + str(ls))
print("rs: " + str(rs))

#Exercise 1.6 ----------------------------------------
#drawing...