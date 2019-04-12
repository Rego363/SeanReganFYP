import torch
import torchvision
import torch.nn.functional as func
import numpy as np
import torch.optim as optim
import math

# Testing to see if i get expected output after training
def test(lineNum, testData, model):
	line = testData[lineNum]	# Test data from dataset
	line = line[:-1]		# Last character goes to next line so we remove that
	data = line.split(', ')		# Seperate data into individual elements
	nums = []			# Testing input
	expected = []			# Expected output

	for x in range(0, len(data)):	# Go through all the testing data
		number = float(data[x])	# Convert string to float

		if x < len(data) - 3:	# All the data in each line of the file is an input excluding the last 3 which are outputs
			nums.append(number)	# Add to test inputs		
		else:
			expected.append(number)	# Add to expected outputs

	nPI = np.array(nums)		# Convert test input array to a numpy array
	t1 = torch.Tensor(nPI)		# Make training data a tensor (size 36, )
	t1.resize_(18, 36, 4, 4)	# Resize to match Conv layer

	exTen = torch.Tensor(np.array(expected))	# Get expected output as tensor

	print("Actual: ", model.forward(t1))	# Display the output from the model
	print("Expected: ", exTen)		# Display what the model should be returning





# Call to train model
def train(exIn, exOut):
	nPI = np.array(exIn)	# Convert array to numpy array
	t = torch.Tensor(nPI)	# Make training data a tensor (size 36, )
	t.resize_(18, 36, 4, 4)	# Resize to match Conv layer

	it = 0	# to track how many iterations it has gone through this episode
	while optimize(my_script_module.forward(t), exOut):	# While the module still has a noticable loss
		print("Episode: ", epi, " / ", len(lines))	# Current episode
		print("Iteration: ", it)			# Current iteration
		it = it + 1					# Increment iteration counter





# Call to update weights
def optimize(act, exp):
	loss = func.smooth_l1_loss(act, exp)	# Get loss

	if loss.item() < 0.00001:	# If loss is not note worthy
		return False	# Tell training loop that we are done

	print("Loss: ", loss.item()) # Output loss to screen

	# Update weights
	optimizer.zero_grad()	# Set model parameters gradient to zero
	loss.backward() 	# Add loss gradient to model gradient
	for param in my_script_module.parameters():
		param.grad.data.clamp_(-1, 1)	# Clamping model gradient between -1 , 1
	optimizer.step()	# Update model parameters
	return True;	# Continue minimizing loss





# Deep Q Neural Network model
class DQN(torch.jit.ScriptModule):
	def __init__(self):
		super(DQN, self).__init__()	# Initialize 
		self.conv = torch.nn.Conv2d(36, 18, kernel_size=4, stride=2)	# Convolutional layer, 36 inputs, 18 outputs
		self.bn = torch.nn.BatchNorm2d(18)				# Batch Normalization, 18 inputs
		self.lin = torch.nn.Linear(18, 3)				# Linear layer, 18 inputs, 3 outputs

	
	@torch.jit.script_method
	def forward(self, input):
		# Pass inputs through layers
		out = self.conv(input)		# Convolutional Layer
		out = self.bn(out)		# Batch normalization
		out = func.relu(out)		# Rectified Linear Units
		out = out.view(out.size(0), -1)	# Resize so that it is 18 instead of 18 x 18
		out =  self.lin(out)		# Linear layer
		return out[0]	# return first array of 3 outputs (All arrays in the outer array are the same)



####################################################
####################################################
####################################################

my_script_module = DQN() # Model object

# Get training data
filehandler = open("TD.txt", "r")				# Open training file
lines = filehandler.readlines()					# Read line
filehandler.close()						# Close training file

optimizer = optim.RMSprop(my_script_module.parameters())	# create optimiser

epi = 0	# Episode counter

for line in lines:	# Loop through all the data in the 
	line = line[:-1]	# Last character goes to next line so we remove that
	data = line.split(', ')	# Seperate data into individual elements

	nums = []	# Training inputs
	expected = []	# Expected outputs

	for x in range(0, len(data)):	# Go through all the training data
		number = float(data[x])	# Convert string to float

		if x < len(data) - 3:	# All the data in each line of the file is an input excluding the last 3 which are outputs
			nums.append(number)	# Add to training inputs		
		else:
			expected.append(number)	# Add to expected outputs

	
	exTen = torch.Tensor(np.array(expected))	# Get expected output 
	train(nums, exTen)				# Train the model with training inputs and outputs

	epi = epi + 1					# Increment episodes counter

my_script_module.save("model.pt")	# Save model to be used in torcs

# Testing model and checking returns
print("TESTS!")
print("Test: 1")
test(1000, lines, my_script_module)
print("Test: 2")
test(3000, lines, my_script_module)
print("Test: 3")
test(5000, lines, my_script_module)
