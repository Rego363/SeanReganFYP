import torch
import torchvision
import torch.nn.functional as func
import numpy as np
import torch.optim as optim
import math

def train(exIn, exOut):
	nPI = np.array(exIn)
	t = torch.Tensor(nPI)	# Make training data a tensor (size 36, )
	t.resize_(18, 36, 4, 4)	# Resize to match Conv layer

	it = 0
	while optimize(my_script_module.forward(t), exOut):
		print("Episode: ", epi, " / ", len(lines))
		print("Iteration: ", it)
		it = it + 1

def optimize(act, exp):
	#if len(memory) < BATCH_SIZE:
		#return
	#transitions = memory.sample(BATCH_SIZE)

	#batch = Transition(*zip(*transitions)) # Transpose the batch

	# non_final_mask = torch.tensor(tuple(map(lambda s: s is not None, batch.next_state)), device=device, dtype=torch.uint8)

	# non_final_next_states = torch.cat([s for s in batch.next_state 
						#if s is not None])

	# state_batch = torch.cat(batch.state)
	# action_batch = torch.cat(batch.action)
	# reward_batch = torch.cat(batch.reward)	

	# Get loss
	loss = func.smooth_l1_loss(act, exp)
	if loss.item() < 0.00001:
		# loss is tiny
		return False

	print("Loss: ", loss.item())

	# Update weights
	optimizer.zero_grad()
	loss.backward()
	for param in my_script_module.parameters():
		param.grad.data.clamp_(-1, 1)
	optimizer.step()
	return True;

class DQN(torch.jit.ScriptModule):
	def __init__(self):
		super(DQN, self).__init__()
		# Convolutional layer, 36 inputs, 18 outputs
		self.conv = torch.nn.Conv2d(36, 18, kernel_size=4, stride=2)
		# Batch Normalization, 18 inputs
		self.bn = torch.nn.BatchNorm2d(18)
		# Linear layer, 18 inputs, 3 outputs
		self.lin = torch.nn.Linear(18, 3)

	
	@torch.jit.script_method
	def forward(self, input):
		# Pass inputs through layers
		# Convolutional Layer
		out = self.conv(input)
		# Batch normalization
		out = self.bn(out)
		# Rectified Linear Units
		out = func.relu(out)
		# Resize so that it is 18 instead of 18 x 18
		out = out.view(out.size(0), -1)
		# Linear layer
		out =  self.lin(out)
		return out[0]

my_script_module = DQN()

 
filehandler = open("TD.txt", "r")	# Open training file
lines = filehandler.readlines()	# Read line
filehandler.close()	# Close training file
optimizer = optim.RMSprop(my_script_module.parameters())	# create optimiser

epi = 0

'''
for line in lines:
	print("Episode: ", epi, " / ", len(lines))
	epi = epi + 1
	line = lines[0]
	line = line[:-1]	# Last character goes to next line so we remove that
	data = line.split(', ')	# Seperate data into individual elements
	nums = []	# Training input
	expected = []	# Expected output
	for x in range(0, len(data)):	# Go through data
		if x < len(data) - 3:
			nums.append(float(data[x]))	# Add to training inputs
		else:
			expected.append(float(data[x]))	# Add to expected outputs

	nPI = np.array(nums)
	t1 = torch.Tensor(nPI)	# Make training data a tensor (size 36, )
	t1.resize_(18, 36, 4, 4)	# Resize to match Conv layer

	out = my_script_module.forward(t1)	# Pass through network
	exTen = torch.Tensor(np.array(expected))	# Get expected output as tensor
	optimizer = optim.RMSprop(my_script_module.parameters())	# create optimiser
	
	#it = 0
	optimize()
	
	if epi > 1:
		break;

	#print("Iteration: ", it)
	#it = it + 1
'''



for line in lines:
	#line = lines[0]
	line = line[:-1]	# Last character goes to next line so we remove that
	data = line.split(', ')	# Seperate data into individual elements
	nums = []	# Training input
	expected = []	# Expected output


	for x in range(0, len(data)):	# Go through data
		number = float(data[x])
		if x < len(data) - 3:
			nums.append(number)	# Add to training inputs		
		else:
			expected.append(number)	# Add to expected outputs
	nPI = np.array(nums)
	t1 = torch.Tensor(nPI)	# Make training data a tensor (size 36, )
	t1.resize_(18, 36, 4, 4)	# Resize to match Conv layer
	exTen = torch.Tensor(np.array(expected))	# Get expected output 
	train(t1, exTen)
	epi = epi + 1
	
# Testing to see if i get expected output after training
line = lines[0]
line = line[:-1]	# Last character goes to next line so we remove that
data = line.split(', ')	# Seperate data into individual elements
nums = []	# Training input
expected = []	# Expected output


for x in range(0, len(data)):	# Go through data
	if x < len(data) - 3:
		nums.append(float(data[x]))	# Add to training inputs
	else:
		expected.append(float(data[x]))	# Add to expected outputs

nPI = np.array(nums)
t1 = torch.Tensor(nPI)	# Make training data a tensor (size 36, )
t1.resize_(18, 36, 4, 4)	# Resize to match Conv layer
exTen = torch.Tensor(np.array(expected))	# Get expected output as tensor

print("Actual: ", my_script_module.forward(t1))
print("Expected: ", exTen)
my_script_module.save("model.pt")
