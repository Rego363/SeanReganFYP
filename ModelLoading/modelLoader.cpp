#include <torch/script.h> 
#include <iostream>
#include <memory>
#include <stdlib.h>

int main(int argc, const char* argv[])
{	
	torch::IntArrayRef s = torch::IntArrayRef{18,36, 4, 4};	// New shape of tensor
	std::vector<torch::jit::IValue> inputs; // Create a vector of inputs.	
	float sensors[36];			// Array of floats for inputs. 
	
	// Test inputs from the first frame of the race
	sensors[0] = 0.0f;
	sensors[1] = 6.0f;
	sensors[2] = -10.0f;
	sensors[3] = 0.0f;
	sensors[4] = 0.0f;
	sensors[5] = 6.0f;
	sensors[6] = -20.0f;
	sensors[7] = 0.0f;
	sensors[8] = 0.0f;
	sensors[9] = 6.0f;
	sensors[10] = -30.0f;
	sensors[11] = 0.0f;
	sensors[12] = 0.0f;
	sensors[13] = 6.0f;
	sensors[14] = -40.0f;
	sensors[15] = 0.0f;
	sensors[16] = -6.28321f;
	sensors[17] = 1.77618f;
	sensors[18] = 7.3335f;
	sensors[19] = 3.6665f;
	sensors[20] = 0.0336153f;
	sensors[21] = 0.100842f;
	sensors[22] = -0.0245407f;
	sensors[23] = 24.9966f;
	sensors[24] = 3.0f;
	sensors[25] = 11.0f;
	sensors[26] = 3.87097f;
	sensors[27] = 0.0f;
	sensors[28] = 3.0f;
	sensors[29] = 11.0f;
	sensors[30] = 3.93443f;
	sensors[31] = 0.0f;
	sensors[32] = 3.0f;
	sensors[33] = 11.0f;
	sensors[34] = 3.93443f;
	sensors[35] = 0.0f;
	
	// Load the model from a file
	std::shared_ptr<torch::jit::script::Module> module = torch::jit::load("model.pt");
	
	assert(module != nullptr);	// If the model isn't loaded alert user
	std::cout << "ok\n";	// Let user know model was loaded

	// Put inputs into tensor
	auto t = torch::tensor({sensors[0], sensors[1], sensors[2],sensors[3],
				sensors[4], sensors[5], sensors[6],sensors[7],
				sensors[8], sensors[9], sensors[10],sensors[11],
				sensors[12], sensors[13], sensors[14],sensors[15],
				sensors[16], sensors[17], sensors[18],sensors[19],
				sensors[20], sensors[21], sensors[22],sensors[23],
				sensors[24], sensors[25], sensors[26],sensors[27],
				sensors[28], sensors[29], sensors[30],sensors[31],
				sensors[32], sensors[33], sensors[34],sensors[35]});

	t.resize_(s);	// Resize tensor (36) to fit model (18, 36, 4,4)
	
	inputs.push_back(t); // Put tensor into inputs 

	auto output = module->forward(inputs).toTensor();	// Pass inputs through model 
	std::cout << output << std::endl;			// Display output to user
	
}
