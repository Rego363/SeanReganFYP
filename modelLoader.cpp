#include <torch/script.h> 
#include <iostream>
#include <memory>

int main(int argc, const char* argv[])
{
	if(argc != 2)
	{
		std::cerr << "Usage: modelLoader <path-to-exported-script-module>\n";
		return -1;
	}
	
	// Deserialize the ScriptModule from a file using torch::jit::load().
	std::shared_ptr<torch::jit::script::Module> module = torch::jit::load(argv[1]);

	assert(module != nullptr);
	std::cout << "ok\n";

	// Create a vector of inputs.
	//std::vector<torch::jit::IValue> inputs;
	float sensors[36];
	for(int i = 0; i < 36; i++)
	{
		sensors[i] = 0.0f;	
	}
	// Put into numpy array
	//nPI = np.array(sensors);
	// Put into torch.Tensor
	//t = torch::Tensor(nPI);
	// Resize to (18, 36, 4, 4)
	//t.resize(18, 36, 4, 4);
	//inputs.push_back( );//t); 

	// Execute the model and turn its outputs into a tensor
	at::Tensor output = module->arrayForward(sensors).toTensor();
	std::cout << output << std::endl;
