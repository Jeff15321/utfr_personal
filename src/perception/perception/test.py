import torch

# Check if CUDA is available
print("CUDA available:", torch.cuda.is_available())

# Get number of CUDA devices
print("CUDA device count:", torch.cuda.device_count())

if torch.cuda.is_available():
    # Get current CUDA device name
    print("CUDA device name:", torch.cuda.get_device_name(0))
    
    # Simple CUDA operation test
    x = torch.rand(5, 3)
    print("CPU tensor:", x)
    
    # Move tensor to GPU
    if torch.cuda.is_available():
        x = x.cuda()
        print("GPU tensor:", x)
        
    # Perform a simple operation
    y = x * 2
    print("Operation result:", y)