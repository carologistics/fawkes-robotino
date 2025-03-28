# Performance Tuning for Object Tracking on Raspberry Pi 5

This document outlines configuration parameters to optimize the object tracking plugin performance on Raspberry Pi 5.

## Configuration Parameters

Add these parameters to your configuration file:

```yaml
plugins/object_tracking/perf:
  # Basic performance options
  # ------------------------
  # Use hardware acceleration if available (OpenCL on RPi5)
  use_hardware_accel: true
  
  # Scale factor for input images before processing (0.5 = half resolution)
  # Lower values improve performance at the cost of detection quality
  # Recommended range: 0.3 to 0.7
  input_scale_factor: 0.5
  
  # Enable detailed performance logging
  enable_logging: false
  
  # Skip frames for higher performance (0 = process every frame)
  # Higher values improve CPU usage but reduce detection rate
  skip_frames: 0
  
  # Advanced performance options
  # ---------------------------
  # Apply 8-bit quantization to the model (reduces precision but improves speed)
  use_model_quantization: false
  
  # Only process a region of interest in the image
  use_roi_detection: false
  
  # Scale factor for ROI (percentage of original image size)
  roi_scale_factor: 0.8
  
  # Multi-core processing options
  # ----------------------------
  # Enable multi-core processing (uses all cores of RPi5)
  use_multi_core: true
  
  # Number of threads to use (default: 4 for RPi5 quad-core)
  num_threads: 4
  
  # Pin thread to specific CPU core (not recommended when multi-core is enabled)
  use_cpu_pinning: false
  
  # CPU core to use if pinning is enabled (0-based index, 0-3 for RPi5)
  cpu_core_id: 1
  
  # Automatically adjust quality settings to maintain target FPS
  use_adaptive_scaling: true
  
  # Target frames per second for adaptive scaling
  target_fps: 15.0
  
  # Convert color image to grayscale before processing (may improve performance)
  use_grayscale: false
  
  # Thread priority (0 = default, higher values = higher priority)
  thread_priority: 0
  
  # Enable matrix caching (reduces memory allocations)
  enable_caching: true
```

## Recommended Settings for Different Scenarios

### Maximum Performance
```yaml
input_scale_factor: 0.3
skip_frames: 2
use_roi_detection: true
roi_scale_factor: 0.7
use_multi_core: true
num_threads: 4
use_grayscale: true
use_model_quantization: true
```

### Balanced Performance and Quality
```yaml
input_scale_factor: 0.5
skip_frames: 1
use_roi_detection: false
use_multi_core: true
num_threads: 4
use_adaptive_scaling: true
target_fps: 15.0
```

### Maximum Quality
```yaml
input_scale_factor: 1.0
skip_frames: 0
use_hardware_accel: true
use_multi_core: true
num_threads: 4
```

## Raspberry Pi 5 Quad-Core Optimizations

The Raspberry Pi 5 has a quad-core Cortex-A76 CPU that can be fully utilized for image processing:

1. **Quad-Core Processing**: 
   - Enable `use_multi_core: true` to use all four CPU cores
   - Set `num_threads: 4` to match the number of cores on the RPi5
   - Disable thread pinning to allow the OS to schedule threads on all cores

2. **Hardware Acceleration**:
   - Enable OpenCL acceleration with `use_hardware_accel: true`
   - The RPi5 GPU can help accelerate certain operations when OpenCL is available

3. **Memory Management**:
   - Enable caching to reduce memory allocations
   - Consider adjusting the `/boot/config.txt` to allocate more GPU memory if using OpenCL
   - Recommended settings for `/boot/config.txt`:
     ```
     gpu_mem=128
     ```

4. **Parallel Processing Strategies**:
   - Neural network inference uses all cores by default
   - Image preprocessing is distributed across cores
   - Post-processing operations are parallelized when beneficial

5. **Thermal Management**:
   - Using all four cores will generate more heat
   - Ensure adequate cooling or consider reducing the workload if thermal throttling occurs
   - Monitor temperatures with `vcgencmd measure_temp`
   - Consider adding a heatsink and/or fan to maintain peak performance

## Troubleshooting

If you experience issues with multi-core processing:
1. Check if OpenMP is available in your OpenCV build
2. Try gradually reducing the number of threads (4→3→2→1) to find the optimal setting
3. Monitor CPU usage with `htop` to verify all cores are being utilized
4. Check for thermal throttling with `vcgencmd get_throttled`

For hardware acceleration issues:
1. Verify OpenCL is working with `clinfo` command
2. Check OpenCV was built with OpenCL support
3. Update firmware and drivers to the latest version
