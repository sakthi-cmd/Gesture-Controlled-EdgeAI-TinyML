# Gesture-Controlled-EdgeAI-TinyML
## Embedded Implementation Details

The firmware target runs on an STM32 platform utilizing the **X-CUBE-AI** hardware abstraction runtime to execute local, real-time inference window checks.

* **Sensor Interfacing:** Implements low-level **I2C memory-mapped transactions** to initialize, wake, and read raw data from an **MPU6050 Inertial Measurement Unit (IMU)**.
* **Feature Extraction Streaming:** Streams 3-axis gyroscope data (`Gx`, `Gy`, `Gz`) at 100ms processing windows, managing data loading dynamically via an array sequence logic buffer (`aiInData`).
* **Hardware Execution:** Invokes the optimized model runtime computation (`ai_network_run`) once the stride buffer constraint matches `AI_NETWORK_IN_1_SIZE`.
* **Output Processing:** Leverages a lightweight `argmax` algorithm to isolate structural multi-class probability scores ("pitch", "roll", "yaw") via localized inference.
* **Telemetry Output:** Implements custom target system retargeting (`_write`) to map standard `printf` commands directly onto hardware **USART1 peripheral data registers** for logging.
  
## System Architecture & Workflow
1. **Data Collection:** Aggregates real-time motion telemetry from an MPU6050 IMU.
2. **Model Training:** Neural network trained to map sequential sensory patterns into structural coordinate predictions.
3. **Quantization:** Converted the trained model using 32-bit floating-point parameters to minimize flash memory allocation and power draw.
4. **Hardware Deployment:** Integrated the optimized TinyML execution engine directly onto microprocessing hardware for sub-millisecond local inference.

## Tech Stack
* **Frameworks:** PyTorch / TensorFlow, X-CUBE-AI, TinyML
* **Languages:** Python (Training & Optimization), Embedded C (Hardware Inference)
