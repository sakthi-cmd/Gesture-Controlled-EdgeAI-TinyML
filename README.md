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

## Active Hardware Actuation & Peripheral Control

This production iteration shifts beyond purely predicting data arrays to closing the hardware-in-the-loop (HIL) automation system by implementing direct peripheral feedback.

* **Autonomous I2C Bus Diagnostics:** Implements an un-mapped `HAL_I2C_Master_Transmit` address scan sweeping from `0x08` to `0x78` during initial setup boot sequences to dynamically verify sensor attachment sanity prior to running the critical application layers.
* **Hardware Self-Diagnostic Sequences:** Triggers an automatic startup sweeping test on **TIM2 Channel 3** across structural pulse comparative windows (`50` ➔ `100` ➔ `75`) to calibrate peripheral hardware boundaries.
* **Deterministic PWM Servo Driving:** Uses mathematical mapping arrays to bind prediction classifications directly to automated physical servos. High-confidence classifications ($\ge 80\%$) programmatically scale real-time duty cycle configurations via `__HAL_TIM_SET_COMPARE`:
  * **Pitch Detection:** Maps to duty register index `25`.
  * **Roll Detection:** Maps to duty register index `75`.
  * **Yaw Detection:** Maps to duty register index `125`.
* **Low Confidence Safety Rejection:** Automatically filters out classifications dropping below the $0.80$ probability constraint margin to actively prevent false physical servo actuation.
