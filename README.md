# VLN - Offline Model for Walter

## Project Overview

## Download the VLN package (walter_vln_model) from MEGA before everything
It does not allow on github to upload all the files to the repository:
https://mega.nz/file/os8FBJQQ#p7d5zGNqlqH0-CUUNa-DgQcUHgbwVfElydu5D27kRSs

This repository represents an evolution of the research initiated in the [Walter Project](https://github.com/JrLuimira/walter_description). It provides an accessible implementation of a low-cost **Visual-Language-Navigation (VLN)** model. 

Unlike many current approaches that rely on paid cloud-based subscriptions (e.g., GPT-4, DeepSeek, or Gemini APIs), this framework focuses on **on-device inference** using fully local agents.

### Key Features

* **Local Inference:** Utilizes lightweight models such as **TinyLlama** to enable Embodied AI capabilities without external dependencies or latency.
* **Multimodal Interaction:** Integrates local Automatic Speech Recognition (ASR) and Text-to-Speech (TTS) models for voice interaction (download links provided below).
* **Reactive Navigation:** The current iteration employs a reactive navigation approach based on visual inputs. *Note: SLAM integration is planned for future work.*

### Comparative Analysis: Visual Encoders

This project conducted a performance benchmark on several Visual Encoders, including [OWL-ViT](https://huggingface.co/docs/transformers/model_doc/owlvit) and [Grounding DINO](https://huggingface.co/docs/transformers/en/model_doc/grounding-dino).

* **Result:** **YOLO** was selected as the optimal solution for standard computing hardware.
* **Reasoning:** While Transformer-based models (like DINO) offer high precision, they require significant VRAM and GPU resources. YOLO provides the best balance between performance and computational efficiency for mid-range PCs.

### Hardware & Requirements

The system works with a minimal sensor setup, relying solely on an **Intel RealSense D435 RGB-D Camera**.

**System Requirements:**
* **Storage:** ~12 GB (Local memory for models and dependencies).
* **RAM:** 6 - 8 GB minimum.
* **GPU:** A dedicated GPU is highly recommended for scalability and smoother inference, though basic operation is possible on standard hardware.

---

### Authors & Acknowledgments

This project is developed by researchers from **ESPOL**:

* **[Luigi Miranda Lemos](https://www.linkedin.com/in/luigi-miranda-lemos-33320b264/)**
* **[Melissa Cobos Condo](https://www.linkedin.com/in/melissa-cobos-094627269/)**

**Under the supervision and guidance of:**

* [Dr. Denny Paillacho, Ph.D.](https://www.cidis.espol.edu.ec/es/node/67)
* [BSc. Joel Hidalgo, M.Sc.](https://www.linkedin.com/in/joel-hidalgo-pisco/)
* [BSc. José Luis Laica, Eng.](https://www.linkedin.com/in/jos%C3%A9-laica-cornejo/)

# Steps for using the code

First activate the conda environment

```bash
 conda activate owl-vit

```

For using the Ultralytics packages and OWL-Vit models.
```bash
 ros2 launch camera_pkg camera.launch.py

```

This is for activating the brain of the robot.
```bash
ros2 run walter_vln_model tinyllama.py

```

Activate the mouth and ears of the robot. Here, it receives what you want him to do.
```bash
ros2 run walter_vln_model voice2text_node.py

```

It is important the you have your own local red to try it with micro-ros. It was tested in a ESP32 via WIFI
```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888

```

At the end, it is important to activate the movement.
```bash
ros2 run walter_vln_model vision_action.py
```

## System Architecture

The following diagram illustrates the integration of the **Vision-Language-Navigation (VLN)** model within the Walter platform. The architecture is designed to be modular, bridging high-level AI reasoning with low-level robotic control:

<img width="830" height="439" alt="image" src="https://github.com/user-attachments/assets/67179d96-c23a-412d-ae53-914e256b49b7" />

* **Perception Layer:** Utilizes an **Intel RealSense** RGB-D camera to capture environmental data, which is processed by **YOLOv8** for real-time object detection and spatial localization.
* **Cognitive Layer:** The "brain" of the system relies on **TinyLlama** for Natural Language Understanding (NLU), allowing the robot to interpret intent from **Vosk** (offline speech recognition).
* **Action Layer:** Executes **reactive indoor navigation** commands based on the visual and cognitive inputs, while maintaining strict operational security protocols to ensure safe movement.

# Libraries to install
```bash
 conda install -c conda-forge trimesh
 conda install -c conda-forge ftfy regex tqdm
 conda install -c conda-forge python-xlib
 pip install ultralytics opencv-python torch torchvision torchaudio
 sudo apt-get install python3-pyaudio portaudio19-dev
 pip install SpeechRecognition pyaudio`
 pip install gTTS`
 pip3 install pyttsx3 vosk pyaudio
 pip3 install llama-cpp-python
wget [https://github.com/rhasspy/piper/releases/download/2023.11.14-2/piper_linux_x86_64.tar.gz](https://github.com/rhasspy/piper/releases/download/2023.11.14-2/piper_linux_x86_64.tar.gz) # this goes inside the piper folder.
wget -O tinyllama-1.1b-chat-v1.0.Q4_K_M.gguf "https://huggingface.co/TheBloke/TinyLlama-1.1B-Chat-v1.0-GGUF/resolve/main/tinyllama-1.1b-chat-v1.0.Q4_K_M.gguf?download=true" #Inside the SLM-TinyLlama folder, this download the AI agentic
 ```

# References:
* https://github.com/real-stanford/cow/tree/main
* https://github.com/openvla/openvla
