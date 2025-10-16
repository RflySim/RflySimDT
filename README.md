
# 🛰️ Digital Twin Platform for UAVs Based on RflySim

This repository provides the **implementation, datasets, and pretrained models** for a **digital twin-enhanced UAV modeling platform**.
Built upon the **RflySim** simulation environment, this platform enables **real–virtual synchronization**, **sensor-level dynamic modeling**, and **hybrid data-driven + physics-informed learning** for intelligent unmanned aerial systems.

---

## 📘 Overview

This project aims to establish a **hybrid digital twin framework** for UAVs that integrates high-fidelity simulation with real flight data. Our approach uses a **feature clustering-based multi-channel soft vector-quantized (FC-MC-SVQ)** to encode multimodal sensor information and reconstruct physical dynamics in real time.

### ✨ Key Features

* 🛰️ **Digital–Physical Synchronization**
  Real and virtual UAV states evolve synchronously through the RflySim co-simulation interface.

* ⚙️ **Physics-Informed Learning**
  Combines data-driven latent encoding with physical priors for robust dynamic modeling.

* 🔗 **Multi-Channel Vector Quantization (MC-SVQ)**
  Decouples heterogeneous sensor signals through multiple codebooks, enabling interpretable latent embeddings.

* 🧩 **Hybrid Simulation–Data Fusion**
  Integrates real sensor logs and RflySim-generated synthetic data for domain bridging.

## 🔧 Installing RflySim 

> **Note:** > RflySim is a high-precision UAV simulation platform and development toolchain led by **Xunhua Dai (Central South University)** and developed by **Feisi Laboratory**.
> It supports hardware-in-loop simulation, multi-agent co-simulation, and digital twin synchronization.

* GitHub: [https://github.com/RflySim](https://github.com/RflySim)
* Official Website: [https://www.rflysim.com](https://www.rflysim.com)
* Download Website: [https://flyeval.com/docs/#/en/7_DownloadAndSupport/DownloadAndSupport](https://flyeval.com/docs/#/en/7_DownloadAndSupport/DownloadAndSupport)

```bibtex
[1] Dai X, Ke C, Quan Q, et al. RFlySim: Automatic test platform for UAV autopilot systems with FPGA-based hardware-in-the-loop simulations. Aerospace Science and Technology, 2021, 114: 106727.

[2] Dai X, Tu J, Quan Q. RflySim ToolChain: A Rapid Development and Validation Toolchain for Intelligent Unmanned Swarm Systems. Journal of Systems Engineering and Electronics, 2025, 36(4): 1077–1093.
```

---

## 📁 Repository Structure

```
├── data/.../                         # Real & simulated UAV datasets
│   ├── *_2_3.csv                     # Quadrotor takeoff flight data
│   ├── *_2_6.csv                     # Quadrotor hovering flight data
│   ├── raw*                          # Raw sensor logs
│   ├── sim*                          # High-fidelity simulation data from RflySim
│   └── sync*                         # Preprocessed and aligned data
│
├── include/
│   └── rflysim_sdk/                  # RflySim SDK package and APIs for co-simulation
│
├── src/
│   ├── model/                        # Simulink models of UAV twins
│   ├── control/                      # Real-time control scripts and co-simulation management
│   ├── network/                      # Neural network models and training scripts
│   │   └── mymodel.ipynb             # End-to-end training & inference pipeline
│   │   └── ....                      # Other pipeline
│   └── trained/                      # Trained model 
│
└── README.md                         # Project documentation
```

---

## 🧠 Model Architecture

### **1. MultiHeadVectorQuantizer**

Each sensor group (e.g., IMU, magnetometer, position) is encoded using a dedicated codebook.
This design enables **modality-specific quantization** and improves robustness to cross-sensor noise.

```python
class MultiHeadVectorQuantizer(nn.Module):
    def __init__(self, num_embeddings, embedding_dim, num_clusters, beta=0.25):
        super().__init__()
        self.embeddings = nn.ModuleList([
            nn.Embedding(num_embeddings, embedding_dim) for _ in range(num_clusters)
        ])
        self.beta = beta

    def forward(self, latents):
        quantized_outputs, vq_losses = [], []
        for embedding, sub_latents in zip(self.embeddings, latents):
            B, T, D = sub_latents.shape
            dist = torch.cdist(sub_latents.view(-1, D), embedding.weight) ** 2
            soft_assign = F.softmax(-dist, dim=1)
            quantized = torch.matmul(soft_assign, embedding.weight).view(B, T, D)
            loss = self.beta * F.mse_loss(quantized.detach(), sub_latents) + \
                   F.mse_loss(quantized, sub_latents.detach())
            quantized_outputs.append(sub_latents + (quantized - sub_latents).detach())
            vq_losses.append(loss)
        return quantized_outputs, torch.stack(vq_losses).sum()
```

---

### **2. Network**

A **multi-channel LSTM encoder–decoder** processes temporal dependencies within each sensor group.
Conditional embeddings introduce contextual awareness (e.g., flight phase or external condition).

**Key parameters:**

| Parameter        | Value             | Description                         |
| ---------------- | ----------------- | ----------------------------------- |
| `input_dim`      | 15                | Total sensor feature dimension      |
| `seq_len`        | 150               | Sequence length per training sample |
| `hidden_dims`    | [64, 128]         | LSTM encoder hidden dimensions      |
| `recon_dims`     | [128, 64, 32, 18] | Decoder reconstruction hierarchy    |
| `embedding_dim`  | 128               | Codebook embedding dimension        |
| `num_embeddings` | 512               | Number of discrete latent vectors   |
| `num_clusters`   | 4–6               | Number of sensor feature groups     |
| `beta`           | 0.25              | VQ commitment loss weight           |
| `optimizer`      | Adam(lr=1e-3)     | Optimization algorithm              |
| `seed`           | 42                | Random seed for reproducibility     |

---

## 📊 Dataset Description

Each sample contains **15 sensor and kinematic features**, covering IMU, magnetometer, and position–velocity information.

| Feature                   | Description                                      |
| :------------------------ | :----------------------------------------------- |
| `gyro_rad[0-2]`           | Angular velocity (rad/s) along X/Y/Z axes        |
| `accelerometer_m_s2[0-2]` | Linear acceleration (m/s²) along X/Y/Z axes      |
| `magnetometer_ga[0-2]`    | Magnetic field strength (gauss) along X/Y/Z axes |
| `x, y, z`                 | Position coordinates (m)                         |
| `vx, vy, vz`              | Velocity components (m/s)                        |

**Data types:**

* `*_2_3.csv` → Takeoff process flight data
* `*_2_6.csv` → Hovering process flight data
  Both are used for model training and validation after preprocessing and synchronization.

---

## ⚙️ Environment Setup

Dependencies (`requirements.txt`):

```
numpy
scipy
pandas
tqdm
matplotlib
scikit-learn
torch==1.13.1
torch_geometric==2.6.1
torch_sparse==0.6.18
torchvision==0.14.1
```

Configuration :

```yaml
model:
  name: FC-MC-SVQ
  input_dim: 15
  embedding_dim: 128
  num_embeddings: 512
  num_clusters: 4
  beta: 0.25
  hidden_dims: [64, 128]
  recon_dims: [128, 64, 32, 18]

train:
  batch_size: 4
  epochs: 250
  lr: 1e-3
  seed: 42
  device: cuda:0
```

---

## 💻 Training & Inference Pipeline

The complete **training and inference workflow** is implemented in:

> `src/network/mymodel.ipynb`

It includes:

* ✅ **Data loading** (real + simulated UAV datasets)
* 🧹 **Preprocessing** (normalization, sequence slicing, feature clustering)
* 🧠 **Model training** (VQVAE + optimizer setup)
* 📈 **Evaluation** (MAE, RMSE, Wasserstein distance)
* 💾 **Model saving & checkpointing**

---

## 🔬 Hardware Configuration

All experiments were conducted on the following system:

| Component | Specification                              |
| --------- | ------------------------------------------ |
| CPU       | 12th Gen Intel® Core™ i9-12900H (2.50 GHz) |
| GPU       | NVIDIA GeForce RTX 3060 Laptop GPU         |
| RAM       | 32 GB DDR5                                 |
| OS        | Windows 11                                 |

The training on this setup typically takes **≈2 hours** for 250 epochs with a batch size of 4.

---

## 🧩 Evaluation Metrics

| Metric       | Description                                                  |
| ------------ | ------------------------------------------------------------ |
| `MAE`        | Mean Absolute Error between reconstructed and input signals  |
| `RMSE`       | Root Mean Square Error                                       |
| `W-Loss`     | Wasserstein distance between real and reconstructed dynamics |

---

## 📫 Contact

**Author:** Tu Jinhu
**Affiliation:** Central South University, Feisi Laboratory
**Email:** [[tjhcsu@csu.edu.cn](tjhcsu@csu.edu.cn)]

