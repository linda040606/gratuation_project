# Missile Closed-Loop Control System (Simulink + MATLAB)

## 📌 项目简介
本项目基于 MATLAB/Simulink 构建了一个跨介质导弹（空气→水）闭环控制系统模型，研究其在介质突变条件下的姿态角稳定控制问题。

系统采用**外环角度控制 + 内环角速度控制 + 变参数 PID + 扰动观测器**的复合控制结构，实现对非线性、水动力时变系统的稳定控制。

---

## ⚙️ 系统结构

系统主要包括以下模块：

- Signal_Interface（信号接口与外环控制）
- Variable_PID（变参数PID内环控制）
- Disturbance_Observer（扰动观测器）
- Water_Dynamics（跨介质动力学模型）
- M_Comp（控制力矩合成）

---
