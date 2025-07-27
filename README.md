# CryptiComm 🔐📟
A bare-metal communication demo project on STM32F4 that **encrypts user data with AES** and **transmits it via I2C**, while **displaying messages on an LCD** — all using **drivers written from scratch**.

---

## ✨ Features

- ✅ **Bare-metal drivers** for:
  - GPIO
  - I2C (Master/Slave)
  - LCD (16x2 in 4-bit mode)
- 🔐 **AES-128 ECB mode encryption** using a minimal prewritten algorithm
- 📟 **LCD output** for displaying status, messages, or plaintext
- 📡 **I2C communication** of encrypted data
- 🧩 Fully modular code using `C` (no HAL, no CMSIS abstraction)

---

## 📦 Project Structure

```bash
.
├── Drivers/
│   ├── Src/         # Source files for SPI, I2C, LCD, GPIO, AES
│   └── Inc/         # Headers for all peripheral drivers
├── LCD_Test/        # Main application code
├── aes.c/.h         # Lightweight ECB-mode AES implementation
├── README.md        # This file
└── ...

