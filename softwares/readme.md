# 🚀 USB Driver Setup Guide for STM32 on Windows and Linux

This guide provides instructions for setting up the USB driver for STM32 devices on both **Windows** and **Linux** operating systems. Follow the steps below to ensure proper installation and functionality. 🛠️

---

## **🖥️ For Windows Users**

### **Step 1: Download the Official STM Driver**
1. Visit the official STM driver download page: [STSW-LINK009](https://www.st.com/en/development-tools/stsw-link009.html). 🌐
2. Download the driver package compatible with your Windows version. 📥

### **Step 2: Install the Driver**
1. Extract the downloaded ZIP file to a folder on your computer. 📂
2. Open the extracted folder and locate the installer executable (e.g., `st-link_winusb_install.exe`). 🔍
3. Run the installer and follow the on-screen instructions to complete the installation. ⚙️
4. Restart your computer if prompted. 🔄

### **Step 3: Verify Installation**
1. Connect your STM32 device to your computer via USB. 🔌
2. Open **Device Manager** and check if the STM32 device is listed under "Universal Serial Bus devices" or a similar category. 🖥️
3. If the device is recognized without any warning icons, the driver installation is successful. ✅

---

## **🐧 For Linux Users**

### **Step 1: Install the Open Source STM32 Library**
1. Open a terminal on your Linux system. 💻
2. Clone the `stlink` repository from GitHub:
   ```bash
   git clone https://github.com/stlink-org/stlink.git
Navigate to the cloned directory:

```bash
cd stlink
Install the required dependencies and build the library:
```

```bash
sudo apt-get update
sudo apt-get install cmake libusb-1.0-0-dev
mkdir build
cd build
cmake ..
make
sudo make install
```

Step 2: Verify Installation
Connect your STM32 device to your computer via USB. 🔌

Run the following command to check if the device is detected:

```bash
st-info --probe
```

If the device is detected and information is displayed, the installation is successful. 🎉

📚 Additional Resources
🎥 Video Tutorial: For a step-by-step visual guide, watch this video: STM32 USB Driver Setup Tutorial.

🛠️ Troubleshooting
🖥️ Windows: If the device is not recognized, ensure that the driver is correctly installed and try using a different USB port. 🔄

🐧 Linux: If the st-info --probe command does not detect the device, ensure that the udev rules are properly configured. 
Refer to the Stlink GitHub repository for detailed troubleshooting steps. 🔧

📞 Support
For further assistance, refer to the official STM32 documentation or visit the ST Community Forum. 🤝

⚠️ Note: Ensure that your STM32 device is powered and properly connected during the setup process. 🔋
