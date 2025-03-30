# Makefile
# code by LinCC111 2025.1.13 Box2AI-Robotics copyright 盒桥智能 版权所有
TEMP_DIR := /tmp/joycon-install-temp
RULES_DIR := /etc/udev/rules.d
UDEV_RULES := udev/99-nitendo.rules
NINTENDO_REPO := https://github.com/nicman23/dkms-hid-nintendo
JOYNCON_REPO := https://github.com/DanielOgorchock/joycond

# DKMS INSTALL VERSION
DKMS_VERSION := 3.2

# INSTALL NINTENDO DKMS MODULES
install_nintendo:
	@echo "Cloning dkms-hid-nintendo repository..."
	@rm -rf $(TEMP_DIR)/dkms-hid-nintendo
	@mkdir -p $(TEMP_DIR)
	@cd $(TEMP_DIR) && git clone $(NINTENDO_REPO)
	@echo "Removing any previous instances of the nintendo module..."
	@cd $(TEMP_DIR)/dkms-hid-nintendo && sudo dkms remove nintendo -v $(DKMS_VERSION) --all || true
	@echo "Checking for any existing nintendo modules..."
	@sudo dkms status | grep -i nintendo && sudo dkms remove nintendo -v $(DKMS_VERSION) --all || true
	@cd $(TEMP_DIR)/dkms-hid-nintendo && sudo dkms add .
	@cd $(TEMP_DIR)/dkms-hid-nintendo && sudo dkms build nintendo -v $(DKMS_VERSION)
	@cd $(TEMP_DIR)/dkms-hid-nintendo && sudo dkms install nintendo -v $(DKMS_VERSION)
	@echo "nintendo dkms module installed successfully."

# INSTALL JOYCOND
install_joycond:
	@echo "Cloning joycond repository..."
	@rm -rf $(TEMP_DIR)/joycond 
	@cd $(TEMP_DIR) && git clone $(JOYNCON_REPO)
	@cd $(TEMP_DIR)/joycond && cmake .
	@cd $(TEMP_DIR)/joycond && sudo make install
	@cd $(TEMP_DIR)/joycond && sudo systemctl enable --now joycond
	@echo "joycond installed and started successfully."

# INSTALL UBUNTU SYSTEM DEPENDENCIES
install-hid-deps:
	sudo apt-get install -y \
		libhidapi-dev \
		libhidapi-hidraw0 \
		libhidapi-libusb0 \
		
# INSTALL JOYCON UDEV RULES
install-udev-rules:
	@echo "Installing udev rules..."
	@sudo cp $(UDEV_RULES) $(RULES_DIR)
	sudo udevadm control --reload-rules && sudo udevadm trigger
	@echo "Udev rules installed successfully."

# DELETE TEMPORARY FILES
clean:
	@echo "Cleaning up temporary files..."
	@rm -rf $(TEMP_DIR)
	@echo "Cleanup completed."

# INSTALL ALL TARGET
install: install_nintendo install_joycond clean install-hid-deps install-udev-rules
	@echo "All dependencies installed successfully."

