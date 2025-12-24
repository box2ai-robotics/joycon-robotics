# Makefile
# code by Boxjod 2025.12.23 Box2AI-Robotics copyright 盒桥智能 版权所有

TEMP_DIR := /tmp/joycon-install-temp
RULES_DIR := /etc/udev/rules.d
UDEV_RULES := udev/99-nitendo.rules
LOCAL_DIR := joyconrobotics/system_lib
NINTENDO_DIR := $(LOCAL_DIR)/dkms-hid-nintendo
JOYCOND_DIR := $(LOCAL_DIR)/joycond

# DKMS INSTALL VERSION
DKMS_VERSION := 3.2

# CHECK IF LOCAL REPOS EXIST
check-local-repos:
	@if [ ! -d "$(NINTENDO_DIR)" ]; then \
		echo "Error: dkms-hid-nintendo directory not found at $(NINTENDO_DIR)"; \
		exit 1; \
	fi
	@if [ ! -d "$(JOYCOND_DIR)" ]; then \
		echo "Error: joycond directory not found at $(JOYCOND_DIR)"; \
		exit 1; \
	fi
	@echo "Local repositories found successfully."

# INSTALL NINTENDO DKMS MODULES
install_nintendo: check-local-repos
	@echo "Using local dkms-hid-nintendo repository..."
	@echo "Removing any previous instances of the nintendo module..."
	@cd $(NINTENDO_DIR) && sudo dkms remove nintendo -v $(DKMS_VERSION) --all || true
	@echo "Checking for any existing nintendo modules..."
	@sudo dkms status | grep -i nintendo && sudo dkms remove nintendo -v $(DKMS_VERSION) --all || true
	@cd $(NINTENDO_DIR) && sudo dkms add .
	@cd $(NINTENDO_DIR) && sudo dkms build nintendo -v $(DKMS_VERSION)
	@cd $(NINTENDO_DIR) && sudo dkms install nintendo -v $(DKMS_VERSION)
	@echo "nintendo dkms module installed successfully."

# INSTALL JOYCOND
install_joycond: check-local-repos
	@echo "Using local joycond repository..."
	@if [ ! -f "$(JOYCOND_DIR)/CMakeLists.txt" ]; then \
		echo "Error: joycond directory does not contain CMakeLists.txt"; \
		exit 1; \
	fi
	@cd $(JOYCOND_DIR) && cmake .
	@cd $(JOYCOND_DIR) && sudo make install
	@cd $(JOYCOND_DIR) && sudo systemctl enable --now joycond
	@echo "joycond installed and started successfully."

# INSTALL UBUNTU SYSTEM DEPENDENCIES
install-hid-deps:
	sudo apt-get install -y \
		libhidapi-dev \
		libhidapi-hidraw0 \
		libhidapi-libusb0

# INSTALL JOYCON UDEV RULES
install-udev-rules:
	@echo "Installing udev rules..."
	@sudo cp $(UDEV_RULES) $(RULES_DIR)
	@sudo udevadm control --reload-rules && sudo udevadm trigger
	@echo "Udev rules installed successfully."

# DELETE TEMPORARY FILES
clean:
	@echo "Cleaning up temporary files..."
	@rm -rf $(TEMP_DIR)
	@echo "Cleanup completed."

# INSTALL ALL TARGET
install: install_nintendo install_joycond install-hid-deps install-udev-rules
	@echo "All dependencies installed successfully."
