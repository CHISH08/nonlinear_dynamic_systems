SHELL := /bin/bash

build:
	colcon build --symlink-install

clean:
	rm -rf build install log .colcon

run:
	@env -i \
	  HOME="$(HOME)" USER="$(USER)" SHELL="/bin/bash" \
	  PATH="/usr/sbin:/usr/bin:/bin" \
	  DISPLAY="$(DISPLAY)" XDG_RUNTIME_DIR="$(XDG_RUNTIME_DIR)" \
	  WAYLAND_DISPLAY="$(WAYLAND_DISPLAY)" DBUS_SESSION_BUS_ADDRESS="$(DBUS_SESSION_BUS_ADDRESS)" \
	  bash -lc 'scripts/run_display.sh'
