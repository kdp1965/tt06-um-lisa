all:
	rm -rf runs && nix-shell ${OPENLANE2_ROOT}/shell.nix --run "python build.py"

