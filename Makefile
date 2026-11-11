BUILD ?= build
MUJOCO_DIR ?= $(shell python3 -c "import mujoco,os;print(os.path.dirname(mujoco.__file__))")

.PHONY: build test bench clean
build:
	cmake -S . -B $(BUILD) -DMUJOCO_DIR=$(MUJOCO_DIR)
	cmake --build $(BUILD) -j4
test:
	python3 -m pytest -q tests
bench: build
	python3 -m ctrlbench.run_all --build $(BUILD)
clean:
	rm -rf $(BUILD)
