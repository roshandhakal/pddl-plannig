## ==== Core Arguments and Parameters ====
MAJOR ?= 0
MINOR ?= 1
VERSION = $(MAJOR).$(MINOR)
APP_NAME ?= pddl-plan
NUM_BUILD_CORES ?= $(shell grep -c ^processor /proc/cpuinfo)



# Docker args
DISPLAY ?= :0.0
XPASSTHROUGH ?= false
DOCKER_FILE_DIR = "."
DOCKERFILE = ${DOCKER_FILE_DIR}/Dockerfile
IMAGE_NAME = ${APP_NAME}
DOCKER_CORE_VOLUMES = \
	--env XPASSTHROUGH=$(XPASSTHROUGH) \
	--env DISPLAY=$(DISPLAY) \
	--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"
DOCKER_BASE = docker run --init --ipc=host \
	$(DOCKER_ARGS) $(DOCKER_CORE_VOLUMES) \
	${IMAGE_NAME}:${VERSION}
DOCKER_PYTHON = $(DOCKER_BASE) python3


.PHONY: help
help::
	@echo ''
	@echo 'Usage: make [TARGET] [EXTRA_ARGUMENTS]'
	@echo 'Targets:'
	@echo '  help		display this help message'
	@echo '  build		build docker image (incremental)'
	@echo '  rebuild	build docker image from scratch'
	@echo '  kill		close all project-related docker containers'
	@echo 'Extra Arguments:'
	@echo '  XPASSTHROUGH	[false] use the local X server for visualization'
	@echo '  DOCKER_ARGS	[] extra arguments passed to Docker; -it will enable "interactive mode"'
	@echo ''


## ==== Helpers for setting up the environment ====

define xhost_activate
	@echo "Enabling local xhost sharing:"
	@echo "  Display: $(DISPLAY)"
	@-DISPLAY=$(DISPLAY) xhost  +
	@-xhost  +
endef

xhost-activate:
	$(call xhost_activate)


## ==== Build targets ====

.PHONY: build
build:
	@echo "Building the Docker container"
	@docker build -t ${IMAGE_NAME}:${VERSION} \
		--build-arg NUM_BUILD_CORES=$(NUM_BUILD_CORES) \
		-f ./${DOCKERFILE} .


.PHONY: plan
plan: build
	@$(call xhost_activate)
	@$(DOCKER_PYTHON) -m plan.scripts.run

