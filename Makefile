#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
.PHONY: robolab capx capx-sim robodojo

ROBOLAB_URL := https://github.com/NVlabs/RoboLab.git
ROBOLAB_COMMIT := 9db0aaf09d9fe5d4f37b168320788258c7012463
CAPX_URL := https://github.com/capgym/cap-x.git
CAPX_COMMIT := 53e9966d7a8e2fa7494676772bccc35280f5c0ed
ROBODOJO_URL := https://github.com/RoboDojo-Benchmark/RoboDojo.git
ROBODOJO_COMMIT := 09bd5f5e762b949f5858ece31347ce62057b3a5e

define pinned
	@if [ -e $(2)/.git ]; then \
	  head=$$(git -C $(2) rev-parse HEAD); \
	  [ "$$head" = "$(3)" ] || { echo "$(2) is at $$head, not the pinned $(3): check out the pin or move it aside" >&2; exit 1; }; \
	  echo "$(2) is at the pinned $(3)"; \
	else \
	  git clone $(1) $(2) && git -C $(2) -c advice.detachedHead=false checkout $(3); \
	fi
endef

robolab:
	$(call pinned,$(ROBOLAB_URL),external/robolab,$(ROBOLAB_COMMIT))
	git -C external/robolab lfs pull

capx:
	$(call pinned,$(CAPX_URL),external/cap-x,$(CAPX_COMMIT))

CAPX_SIM_REPOS := capx/third_party/LIBERO-PRO capx/third_party/libero_dependencies/robosuite \
	capx/third_party/robosuite capx/third_party/sam3
capx-sim: capx
	git -C external/cap-x submodule update --init --recursive -- $(CAPX_SIM_REPOS)

robodojo:
	$(call pinned,$(ROBODOJO_URL),external/robodojo,$(ROBODOJO_COMMIT))
	git -C external/robodojo submodule update --init --recursive
